import rclpy
from rclpy.node import Node
import numpy as np
import os
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
from icuas25_msgs.srv import BatteryService
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Point

class ManageBattery(Node):

  def __init__(self, drone_ids):
    super().__init__('manage_battery')
    
    self.drone_ids = drone_ids
    self.poses = {}
    self.batteries = {}
    
    self.srv_check_cluster_battery = self.create_service(BatteryService, '/ghost/check_cluster_battery', self.handle_request_cluster)
    self.srv_check_full_battery = self.create_service(BatteryService, '/ghost/check_full_battery', self.handle_request_full)
    
    self.battery_threshold = 20.0  # Battery threshold in percentage
    self.avg_vel = 0.7

    # Charging parameters
    # Se leva 10 min pra carregar de de 0 a 90%, então o parâmetro abaixo seria igual a (10*60)/90 segundos 
    self.charging_time_per_percentage = 10.0  # Time (in seconds) to charge 1% of the battery
    self.charging_timers = {drone_id: None for drone_id in self.drone_ids}  # Timers for charging
    self.charging_target = 100.0  # Target battery percentage for full charge
    
    # Store the trajectory that couldn't be completed due to low battery
    self.pending_trajectories = {drone_id: None for drone_id in self.drone_ids}
    qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

    for drone_id in self.drone_ids:
      pose_topic = f'/cf_{drone_id}/pose'
      battery_topic = f'/cf_{drone_id}/battery_status'
      self.create_subscription(
                PoseStamped,
                pose_topic,
                lambda msg, id=drone_id: self.pose_callback(msg, id),
                qos_profile
            )
      self.create_subscription(
                BatteryState,
                battery_topic,
                lambda msg, id=drone_id: self.battery_callback(msg, id),
                qos_profile
            )

  def battery_callback(self, msg, drone_id):
    self.batteries[drone_id] = msg
  def pose_callback(self, msg, drone_id):
    self.poses[drone_id] = msg
  
  def handle_request_cluster(self, request, response:BatteryService):
      # Lógica de exemplo: Definir a mensagem e o status da bateria
      
      achieves,msg = self.check_cluster() # informa se consegue, se sim, retorna true,"", se não, retorna false,"time (em segundos)"
      
      response.msg = String(data=msg)
      response.full_battery = Bool(data=achieves)  # Ou False, dependendo da lógica
      
      self.get_logger().info('Requisição recebida. Enviando resposta.')
      if not achieves:
        time.sleep(0.5)
        self.start_charging()
      return response
  
  def check_cluster(self):
    current_battery = 100.0
    for drone_id in self.drone_ids:
      local_battery = self.batteries[drone_id].percentage
      if local_battery < current_battery:
        current_battery = local_battery
    
    time_to_travel_the_cluster_trajectorie = 0
    # tempo = tempo de perseguir uma trajetória
    
    for drone_id in self.drone_ids:
      position = self.poses[drone_id].pose.position
      if isinstance(position,Point):
        current_position = np.array([
        position.x,
        position.y,
        position.z
        ])
      
      waypoints = np.array([
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        for pose in self.trajectories_path[drone_id]
      ])
    
    
    # tempo para cerragar a bateria no posto
    
  def calc_duration(self, pos_1:PoseStamped, pos_2:PoseStamped) -> float:
    if isinstance(pos_1,PoseStamped) and isinstance(pos_2,PoseStamped):
      position_01 = pos_1.pose.position
      position_02 = pos_2.pose.position
      pos_1 = np.array([position_01.x, position_01.y, position_01.z], dtype='float32')
      pos_2 = np.array([position_02.x, position_02.y, position_02.z], dtype='float32')

      duration = np.linalg.norm(pos_2 - pos_1) / self.avg_vel

      return float(duration)

  def start_charging(self):
      """
      Start the charging process for a drone.
      """
      if drone_id not in self.batteries:
          self.get_logger().error(f"No battery data available for drone {drone_id}.")
          return

      current_battery = self.batteries[drone_id].percentage
      if current_battery >= self.charging_target:
          self.get_logger().info(f"Drone {drone_id} is already fully charged.")
          self.proceed_with_trajectory(drone_id)
          return

      
      time_to_charge = (self.charging_target - current_battery) * self.charging_time_per_percentage
      self.get_logger().info(f"Drone {drone_id} will be fully charged in {time_to_charge} seconds.")

      time.sleep(time_to_charge*0.9)
      # Set a timer to check the battery status after the charging time
      self.charging_timers[drone_id] = self.create_timer(
          time_to_charge,
          lambda: self.check_battery_and_proceed(drone_id)
      )

  def check_battery_and_proceed(self, drone_id):
      """
      Check if the drone's battery is fully charged. If so, proceed with the trajectory.
      If not, continue charging.
      """
      if drone_id not in self.batteries:
          self.get_logger().error(f"No battery data available for drone {drone_id}.")
          return

      current_battery = self.batteries[drone_id].percentage 
      if current_battery >= self.charging_target:
          self.get_logger().info(f"Drone {drone_id} is fully charged. Proceeding with the trajectory.")
          self.proceed_with_trajectory(drone_id)
      else:
          self.get_logger().info(f"Drone {drone_id} is not fully charged. Continuing to charge.")
          # Continue charging for another period
          #self.start_charging(drone_id)

  def proceed_with_trajectory(self, drone_id):
      """
      Proceed with the drone's trajectory if the battery is fully charged.
      """
      if drone_id not in self.trajectories_path:
          self.get_logger().error(f"No trajectory data available for drone {drone_id}.")
          return

      # Reset the charging timer
      if self.charging_timers[drone_id]:
          self.charging_timers[drone_id].cancel()
          self.charging_timers[drone_id] = None

      # Check if there is a pending trajectory for this drone
      if self.pending_trajectories[drone_id] is not None:
          self.get_logger().info(f"Drone {drone_id} is proceeding with its pending trajectory.")
          # Restore the pending trajectory
          self.trajectories_path[drone_id] = self.pending_trajectories[drone_id]
          self.pending_trajectories[drone_id] = None
          # Reset the current step and substep for this drone
          self.current_step = 0
          self.current_substep[drone_id] = 0
          self.command_sent[drone_id] = False
      else:
          self.get_logger().info(f"Drone {drone_id} has no pending trajectory.")

  def should_return_to_base(self, drone_id):
      """
      Determine if the drone should return to base based on battery consumption.

      Parameters:
      - drone_id: The ID of the drone.

      Returns:
      - bool: True if the drone should return to base immediately, False if it can complete the trajectory first.
      """
      current_position = np.array([
          self.poses[drone_id].pose.position.x,
          self.poses[drone_id].pose.position.y,
          self.poses[drone_id].pose.position.z
      ])
      waypoints = np.array([
          [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
          for pose in self.trajectories_path[drone_id]
      ])
      battery_percentage = self.batteries[drone_id].percentage # Convert to percentage

      # Compute total travel distance (sum of Euclidean distances between consecutive waypoints)
      total_distance = np.linalg.norm(current_position - waypoints[0]) + sum(np.linalg.norm(waypoints[i+1] - waypoints[i]) for i in range(len(waypoints) - 1))
      
      # Compute total time for trajectory
      total_time = total_distance / self.avg_vel  # Time in seconds

      # Compute battery consumption for the planned trajectory
      battery_needed_for_trajectory = (total_time / (4/3)) * 10  # Percentage of battery used

      # Compute the distance from the last waypoint to the base
      return_distance = np.linalg.norm(waypoints[-1] - self.base_position)
      
      # Compute return time
      return_time = return_distance / self.avg_vel

      # Compute battery needed for return to base
      battery_needed_for_return = (return_time / (4/3)) * 10  # Percentage of battery used
      
      # Check if, after completing the trajectory, the drone has enough battery to return
      remaining_battery_after_trajectory = battery_percentage - battery_needed_for_trajectory
      test = remaining_battery_after_trajectory >= battery_needed_for_return + self.battery_threshold
      print("\nTEST:", test, battery_percentage)
      if test:
          return False  # Drone can complete the trajectory first
      else:
          return True  # Drone should return to base immediately

def main(args=None):
    rclpy.init(args=args)
    
    num_robots = int(os.getenv("NUM_ROBOTS", "5"))
    drone_ids = list(range(1, num_robots + 1))
    
    node = ManageBattery(drone_ids)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()