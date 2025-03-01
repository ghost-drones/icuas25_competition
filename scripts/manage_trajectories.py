#!/usr/bin/env python3
import os
import math
import rclpy
import ast
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path

from crazyflie_interfaces.srv import GoTo, Land
from tf_transformations import euler_from_quaternion

import numpy as np

class ManageSwarmNode(Node):
    def __init__(self, drone_ids):
        super().__init__('manage_swarm_node')
        self.drone_ids = drone_ids

        self.poses = {}
        self.batteries = {}
        self.trajectories = {}       # Primary action (string) for each drone
        self.trajectories_id = {}    # Elements that determine the waypoints
        self.trajectories_path = {}  # List of PoseStamped (waypoints)
        self.base_position = np.array([0.0, 0.0, 0.0])  # Base position
        self.battery_threshold = 20.0  # Battery threshold in percentage
        self.avg_speed = 2.0  # Average speed of the drone in m/s

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

        # Clients for go_to and land services
        self.go_to_clients = {}
        self.land_clients = {}

        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'
            go_to_service = f'/cf_{drone_id}/go_to'
            land_service = f'/cf_{drone_id}/land'

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
            self.create_subscription(
                String,
                trajectory_topic,
                lambda msg, id=drone_id: self.trajectory_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                String,
                trajectory_ids_topic,
                lambda msg, id=drone_id: self.trajectory_ids_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                Path,
                trajectory_path_topic,
                lambda msg, id=drone_id: self.trajectory_path_callback(msg, id),
                qos_profile
            )

            self.go_to_clients[drone_id] = self.create_client(GoTo, go_to_service)
            self.land_clients[drone_id] = self.create_client(Land, land_service)

            self.get_logger().info(f'Subscribed to topics and services for drone {drone_id}')

        # Trajectory management state (global for all drones)
        self.current_step = 0  # Index of the primary step (element of trajectories_id)
        # If the step is a list, it indicates the current index in the sublist for each drone.
        self.current_substep = {drone_id: 0 for drone_id in self.drone_ids}
        # Indicates whether the command for the current step/substep has already been sent.
        self.command_sent = {drone_id: False for drone_id in self.drone_ids}

        hz = 10
        T = 1 / hz
        self.timer = self.create_timer(T, self.timer_callback)
    
    def start_charging(self, drone_id):
    """
    Start the charging process for a drone.
    """
    if drone_id not in self.batteries:
        self.get_logger().error(f"No battery data available for drone {drone_id}.")
        return

    current_battery = self.batteries[drone_id].percentage * 100  # Convert to percentage
    if current_battery >= self.charging_target:
        self.get_logger().info(f"Drone {drone_id} is already fully charged.")
        self.proceed_with_trajectory(drone_id)
        return

    # Calculate the time required to fully charge the battery
    time_to_charge = (self.charging_target - current_battery) * self.charging_time_per_percentage
    self.get_logger().info(f"Drone {drone_id} will be fully charged in {time_to_charge} seconds.")

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

        current_battery = self.batteries[drone_id].percentage * 100  # Convert to percentage
        if current_battery >= self.charging_target:
            self.get_logger().info(f"Drone {drone_id} is fully charged. Proceeding with the trajectory.")
            self.proceed_with_trajectory(drone_id)
        else:
            self.get_logger().info(f"Drone {drone_id} is not fully charged. Continuing to charge.")
            # Continue charging for another period
            self.start_charging(drone_id)

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

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg

    def trajectory_callback(self, msg, drone_id):
        # Store the primary action as a string
        self.trajectories[drone_id] = msg.data

    def trajectory_ids_callback(self, msg, drone_id):
        self.trajectories_id[drone_id] = ast.literal_eval(msg.data)

    def trajectory_path_callback(self, msg, drone_id):
        self.trajectories_path[drone_id] = msg.poses

    def get_yaw_from_pose(self, pose: PoseStamped) -> float:
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        # Convert quaternion to Euler (roll, pitch, yaw) in radians
        _, _, yaw = euler_from_quaternion(quat)
        return yaw

    def send_go_to(self, drone_id, pose: PoseStamped, duration_sec, group_mask=0, relative=False):
        if drone_id not in self.go_to_clients:
            self.get_logger().error(f'GoTo client not found for drone {drone_id}')
            return

        client = self.go_to_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service /cf_{drone_id}/go_to unavailable')
            return

        goal = pose.pose.position
        yaw = self.get_yaw_from_pose(pose)

        request = GoTo.Request()
        request.group_mask = group_mask
        request.relative = relative
        request.goal = goal
        request.yaw = yaw
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        client.call_async(request)

    def send_land(self, drone_id, height, duration_sec, group_mask=0):
        if drone_id not in self.land_clients:
            self.get_logger().error(f'Land client not found for drone {drone_id}')
            return

        client = self.land_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service /cf_{drone_id}/land unavailable')
            return

        request = Land.Request()
        request.group_mask = group_mask
        request.height = height
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        client.call_async(request)

    def is_pose_reached(self, current_pose: PoseStamped, target_pose: PoseStamped, tolerance=0.2) -> bool:
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        dz = current_pose.pose.position.z - target_pose.pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < tolerance

    def get_flat_index(self, drone_id, step, substep):
        """
        Calculate the flattened index in the trajectories_path list based on the step (element of trajectories_id)
        and the substep. For each previous element, add 1 if it's an int or the length of the list if it's a list.
        For the current element, add the value of substep (or 0 if it's an int).
        """
        steps = self.trajectories_id[drone_id]
        flat_index = 0
        for i in range(step):
            elem = steps[i]
            if isinstance(elem, list):
                flat_index += len(elem)
            else:
                flat_index += 1
        current_elem = steps[step]
        if isinstance(current_elem, list):
            flat_index += substep
        else:
            flat_index += 0
        return flat_index

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
        battery_percentage = self.batteries[drone_id].percentage * 100  # Convert to percentage

        # Compute total travel distance (sum of Euclidean distances between consecutive waypoints)
        total_distance = np.linalg.norm(current_position - waypoints[0]) + sum(np.linalg.norm(waypoints[i+1] - waypoints[i]) for i in range(len(waypoints) - 1))
        
        # Compute total time for trajectory
        total_time = total_distance / self.avg_speed  # Time in seconds

        # Compute battery consumption for the planned trajectory
        battery_needed_for_trajectory = (total_time / (4/3)) * 10  # Percentage of battery used

        # Compute the distance from the last waypoint to the base
        return_distance = np.linalg.norm(waypoints[-1] - self.base_position)
        
        # Compute return time
        return_time = return_distance / self.avg_speed

        # Compute battery needed for return to base
        battery_needed_for_return = (return_time / (4/3)) * 10  # Percentage of battery used
        
        # Check if, after completing the trajectory, the drone has enough battery to return
        remaining_battery_after_trajectory = battery_percentage - battery_needed_for_trajectory

        if remaining_battery_after_trajectory >= battery_needed_for_return + self.battery_threshold:
            return False  # Drone can complete the trajectory first
        else:
            return True  # Drone should return to base immediately

    def timer_callback(self):
        # Check if the necessary data has been received for ALL drones.
        for drone_id in self.drone_ids:
            if (drone_id not in self.poses or 
                drone_id not in self.batteries or 
                drone_id not in self.trajectories or
                drone_id not in self.trajectories_id or 
                drone_id not in self.trajectories_path):
                return

        total_steps = len(self.trajectories_id[self.drone_ids[0]])
        if self.current_step >= total_steps:
            self.get_logger().info("All trajectories completed.")
            return

        finished_current_step = {}

        for drone_id in self.drone_ids:
            steps = self.trajectories_id[drone_id]
            current_command = steps[self.current_step]
            current_pose = self.poses[drone_id]

            if isinstance(current_command, list):
                substep = self.current_substep[drone_id]
                flat_index = self.get_flat_index(drone_id, self.current_step, substep)
            else:
                flat_index = self.get_flat_index(drone_id, self.current_step, 0)

            target_pose = self.trajectories_path[drone_id][flat_index]

            # Check if the drone should return to base before sending the trajectory
            if self.should_return_to_base(drone_id):
                self.get_logger().info(f"Drone {drone_id} has low battery. Returning to base.")
                
                # Save the current trajectory as pending
                self.pending_trajectories[drone_id] = self.trajectories_path[drone_id]
                
                # Calculate the distance to the base
                current_position = np.array([
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z
                ])
                distance_to_base = np.linalg.norm(current_position - self.base_position)
                
                # Calculate the duration for the go_to command
                duration_to_base = distance_to_base / self.avg_speed
                
                # Send the drone to the base
                base_pose = PoseStamped()
                base_pose.pose.position.x = self.base_position[0]
                base_pose.pose.position.y = self.base_position[1]
                base_pose.pose.position.z = self.base_position[2]
                self.send_go_to(drone_id, base_pose, duration_to_base)
                # Land the drone at the base
                self.send_land(drone_id, height=0.0, duration_sec=10.0)
                # Start charging the drone
                self.start_charging(drone_id)
                continue

            # Send the command only if it hasn't been sent yet
            if not self.command_sent[drone_id]:
                self.send_go_to(drone_id, target_pose, duration_sec=20.0)
                # Print the primary action using self.trajectories (string)
                self.command_sent[drone_id] = True

            # Check if the drone has reached the target waypoint
            if self.is_pose_reached(current_pose, target_pose):
                if isinstance(current_command, list):
                    # For secondary actions, each drone moves to the next waypoint immediately.
                    if self.current_substep[drone_id] < len(current_command) - 1:
                        self.current_substep[drone_id] += 1
                        self.command_sent[drone_id] = False
                        finished_current_step[drone_id] = False
                    else:
                        finished_current_step[drone_id] = True
                else:
                    finished_current_step[drone_id] = True
            else:
                finished_current_step[drone_id] = False

        # For primary actions, wait until ALL drones have completed the step
        if all(finished_current_step.get(d, False) for d in self.drone_ids):
            self.get_logger().info(f"Step {self.current_step} completed for all drones.")
            self.current_step += 1
            for drone_id in self.drone_ids:
                self.command_sent[drone_id] = False
                self.current_substep[drone_id] = 0

def main(args=None):
    rclpy.init(args=args)
    
    num_robots = int(os.getenv("NUM_ROBOTS", "5"))
    drone_ids = list(range(1, num_robots + 1))
    
    node = ManageSwarmNode(drone_ids)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()