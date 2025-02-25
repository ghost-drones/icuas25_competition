#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

class ManageTrajectoriesNode(Node):
    def __init__(self, drone_ids):
        super().__init__('manage_trajectories_node')
        self.drone_ids = drone_ids

        self.poses = {}
        self.batteries = {}
        self.trajectories = {}

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'

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
            self.get_logger().info(f'Inscrito nos tópicos do drone {drone_id}')

        # Timer para processamento periódico dos dados (a lógica pode ser expandida)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg
        self.get_logger().debug(f'Nova pose recebida do drone {drone_id}')

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg
        self.get_logger().debug(f'Novo status de bateria recebido do drone {drone_id}')

    def trajectory_callback(self, msg, drone_id):
        self.trajectories[drone_id] = msg.data
        self.get_logger().debug(f'Nova trajetória codificada recebida do drone {drone_id}')

    def timer_callback(self):
        for drone_id in self.drone_ids:
            pose = self.get_latest_pose(drone_id)
            battery = self.get_latest_battery(drone_id)
            trajectory = self.get_latest_trajectory(drone_id)
            self.get_logger().debug(
                f'Drone {drone_id}: Pose: {pose}, Bateria: {battery}, Trajetória: {trajectory}'
            )

    def get_latest_pose(self, drone_id):
        return self.poses.get(drone_id)

    def get_latest_battery(self, drone_id):
        return self.batteries.get(drone_id)

    def get_latest_trajectory(self, drone_id):
        return self.trajectories.get(drone_id)

def main(args=None):
    rclpy.init(args=args)
    
    # Defina os IDs dos drones (por exemplo, de 1 a 4; ajuste para 5 ou 6 se necessário)
    drone_ids = [0,1,2,3,4]
    node = ManageTrajectoriesNode(drone_ids)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
