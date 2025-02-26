#!/usr/bin/env python3
import os
import rclpy
import ast
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path

class ManageTrajectoriesNode(Node):
    def __init__(self, drone_ids):
        super().__init__('manage_trajectories_node')
        self.drone_ids = drone_ids

        self.poses = {}
        self.batteries = {}
        self.trajectories = {}
        self.trajectories_id = {}
        self.trajectories_path = {}

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'

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
            self.get_logger().info(f'Inscrito nos tópicos do drone {drone_id}')

        # Timer para processamento periódico dos dados
        self.timer = self.create_timer(1.0, self.timer_callback)

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg

    def trajectory_callback(self, msg, drone_id):
        self.trajectories[drone_id] = msg.data

    def trajectory_ids_callback(self, msg, drone_id):
        self.trajectories_id[drone_id] = ast.literal_eval(msg.data)

    def trajectory_path_callback(self, msg, drone_id):
        self.trajectories_path[drone_id] = msg.poses

    def timer_callback(self):
        for drone_id in self.drone_ids:
            pose = self.get_latest_pose(drone_id)
            battery = self.get_latest_battery(drone_id)
            trajectory = self.get_latest_trajectory(drone_id)
            trajectory_id = self.get_latest_trajectory_id(drone_id)
            trajectory_path = self.get_latest_trajectory_path(drone_id)

            if pose:
                print_pose = (round(pose.pose.position.x, 2), round(pose.pose.position.y, 2), round(pose.pose.position.z, 2))
            else:
                print_pose = None
            
            if battery:
                print_battery = round(battery.percentage, 2)
            else:
                print_battery = None
            
            if (trajectory and trajectory_id and trajectory_path) is not None:
                print_trajectory = True
            else:
                print_trajectory = False

            self.get_logger().info(
                f'\nDrone {drone_id}:\n Pose: {print_pose},\n Battery: {print_battery},\n Received Traj: {print_trajectory}'
            )

    def get_latest_pose(self, drone_id):
        return self.poses.get(drone_id)

    def get_latest_battery(self, drone_id):
        return self.batteries.get(drone_id)

    def get_latest_trajectory(self, drone_id):
        return self.trajectories.get(drone_id)

    def get_latest_trajectory_id(self, drone_id):
        return self.trajectories_id.get(drone_id)

    def get_latest_trajectory_path(self, drone_id):
        return self.trajectories_path.get(drone_id)
    
def main(args=None):
    rclpy.init(args=args)
    
    num_robots = int(os.getenv("NUM_ROBOTS", "5"))
    drone_ids = list(range(1, num_robots + 1))
    
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