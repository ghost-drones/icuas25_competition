#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from icuas25_msgs.msg import TargetInfo
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseArray

class ArucoProcessor(Node):
    def __init__(self):
        super().__init__('aruco_processor')


        self.publisher = self.create_publisher(TargetInfo, '/target_info', 10)

        self.subscription_poses = self.create_subscription(PoseArray,'/cf_1/aruco_poses',self.poses_callback,10)

        self.subscription_markers = self.create_subscription(ArucoMarkers,'/cf_1/aruco_markers',self.markers_callback,10)

        self.target_msg = TargetInfo()

    def poses_callback(self, msg):
        if len(msg.poses) > 0:
            pose = msg.poses[0]
            self.target_msg.location = Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)
            self.get_logger().info(f"TargetInfo with PoseArray: {self.target_msg.location}")

    def markers_callback(self, msg):

        if len(msg.poses) > 0:
            self.target_msg.id = msg.marker_ids[0]
            self.get_logger().info(f"TargetInfo with ArucoMarkers: ID={self.target_msg.id}")

        self.publisher.publish(self.target_msg)

def main():
    rclpy.init()

    node = ArucoProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando o nó...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
