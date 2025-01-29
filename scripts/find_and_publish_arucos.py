#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Quaternion
from icuas25_msgs.msg import TargetInfo
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf_transformations as tf
import tf2_ros
import time

class ArucoProcessor(Node):
    def __init__(self):
        super().__init__('aruco_processor')

        self.publisher = self.create_publisher(TargetInfo, '/target_info', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription_markers = self.create_subscription(
            ArucoMarkers, '/cf_1/aruco_markers', self.markers_callback, 10)
        
        self.subscription_drone_pose = self.create_subscription(
            PoseStamped, '/cf_1/pose', self.drone_pose_callback, 10)

        self.drone_pose = None
        self.active_markers = {}  # Key: marker_id, Value: (last_detection_time, pose)
        self.timer = self.create_timer(0.1, self.check_expired_markers)

    def drone_pose_callback(self, msg):
        self.publish_camera_tf()
        self.drone_pose = msg.pose

    def markers_callback(self, msg):
        current_time = time.time()
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            # Update the active markers dictionary with current time and pose
            self.active_markers[marker_id] = (current_time, pose)
            # Publish TF for this marker
            self.publish_tf(pose, marker_id)

    def check_expired_markers(self):
        current_time = time.time()
        expired_ids = []
        for marker_id, (last_time, pose) in self.active_markers.items():
            if current_time - last_time > 3.0:
                self.publish_target_info(pose.position.x, pose.position.y, pose.position.z, marker_id)
                expired_ids.append(marker_id)
        # Remove expired markers from the active tracking
        for marker_id in expired_ids:
            del self.active_markers[marker_id]

    def publish_target_info(self, x, y, z, marker_id):
        transform_aruco = self.tf_buffer.lookup_transform("world", f"aruco_{marker_id}", rclpy.time.Time())

        target_info = TargetInfo()
        target_info.id = int(marker_id)
        target_info.location.x = transform_aruco.transform.translation.x
        target_info.location.y = transform_aruco.transform.translation.y
        target_info.location.z = transform_aruco.transform.translation.z
        self.publisher.publish(target_info)

    def publish_tf(self, pose, aruco_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "cf_1/camera_frame"
        t.child_frame_id = f"aruco_{aruco_id}"
        
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)
    
    def publish_camera_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "cf_1"
        t.child_frame_id = "cf_1/camera_frame"

        t.transform.translation.x = 0.0  
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.01

        q = tf.quaternion_from_euler(1.57, 0, 1.57)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ArucoProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node terminated")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()