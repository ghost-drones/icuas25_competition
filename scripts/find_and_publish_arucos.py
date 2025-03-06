#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from icuas25_msgs.msg import TargetInfo
from ros2_aruco_interfaces.msg import ArucoMarkers
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
import tf_transformations as tf
import tf2_ros
import time

# Número de robôs definidos na variável de ambiente
num_robots = int(os.getenv("NUM_ROBOTS", "5"))

class ArucoProcessor(Node):
    def __init__(self):
        super().__init__('aruco_processor')
        
        # Publicador único para target_found
        self.publisher = self.create_publisher(TargetInfo, '/target_found', 10)
        # Publicador para marcadores RViz
        self.marker_pub = self.create_publisher(Marker, '/ghost/visualization_aruco_markers', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Dicionários para armazenar as poses dos drones e os marcadores ativos, indexados pelo ID do robô
        self.drone_pose = {}
        self.active_markers = {}  # { robot_id: { marker_id: (last_detection_time, pose) } }
        # Conjunto para armazenar os IDs dos targets já publicados
        self.published_targets = set()
        
        # Cria as assinaturas para cada crazyflie, de cf_1 até cf_{NUM_ROBOTS}
        for i in range(1, num_robots + 1):
            robot_prefix = f"cf_{i}"
            self.active_markers[i] = {}
            
            # Assinatura para os marcadores ArUco
            self.create_subscription(
                ArucoMarkers,
                f'/{robot_prefix}/aruco_markers',
                lambda msg, robot_id=i: self.markers_callback(msg, robot_id),
                10
            )
            
            # Assinatura para a pose do drone
            self.create_subscription(
                PoseStamped,
                f'/{robot_prefix}/pose',
                lambda msg, robot_id=i: self.drone_pose_callback(msg, robot_id),
                100
            )
        
        # Timer para verificar marcadores expirados (não atualizados há mais de 3 segundos)
        self.timer = self.create_timer(0.1, self.check_expired_markers)

    def drone_pose_callback(self, msg, robot_id):
        # Atualiza a pose do drone e publica a TF da câmera para o robô específico
        self.publish_camera_tf(robot_id)
        self.drone_pose[robot_id] = msg.pose

    def markers_callback(self, msg, robot_id):
        current_time = time.time()
        # Para cada marcador detectado, atualiza o dicionário e publica sua TF
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            self.active_markers[robot_id][marker_id] = (current_time, pose)
            self.publish_tf(pose, marker_id, robot_id)

    def check_expired_markers(self):
        current_time = time.time()
        # Para cada robô, verifica se algum marcador não foi atualizado há mais de 1 segundo
        for robot_id, markers in self.active_markers.items():
            expired_ids = []
            for marker_id, (last_time, pose) in markers.items():
                if current_time - last_time > 1.0:
                    # Só publica se o target ainda não foi publicado
                    if marker_id not in self.published_targets:
                        self.publish_target_info(pose.position.x, pose.position.y, pose.position.z, marker_id, robot_id)
                        self.published_targets.add(marker_id)
                    expired_ids.append(marker_id)
            for marker_id in expired_ids:
                del markers[marker_id]

    def publish_target_info(self, x, y, z, marker_id, robot_id):
        # Procura a transformação do frame do marcador específico do robô
        try:
            transform_aruco = self.tf_buffer.lookup_transform(
                "world",
                f"cf_{robot_id}/aruco_{marker_id}",
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Falha na busca da transformação para cf_{robot_id}/aruco_{marker_id}: {e}")
            return

        target_info = TargetInfo()
        target_info.id = int(marker_id)
        target_info.location.x = transform_aruco.transform.translation.x
        target_info.location.y = transform_aruco.transform.translation.y
        target_info.location.z = transform_aruco.transform.translation.z
        self.publisher.publish(target_info)

        # Publica também um marcador RViz para visualização
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"
        marker.ns = "aruco_markers"
        # Combina robot_id e marker_id para evitar conflitos (ex.: cf_2 e marker 23 => id = 2000 + 23)
        marker.id = robot_id * 1000 + marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = transform_aruco.transform.translation.x
        marker.pose.position.y = transform_aruco.transform.translation.y
        marker.pose.position.z = transform_aruco.transform.translation.z
        # Orientação neutra
        marker.pose.orientation.w = 1.0
        # Tamanho do marcador (ajustável conforme necessário)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        # Cor: vermelho
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Tempo de vida do marcador (3 segundos)
        marker.lifetime = Duration(sec=0, nanosec=0)
        self.marker_pub.publish(marker)

    def publish_tf(self, pose, aruco_id, robot_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # Frame de referência da câmera do robô específico
        t.header.frame_id = f"cf_{robot_id}/camera_frame"
        # Frame filho para o marcador, com namespace do robô
        t.child_frame_id = f"cf_{robot_id}/aruco_{aruco_id}"
        
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_camera_tf(self, robot_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # Frame base do robô
        t.header.frame_id = f"cf_{robot_id}"
        # Frame da câmera do robô
        t.child_frame_id = f"cf_{robot_id}/camera_frame"
        
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
