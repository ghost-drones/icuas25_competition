#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from icuas25_msgs.msg import Waypoints

from trajectory_utils import (
    process_clusters,
    trajectory_to_path_msg,
    generate_macro_trajectory,
    translate_macro_to_drone_trajectories,
    split_list_equally,
    decode_encoded_trajectories,
    create_path_msgs_from_decoded
)

class TrajectoryBuilder(Node):
    def __init__(self):
        super().__init__('trajectory_builder')
        self.executed = False
        self.subscription = self.create_subscription(
            Waypoints,
            '/ghost/waypoints',
            self.waypoints_callback,
            10
        )
        self.drone_publishers = {}
        self.path_publishers = {}
        self.ids_publishers = {}  # Publisher para trajectory_ids
        self.encoded_trajs = {}  # Trajetórias codificadas para cada drone
        self.clusters_data = []  # Dados processados dos clusters
        
        # Variáveis para armazenar os dados decodificados
        self.decoded_instructions = None
        self.id_waypoints = None

        # Cria um timer que chama 'timer_callback' a cada 1 segundo (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('TrajectoryBuilder iniciado.')

    def waypoints_callback(self, msg):
        if self.executed:
            return
        self.executed = True
        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        self.clusters_data = process_clusters(msg, num_robots)

        self.get_logger().info("Ordem de prioridade dos clusters:")
        for idx, c in enumerate(self.clusters_data):
            self.get_logger().info(
                f"{idx+1}º: Cluster {c['cluster_id']} | Drone Cap.: {c['drone_capacity']} | "
                f"Conexões: {c['total_connections']} | Ordem: {c['order']}"
            )

        # Cria os publishers para cada drone
        for drone_id in range(num_robots):
            topic_encoded = f"/ghost/cf_{drone_id+1}/trajectory_encoded"
            self.drone_publishers[drone_id] = self.create_publisher(String, topic_encoded, 10)
            topic_path = f"/ghost/cf_{drone_id+1}/trajectory_path"
            self.path_publishers[drone_id] = self.create_publisher(Path, topic_path, 10)
            topic_ids = f"/ghost/cf_{drone_id+1}/trajectory_ids"
            self.ids_publishers[drone_id] = self.create_publisher(String, topic_ids, 10)

        # Criação do clusters_info a partir dos waypoints com atributo transition_point
        clusters_info = {0: {'prev_cluster_id': None, 'order': 0}}
        for wp in msg.waypoints:
            if hasattr(wp, 'transition_point') and wp.transition_point:
                cid = wp.cluster_id
                if cid not in clusters_info:
                    clusters_info[cid] = {'prev_cluster_id': wp.prev_cluster_id, 'order': wp.order}

        topic_encoded = "/ghost/trajectory_encoded"
        self.macro_traj_pub = self.create_publisher(String, topic_encoded, 10)
        
        macro_traj = generate_macro_trajectory(self.clusters_data, clusters_info)
        self.get_logger().info(f"Macro trajetória: {macro_traj}")
        cluster_orders = {cluster['cluster_id']: cluster['order'] for cluster in self.clusters_data}
        self.encoded_trajs = translate_macro_to_drone_trajectories(macro_traj, cluster_orders, num_robots, clusters_info)

        self.get_logger().info("Trajetória individual para cada drone:")
        for drone_id in sorted(self.encoded_trajs.keys()):
            self.get_logger().info(f"Drone {drone_id+1}: {self.encoded_trajs[drone_id]}")

        # Publica as trajetórias codificadas (JSON) para cada drone (publicação única)
        for drone_id in range(num_robots):
            msg_str = String()
            msg_str.data = json.dumps(self.encoded_trajs[drone_id])
            self.drone_publishers[drone_id].publish(msg_str)

        # Decodifica as trajetórias e armazena para publicação contínua pelo timer
        self.id_waypoints, self.decoded_instructions = decode_encoded_trajectories(self.encoded_trajs, self.clusters_data, num_robots)

        msg_str = String()
        msg_str.data = json.dumps(self.encoded_trajs[drone_id])
        self.macro_traj_pub.publish(msg_str)
        
    def timer_callback(self):
        # Publica as trajetórias (Path) de cada drone, se os dados já tiverem sido calculados
        if self.decoded_instructions is not None:
            paths = create_path_msgs_from_decoded(self.decoded_instructions, self.clusters_data)
            for drone_id, path_msg in paths.items():
                # Atualiza o timestamp de cada PoseStamped no Path
                for ps in path_msg.poses:
                    ps.header.stamp = Clock().now().to_msg()
                self.path_publishers[drone_id].publish(path_msg)
                
        # Publica os trajectory_ids para cada drone, se disponíveis
        if self.id_waypoints is not None:
            for drone_id, ids in self.id_waypoints.items():
                msg_ids = String()
                # Converte a lista (possivelmente aninhada) em JSON para publicação
                msg_ids.data = json.dumps(ids)
                self.ids_publishers[drone_id].publish(msg_ids)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando TrajectoryBuilder...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()