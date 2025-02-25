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
    split_list_equally
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
        self.encoded_trajs = {}  # Trajetórias codificadas para cada drone
        self.path_pub = self.create_publisher(Path, '/ghost/cluster_trajectory_path', 10)
        self.path_publishers = {}
        self.clusters_data = []  # Armazena os dados processados dos clusters
        
        # Variável para armazenar os dados decodificados para criação das mensagens Path
        self.decoded_instructions = None

        # Cria um timer que chama 'timer_callback' a cada 1 segundo (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('TrajectoryBuilder iniciado.')

    def waypoints_callback(self, msg):
        if self.executed:
            return
        self.executed = True
        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        clusters_data = process_clusters(msg, num_robots)
        self.clusters_data = clusters_data

        self.get_logger().info("Ordem de prioridade dos clusters:")
        for idx, c in enumerate(clusters_data):
            self.get_logger().info(
                f"{idx+1}º: Cluster {c['cluster_id']} | Drone Cap.: {c['drone_capacity']} | "
                f"Conexões: {c['total_connections']} | Ordem: {c['order']}"
            )

        # Cria os publishers para cada drone
        for drone_id in range(num_robots):
            topic = f"/ghost/cf_{drone_id}/trajectory_encoded"
            self.drone_publishers[drone_id] = self.create_publisher(String, topic, 10)
            topic_path = f"/ghost/cf_{drone_id}/trajectory_path"
            self.path_publishers[drone_id] = self.create_publisher(Path, topic_path, 10)

        # Criação do clusters_info a partir dos waypoints com atributo transition_point
        clusters_info = {0: {'prev_cluster_id': None, 'order': 0}}
        for wp in msg.waypoints:
            if hasattr(wp, 'transition_point') and wp.transition_point:
                cid = wp.cluster_id
                if cid not in clusters_info:
                    clusters_info[cid] = {'prev_cluster_id': wp.prev_cluster_id, 'order': wp.order}

        macro_traj = generate_macro_trajectory(clusters_data, clusters_info)
        self.get_logger().info(f"Macro trajetória: {macro_traj}")
        cluster_orders = {cluster['cluster_id']: cluster['order'] for cluster in clusters_data}
        encoded_trajs = translate_macro_to_drone_trajectories(macro_traj, cluster_orders, num_robots, clusters_info)
        self.encoded_trajs = encoded_trajs

        self.get_logger().info("Trajetória individual para cada drone:")
        for drone_id in sorted(encoded_trajs.keys()):
            self.get_logger().info(f"Drone {drone_id}: {encoded_trajs[drone_id]}")

        # Publica as trajetórias codificadas (JSON) para cada drone (mantém essa publicação única)
        for drone_id in range(num_robots):
            msg_str = String()
            msg_str.data = json.dumps(encoded_trajs[drone_id])
            self.drone_publishers[drone_id].publish(msg_str)

        # Em vez de publicar as mensagens Path uma única vez, armazene os dados decodificados
        decoded = self.decode_encoded_trajectories(num_robots)
        self.decoded_instructions = decoded  # Armazena para publicação contínua pelo timer

    def timer_callback(self):
        # Publica a trajetória do cluster de maior prioridade (para visualização)
        if self.clusters_data:
            cluster = self.clusters_data[0]
            path_msg = trajectory_to_path_msg(cluster['trajectory'], frame_id="world")
            # Atualiza o timestamp para o tempo atual
            path_msg.header.stamp = Clock().now().to_msg()
            self.path_pub.publish(path_msg)

        # Publica as trajetórias (Path) de cada drone, se os dados já tiverem sido calculados
        if self.decoded_instructions is not None:
            paths = self.create_path_msgs_from_decoded(self.decoded_instructions)
            for drone_id, path_msg in paths.items():
                # Atualiza o timestamp de cada PoseStamped no Path
                for ps in path_msg.poses:
                    ps.header.stamp = Clock().now().to_msg()
                self.path_publishers[drone_id].publish(path_msg)

    def get_support_waypoint(self, cluster_id):
        """
        Retorna o id do waypoint de suporte do cluster.
        Para o cluster 0, retorna 0.
        """
        if cluster_id == 0:
            return 0
        for cluster in self.clusters_data:
            if cluster['cluster_id'] == cluster_id:
                for wp in cluster['trajectory']:
                    if hasattr(wp, 'transition_point') and wp.transition_point:
                        return wp.id
        return None

    def get_cluster(self, cluster_id):
        """Retorna os dados do cluster correspondente."""
        for cluster in self.clusters_data:
            if cluster['cluster_id'] == cluster_id:
                return cluster
        return None

    def decode_encoded_trajectories(self, num_robots):
        """
        Decodifica self.encoded_trajs para gerar uma lista de instruções (waypoints)
        para cada drone.
        """
        decoded = {}
        for drone_id, instructions in self.encoded_trajs.items():
            drone_waypoints = []
            id_waypoints = []
            for instr in instructions:
                if instr.startswith("RS"):
                    cluster_id = int(instr[2:])
                    support_id = self.get_support_waypoint(cluster_id)
                    drone_waypoints.append(("RS", support_id))
                    id_waypoints.append(support_id)
                elif instr.startswith("IS"):
                    cluster_id = int(instr[2:])
                    support_id = self.get_support_waypoint(cluster_id)
                    drone_waypoints.append(("IS", support_id))
                    id_waypoints.append(support_id)
                elif instr.startswith("S"):
                    cluster_id = int(instr[1:])
                    support_id = self.get_support_waypoint(cluster_id)
                    drone_waypoints.append(("S", support_id))
                    id_waypoints.append(support_id)
                elif instr.startswith("E"):
                    cluster_id = int(instr[1:])
                    cluster = self.get_cluster(cluster_id)
                    if cluster is None:
                        segment_waypoints = []
                    else:
                        num_explorers = num_robots - cluster['order']
                        exploring_index = drone_id - cluster['order']
                        if exploring_index < 0 or exploring_index >= num_explorers:
                            segment_waypoints = []
                        else:
                            traj_ids = [wp.id for wp in cluster['trajectory']]
                            segments = split_list_equally(traj_ids, num_explorers)
                            segment_waypoints = segments[exploring_index]
                    drone_waypoints.append(("E", segment_waypoints))
                    id_waypoints.append(segment_waypoints)
            decoded[drone_id] = drone_waypoints
            self.get_logger().info(f"Decoded waypoints for drone {drone_id}: {id_waypoints}")
        return decoded

    def get_waypoint_by_id(self, wp_id):
        """
        Retorna o waypoint (objeto) a partir do id.
        Se wp_id for 0, retorna um waypoint dummy com pose (0,0,0).
        """
        if wp_id == 0:
            dummy = PoseStamped()
            dummy.pose = Pose()
            dummy.pose.position.x = 0.0
            dummy.pose.position.y = 0.0
            dummy.pose.position.z = 0.0
            return dummy
        for cluster in self.clusters_data:
            for wp in cluster['trajectory']:
                if wp.id == wp_id:
                    return wp
        return None

    def create_path_msgs_from_decoded(self, decoded, frame_id="world"):
        """
        Cria mensagens do tipo Path para cada drone a partir do dicionário 'decoded'.
        """
        paths = {}
        for drone_id, instructions in decoded.items():
            path_msg = Path()
            path_msg.header.frame_id = frame_id
            path_msg.header.stamp = Clock().now().to_msg()
            for typ, val in instructions:
                if typ in ["RS", "IS", "S"]:
                    wp_obj = self.get_waypoint_by_id(val)
                    if wp_obj is not None:
                        ps = PoseStamped()
                        ps.header.frame_id = frame_id
                        ps.header.stamp = Clock().now().to_msg()
                        ps.pose = wp_obj.pose
                        path_msg.poses.append(ps)
                elif typ == "E":
                    for wp_id in val:
                        wp_obj = self.get_waypoint_by_id(wp_id)
                        if wp_obj is not None:
                            ps = PoseStamped()
                            ps.header.frame_id = frame_id
                            ps.header.stamp = Clock().now().to_msg()
                            ps.pose = wp_obj.pose
                            path_msg.poses.append(ps)
            paths[drone_id] = path_msg
        return paths

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
