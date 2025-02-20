#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import os
import numpy as np

from icuas25_msgs.msg import Waypoints
from rviz_publishers import aggregate_markers

def distance_from_origin(pose):
    """Calcula a distância 3D da pose à origem."""
    return math.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)

def horizontal_distance(p1, p2):
    """Calcula a distância horizontal entre duas poses."""
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    return math.sqrt(dx*dx + dy*dy)

def full_distance(p1, p2):
    """Calcula a distância 3D entre duas poses."""
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    dz = p1.position.z - p2.position.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def compute_cumulative_distances(trajectory):
    """Retorna uma lista com as distâncias acumuladas ao longo da trajetória."""
    cum = [0.0]
    for i in range(1, len(trajectory)):
        d = full_distance(trajectory[i-1].pose, trajectory[i].pose)
        cum.append(cum[-1] + d)
    return cum

def group_clusters(msg):
    """Agrupa os waypoints por cluster_id."""
    clusters = {}
    for wp in msg.waypoints:
        clusters.setdefault(wp.cluster_id, []).append(wp)
    return clusters

def build_trajectory(waypoints):
    """
    Constrói uma trajetória que passe por TODOS os waypoints do cluster 
    (do ponto de transição inicial ao final), minimizando a distância total
    e penalizando movimentos verticais, usando apenas arestas com LOS direto.
    
    A solução é construída de forma aproximada por meio de uma heurística de inserção.
    """
    vertical_weight = 2.0  # Fator para penalizar movimentos verticais

    def cost(wpA, wpB):
        if wpB.id not in wpA.cluster_los:
            return float('inf')
        horz = horizontal_distance(wpA.pose, wpB.pose)
        vert = abs(wpA.pose.position.z - wpB.pose.position.z)
        return horz + vertical_weight * vert

    transition_waypoints = [wp for wp in waypoints if hasattr(wp, 'is_transition') and wp.is_transition]
    if len(transition_waypoints) >= 2:
        start_wp = min(transition_waypoints, key=lambda wp: distance_from_origin(wp.pose))
        end_wp = max(transition_waypoints, key=lambda wp: distance_from_origin(wp.pose))
    else:
        start_wp = min(waypoints, key=lambda wp: distance_from_origin(wp.pose))
        end_wp = max(waypoints, key=lambda wp: distance_from_origin(wp.pose))
    
    remaining = [wp for wp in waypoints if wp not in (start_wp, end_wp)]
    path = [start_wp, end_wp]

    while remaining:
        best_insertion = None
        best_candidate = None
        best_extra_cost = float('inf')
        for candidate in remaining:
            for i in range(len(path)-1):
                A = path[i]
                B = path[i+1]
                if candidate.id in A.cluster_los and B.id in candidate.cluster_los:
                    extra = cost(A, candidate) + cost(candidate, B) - cost(A, B)
                    if extra < best_extra_cost:
                        best_extra_cost = extra
                        best_candidate = candidate
                        best_insertion = i + 1
        if best_candidate is None:
            break
        path.insert(best_insertion, best_candidate)
        remaining.remove(best_candidate)
    
    if remaining:
        for candidate in remaining:
            if candidate.id in path[-1].cluster_los:
                path.append(candidate)
            else:
                path.append(candidate)
        if path[-1] != end_wp:
            path.append(end_wp)
    
    return path

def compute_cluster_priority(cluster_id, waypoints, trajectory, num_robots):
    """
    Calcula os parâmetros de prioridade para o cluster:
      - drone_capacity: número de drones que podem mapear simultaneamente
      - distance_walked: distância total entre o primeiro e o último ponto da trajetória
      - total_connections: tamanho da trajetória
    """
    drone_capacity = num_robots - waypoints[0].order
    distance_walked = compute_cumulative_distances(trajectory)[-1]
    total_connections = len(trajectory)
    return {
        'cluster_id': cluster_id,
        'trajectory': trajectory,
        'drone_capacity': drone_capacity,
        'distance_walked': distance_walked,
        'total_connections': total_connections
    }

def process_clusters(msg, num_robots):
    """Processa os clusters: agrupa, constrói trajetórias e calcula o ranking de prioridade."""
    clusters = group_clusters(msg)
    clusters_data = []
    transition_points = []
    for waypoint in msg.waypoints:
        if waypoint.transition_point:
            transition_points.append([waypoint.id, waypoint.cluster_id, waypoint.order])
    for cluster_id, wps in clusters.items():
        traj = build_trajectory(wps)
        data = compute_cluster_priority(cluster_id, wps, traj, num_robots)
        clusters_data.append(data)
    clusters_sorted = sorted(clusters_data, key=lambda c: (-c['drone_capacity'],
                                                            -c['total_connections']))
    total_clusters = len(clusters_sorted)
    for rank, cluster in enumerate(clusters_sorted):
        cluster['priority_rank'] = rank
        cluster['total_clusters'] = total_clusters
    return clusters_sorted

class TrajectoryBuilder(Node):
    def __init__(self):
        super().__init__('trajectory_builder')
        self.executed = False
        self.subscription = self.create_subscription(
            Waypoints,
            '/ghost/waypoints',
            self.waypoints_callback,
            10)
        from visualization_msgs.msg import MarkerArray  # Import local para publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/ghost/trajectory_markers', 10)
        self.get_logger().info('TrajectoryBuilder iniciado.')

    def waypoints_callback(self, msg):
        if self.executed:
            return
        self.executed = True
        num_robots = int(os.getenv("NUM_ROBOTS", "1"))
        clusters_data = process_clusters(msg, num_robots)
        self.get_logger().info("Ordem de prioridade dos clusters:")
        for idx, c in enumerate(clusters_data):
            self.get_logger().info(
                f"{idx+1}º: Cluster {c['cluster_id']} | Drone Cap.: {c['drone_capacity']} | "
                f"Conexões: {c['total_connections']}"
            )
        markers = aggregate_markers(self, clusters_data)
        from visualization_msgs.msg import MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Marcadores publicados no RViz: clusters, pontos de transição, trajetórias ótimas, rota de retorno e IDs das poses.")
        rclpy.shutdown()

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
