#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import os
import json

from std_msgs.msg import String  # Publica a trajetória individual (em JSON)
from visualization_msgs.msg import MarkerArray
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
    Constrói uma trajetória aproximada para o cluster usando uma heurística de inserção.
    """
    vertical_weight = 2.0

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
    Calcula os parâmetros de prioridade para o cluster.
    """
    drone_capacity = num_robots - waypoints[0].order
    distance_walked = compute_cumulative_distances(trajectory)[-1]
    total_connections = len(trajectory)
    return {
        'cluster_id': cluster_id,
        'trajectory': trajectory,
        'drone_capacity': drone_capacity,
        'distance_walked': distance_walked,
        'total_connections': total_connections,
        'order': waypoints[0].order
    }

def process_clusters(msg, num_robots):
    """
    Processa os clusters: agrupamento, construção de trajetórias e cálculo de prioridades.
    """
    clusters = group_clusters(msg)
    clusters_data = []
    for cluster_id, wps in clusters.items():
        traj = build_trajectory(wps)
        data = compute_cluster_priority(cluster_id, wps, traj, num_robots)
        clusters_data.append(data)
    clusters_sorted = sorted(clusters_data, key=lambda c: (-c['drone_capacity'], -c['total_connections']))
    total_clusters = len(clusters_sorted)
    for rank, cluster in enumerate(clusters_sorted):
        cluster['priority_rank'] = rank
        cluster['total_clusters'] = total_clusters
    return clusters_sorted

def get_cluster_chain(cid, clusters_info):
    """
    Retorna a cadeia de suporte para um cluster, ex: [1, 5, 0].
    """
    if cid == 0:
        return [0]
    chain = [cid]
    visited = {cid}
    current = cid
    while True:
        if current not in clusters_info:
            chain.append(0)
            break
        parent = clusters_info[current]['prev_cluster_id']
        if parent is None:
            break
        if parent == 0:
            chain.append(0)
            break
        if parent in visited:
            chain.append(f"(ciclo {parent})")
            break
        chain.append(parent)
        visited.add(parent)
        current = parent
    return chain

def get_cluster_chains_str(msg):
    """
    Retorna uma string com a relação total dos clusters, linha a linha.
    """
    clusters_info = {}
    order_list = []
    clusters_info[0] = {'prev_cluster_id': None, 'order': 0}
    order_list.append(0)
    for wp in msg.waypoints:
        if wp.transition_point:
            cid = wp.cluster_id
            if cid not in clusters_info:
                clusters_info[cid] = {'prev_cluster_id': wp.prev_cluster_id, 'order': wp.order}
                order_list.append(cid)
    lines = ["Relação total dos clusters:"]
    for cid in order_list:
        chain = get_cluster_chain(cid, clusters_info)
        lines.append(f"Cluster {cid}: {chain}")
    return "\n".join(lines)

def generate_macro_trajectory(clusters_data, clusters_info):
    """
    Gera uma macro trajetória única representando a ida e a volta dos clusters.
    """
    def get_chain(cid):
        if cid == 0:
            return []
        chain = [cid]
        visited = {cid}
        current = cid
        while True:
            if current not in clusters_info:
                break
            parent = clusters_info[current]['prev_cluster_id']
            if parent is None or parent == 0:
                break
            if parent in visited:
                chain.append(f"(ciclo {parent})")
                break
            chain.append(parent)
            visited.add(parent)
            current = parent
        return chain

    macro_chain = ["E0", "RS0"]  # Inicia na base
    for cluster in clusters_data:
        cid = cluster['cluster_id']
        if cid == 0:
            continue
        chain = get_chain(cid)
        if not chain:
            segment = ["IS" + str(cid), "E" + str(cid), "RS" + str(cid), "RS0"]
        else:
            reversed_chain = list(reversed(chain))
            outbound = ["IS" + str(x) for x in reversed_chain] + ["E" + str(cid)]
            inbound = ["RS" + str(x) for x in reversed(reversed_chain)] + ["RS0"]
            segment = outbound + inbound
        macro_chain.extend(segment)
    return macro_chain

def translate_macro_to_drone_trajectories(macro_traj, cluster_orders, num_drones):
    """
    Traduz a macro trajetória para trajetórias individuais.
    Para cada "E<cid>": se o cluster tiver ordem n > 0, drones com índice < n recebem "S<cid>".
    """
    drone_trajs = {i: [] for i in range(num_drones)}
    for instr in macro_traj:
        if instr.startswith("E"):
            cid_str = instr[1:]
            try:
                cid = int(cid_str)
            except ValueError:
                cid = None
            if cid is not None and cid in cluster_orders and cluster_orders[cid] > 0:
                order = cluster_orders[cid]
                for i in range(num_drones):
                    if i < order:
                        drone_trajs[i].append("S" + cid_str)
                    else:
                        drone_trajs[i].append(instr)
            else:
                for i in range(num_drones):
                    drone_trajs[i].append(instr)
        else:
            for i in range(num_drones):
                drone_trajs[i].append(instr)
    return drone_trajs

def create_base_waypoint():
    """
    Cria o waypoint base (ponto de suporte estacionário).
    """
    class BaseWaypoint:
        pass
    base = BaseWaypoint()
    base.id = 0
    class DummyPosition:
        def __init__(self, x, y, z):
            self.x = x; self.y = y; self.z = z
    class DummyPose:
        def __init__(self, x, y, z):
            self.position = DummyPosition(x, y, z)
    base.pose = DummyPose(0.0, 0.0, 0.0)
    base.order = 0
    return base

class TrajectoryBuilder(Node):
    def __init__(self):
        super().__init__('trajectory_builder')
        self.executed = False
        self.subscription = self.create_subscription(
            Waypoints,
            '/ghost/waypoints',
            self.waypoints_callback,
            10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ghost/trajectory_markers', 10)
        # Publishers para trajetórias individuais
        self.drone_publishers = {}
        self.drone_trajs = None  # Será definido na callback
        self.timer = None
        print('TrajectoryBuilder iniciado.')

    def waypoints_callback(self, msg):
        if self.executed:
            return
        self.executed = True
        num_robots = int(os.getenv("NUM_ROBOTS", "5"))
        clusters_data = process_clusters(msg, num_robots)
        
        # Constrói dicionário de clusters para suporte
        clusters_info = {}
        clusters_info[0] = {'prev_cluster_id': None, 'order': 0}
        for wp in msg.waypoints:
            if wp.transition_point:
                cid = wp.cluster_id
                if cid not in clusters_info:
                    clusters_info[cid] = {'prev_cluster_id': wp.prev_cluster_id, 'order': wp.order}
        
        print("Ordem de prioridade dos clusters:")
        for idx, c in enumerate(clusters_data):
            chain = get_cluster_chain(c['cluster_id'], clusters_info)
            print(
                f"{idx+1}º: Cluster {c['cluster_id']} | Drone Cap.: {c['drone_capacity']} | "
                f"Conexões: {c['total_connections']} | Ordem: {c['order']} | Suporte: {chain}"
            )
        
        chains_str = get_cluster_chains_str(msg)
        
        macro_traj = generate_macro_trajectory(clusters_data, clusters_info)
        #print("Macro Trajetória da Missão:")
        #print(str(macro_traj))
        
        cluster_orders = {cluster['cluster_id']: cluster['order'] for cluster in clusters_data}
        self.drone_trajs = translate_macro_to_drone_trajectories(macro_traj, cluster_orders, num_robots)
        print("Trajetória individual para cada drone:")
        for drone_id in sorted(self.drone_trajs.keys()):
            print(f"Drone {drone_id}: {self.drone_trajs[drone_id]}")
        
        # Cria publishers para cada drone: tópicos /ghost/cf_x/trajectory_code
        for drone_id in range(num_robots):
            topic = f"/ghost/cf_{drone_id}/trajectory_code"
            self.drone_publishers[drone_id] = self.create_publisher(String, topic, 10)
        
        # Armazena os markers para publicação periódica
        markers = aggregate_markers(self, clusters_data)
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_array = marker_array  # Armazena para publicação no timer
        
        # Inicia o timer para publicar as trajetórias e os markers a 1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Publica a trajetória de cada drone
        for drone_id, pub in self.drone_publishers.items():
            msg = String()
            msg.data = json.dumps(self.drone_trajs[drone_id])
            pub.publish(msg)

        # Publica os markers também a 1Hz
        self.marker_pub.publish(self.marker_array)

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
