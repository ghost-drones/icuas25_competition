#!/usr/bin/env python3
import math
import os
import json
import re
from std_msgs.msg import String  # Publica a trajetória individual (em JSON)
from visualization_msgs.msg import MarkerArray
from icuas25_msgs.msg import Waypoints
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.clock import Clock

def horizontal_distance(p1, p2):
    """Calcula a distância horizontal entre duas poses."""
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    return math.sqrt(dx * dx + dy * dy)

def full_distance(p1, p2):
    """Calcula a distância 3D entre duas poses."""
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    dz = p1.position.z - p2.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def compute_cumulative_distances(trajectory):
    """Retorna uma lista com as distâncias acumuladas ao longo da trajetória."""
    cum = [0.0]
    for i in range(1, len(trajectory)):
        d = full_distance(trajectory[i - 1].pose, trajectory[i].pose)
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
    Constrói uma trajetória que minimiza a distância percorrida, conectando somente pontos
    que tenham line of sight (LOS) direto.
    """
    if not waypoints:
        return []
    
    # Seleciona o waypoint de partida (mais próximo da origem)
    start_wp = min(waypoints, key=lambda wp: wp.pose.position.x**2 +
                                       wp.pose.position.y**2 +
                                       wp.pose.position.z**2)
    path = [start_wp]
    unvisited = [wp for wp in waypoints if wp != start_wp]
    current = start_wp

    while unvisited:
        # Filtra os vizinhos com LOS direto a partir do waypoint atual
        valid_neighbors = [wp for wp in unvisited if wp.id in current.cluster_los]
        if valid_neighbors:
            next_wp = min(valid_neighbors, key=lambda wp: full_distance(current.pose, wp.pose))
            path.append(next_wp)
            unvisited.remove(next_wp)
            current = next_wp
        else:
            # Backtracking: procura um waypoint anterior que possibilite avançar
            backtracked = False
            for i in range(len(path) - 2, -1, -1):
                candidate_node = path[i]
                valid_from_candidate = [wp for wp in unvisited if wp.id in candidate_node.cluster_los]
                if valid_from_candidate:
                    backtrack_segment = list(reversed(path[i+1:]))
                    path.extend(backtrack_segment)
                    current = candidate_node
                    backtracked = True
                    break
            if not backtracked:
                break

    return path

def split_list_equally(lst, parts):
    """
    Divide a lista lst em "parts" sublistas com tamanhos aproximadamente iguais.
    """
    n = len(lst)
    if parts <= 0:
        return [lst]
    k, m = divmod(n, parts)
    segments = []
    start = 0
    for i in range(parts):
        end = start + k + (1 if i < m else 0)
        segments.append(lst[start:end])
        start = end
    return segments

def trajectory_to_path_msg(trajectory, frame_id="world"):
    """
    Converte uma lista de waypoints em uma mensagem do tipo nav_msgs/Path.
    """
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    now = Clock().now().to_msg()
    path_msg.header.stamp = now
    for wp in trajectory:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = now
        ps.pose = wp.pose
        path_msg.poses.append(ps)
    return path_msg

def process_clusters(msg, num_robots):
    """
    Processa os clusters: agrupamento, construção de trajetórias e cálculo de prioridades.
    """
    clusters = group_clusters(msg)
    clusters_data = []
    for cluster_id, wps in clusters.items():
        traj = build_trajectory(wps)
        distance_walked = compute_cumulative_distances(traj)[-1]
        total_connections = len(traj)
        clusters_data.append({
            'cluster_id': cluster_id,
            'trajectory': traj,
            'drone_capacity': num_robots - wps[0].order,
            'distance_walked': distance_walked,
            'total_connections': total_connections,
            'order': wps[0].order
        })

    clusters_sorted = sorted(clusters_data, key=lambda c: (-c['drone_capacity'], -c['total_connections']))
    total_clusters = len(clusters_sorted)
    for rank, cluster in enumerate(clusters_sorted):
        cluster['priority_rank'] = rank
        cluster['total_clusters'] = total_clusters
    return clusters_sorted

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

def translate_macro_to_drone_trajectories(macro_traj, cluster_orders, num_drones, clusters_info):
    """
    Traduz a macro trajetória para trajetórias individuais para cada drone.
    """
    drone_trajs = {i: [] for i in range(num_drones)}

    for instr in macro_traj:
        cluster_id = int(re.findall(r'\d+', instr)[0])
        chain = get_cluster_chain(cluster_id, clusters_info)
        
        for i in range(num_drones):
            if cluster_id == 0:  # Se for o cluster da base
                drone_trajs[i].append(instr)
                continue
            
            if (len(chain) == 1) and i == 0 and instr.startswith("E"):  # Correção simples para clusters
                replaced = instr.replace("E", "S")
                drone_trajs[i].append(replaced)
                continue
            
            if (len(chain) - i >= 2):  # O drone precisa permanecer como suporte
                replaced = instr.replace(str(cluster_id), str(chain[len(chain) - i - 2]))
                if instr.startswith("E"):
                    replaced = replaced.replace("E", "S")
                drone_trajs[i].append(replaced)
            else:
                drone_trajs[i].append(instr)

    return drone_trajs

def get_support_waypoint(clusters_data, cluster_id):
    """
    Retorna o id do waypoint de suporte do cluster.
    Para o cluster 0, retorna 0.
    """
    if cluster_id == 0:
        return 0
    for cluster in clusters_data:
        if cluster['cluster_id'] == cluster_id:
            for wp in cluster['trajectory']:
                if hasattr(wp, 'transition_point') and wp.transition_point:
                    return wp.id
    return None

def get_cluster(clusters_data, cluster_id):
    """Retorna os dados do cluster correspondente."""
    for cluster in clusters_data:
        if cluster['cluster_id'] == cluster_id:
            return cluster
    return None

def get_waypoint_by_id(clusters_data, wp_id):
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
    for cluster in clusters_data:
        for wp in cluster['trajectory']:
            if wp.id == wp_id:
                return wp
    return None

def decode_encoded_trajectories(encoded_trajs, clusters_data, num_robots):
    """
    Decodifica as trajetórias codificadas para gerar uma lista de instruções (waypoints)
    para cada drone.
    """
    decoded = {}
    all_id_waypoints = {}  # Dicionário para armazenar os id_waypoints de cada drone
    
    for drone_id, instructions in encoded_trajs.items():
        drone_waypoints = []
        id_waypoints = []
        for instr in instructions:
            if instr.startswith("RS"):
                cluster_id = int(instr[2:])
                support_id = get_support_waypoint(clusters_data, cluster_id)
                drone_waypoints.append(("RS", support_id))
                id_waypoints.append(support_id)
            elif instr.startswith("IS"):
                cluster_id = int(instr[2:])
                support_id = get_support_waypoint(clusters_data, cluster_id)
                drone_waypoints.append(("IS", support_id))
                id_waypoints.append(support_id)
            elif instr.startswith("S"):
                cluster_id = int(instr[1:])
                support_id = get_support_waypoint(clusters_data, cluster_id)
                drone_waypoints.append(("S", support_id))
                id_waypoints.append(support_id)
            elif instr.startswith("E"):
                cluster_id = int(instr[1:])
                cluster = get_cluster(clusters_data, cluster_id)
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
                if not segment_waypoints:
                    if id_waypoints:
                        last = id_waypoints[-1]
                        if isinstance(last, list):
                            last = last[-1] if last else None
                    else:
                        last = None
                    if last is not None:
                        segment_waypoints = [last]
                drone_waypoints.append(("E", segment_waypoints))
                id_waypoints.append(segment_waypoints)
        decoded[drone_id] = drone_waypoints
        all_id_waypoints[drone_id] = id_waypoints  # Armazena os ids deste drone
        
    return all_id_waypoints, decoded

def create_path_msgs_from_decoded(decoded, clusters_data, frame_id="world"):
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
                wp_obj = get_waypoint_by_id(clusters_data, val)
                if wp_obj is not None:
                    ps = PoseStamped()
                    ps.header.frame_id = frame_id
                    ps.header.stamp = Clock().now().to_msg()
                    ps.pose = wp_obj.pose
                    path_msg.poses.append(ps)
            elif typ == "E":
                for wp_id in val:
                    wp_obj = get_waypoint_by_id(clusters_data, wp_id)
                    if wp_obj is not None:
                        ps = PoseStamped()
                        ps.header.frame_id = frame_id
                        ps.header.stamp = Clock().now().to_msg()
                        ps.pose = wp_obj.pose
                        path_msg.poses.append(ps)
        paths[drone_id] = path_msg
    return paths
