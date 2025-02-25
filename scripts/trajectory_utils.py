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

        if instr.startswith("E"):
            if cluster_id is not None and cluster_id in cluster_orders and cluster_orders[cluster_id] > 0:
                order = cluster_orders[cluster_id]
                for i in range(num_drones):
                    if i < order:
                        if (cluster_id != 0) and len(chain) > i+2:
                            replaced = instr.replace(str(cluster_id), str(chain[i+1]))
                            drone_trajs[i].append(replaced)
                        else:
                            drone_trajs[i].append("S" + str(cluster_id))
                    else:
                        drone_trajs[i].append(instr)
            else:
                for i in range(num_drones):
                    drone_trajs[i].append(instr)
        else:
            for i in range(num_drones):
                if (cluster_id != 0) and len(chain) > i+2:
                    replaced = instr.replace(str(cluster_id), str(chain[i+1]))
                    drone_trajs[i].append(replaced)
                else:
                    drone_trajs[i].append(instr)
    return drone_trajs
