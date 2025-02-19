#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import os
import numpy as np

from icuas25_msgs.msg import Waypoints  # Mensagem com um vetor de WaypointInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

EPSILON = 0.1

class DummyPosition:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class DummyPose:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.position = DummyPosition(x, y, z)

class DummyWaypoint:
    """
    Classe auxiliar para criar pontos interpolados com a mesma estrutura
    esperada dos waypoints (atributos: id, pose.position, cluster_id e cluster_los).
    """
    def __init__(self, id, x, y, z, cluster_id=None, cluster_los=None):
        self.id = id
        self.pose = DummyPose(x, y, z)
        self.cluster_id = cluster_id
        self.cluster_los = cluster_los if cluster_los is not None else []

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

def get_cluster_color(cluster_id):
    """Retorna uma cor exclusiva (ColorRGBA) para o cluster."""
    colors = [
        (1.0, 0.0, 0.0, 1.0),   # vermelho
        (0.0, 1.0, 0.0, 1.0),   # verde
        (0.0, 0.0, 1.0, 1.0),   # azul
        (1.0, 1.0, 0.0, 1.0),   # amarelo
        (1.0, 0.0, 1.0, 1.0),   # magenta
        (0.0, 1.0, 1.0, 1.0),   # ciano
        (0.5, 0.5, 0.5, 1.0),   # cinza
    ]
    index = hash(cluster_id) % len(colors)
    r, g, b, a = colors[index]
    return ColorRGBA(r=r, g=g, b=b, a=a)

def get_priority_color(rank, total):
    """
    Retorna uma cor que varia de verde (maior prioridade, rank=0) a vermelho (menor prioridade, rank=total-1).
    Se houver apenas um cluster, retorna verde.
    """
    if total <= 1:
        return ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    factor = rank / (total - 1)  # factor varia de 0 a 1
    # Verde para factor=0: (0,1,0), Vermelho para factor=1: (1,0,0)
    r = factor
    g = 1.0 - factor
    b = 0.0
    return ColorRGBA(r=r, g=g, b=b, a=1.0)

def interpolate_waypoint(wp1, wp2, fraction, new_id):
    """
    Cria um novo waypoint interpolado entre wp1 e wp2.
    fraction: valor entre 0 e 1 indicando a fração da distância.
    new_id: ID atribuído ao waypoint interpolado.
    """
    x = wp1.pose.position.x + fraction * (wp2.pose.position.x - wp1.pose.position.x)
    y = wp1.pose.position.y + fraction * (wp2.pose.position.y - wp1.pose.position.y)
    z = wp1.pose.position.z + fraction * (wp2.pose.position.z - wp1.pose.position.z)
    return DummyWaypoint(new_id, x, y, z, cluster_id=wp1.cluster_id, cluster_los=[])

def divide_trajectory_equal(trajectory, divisions):
    """
    Divide a trajetória (lista de waypoints) em 'divisions' segmentos de igual distância.
    Retorna uma lista de segmentos, onde cada segmento é definido por dois pontos:
    o ponto inicial e o ponto final do segmento.
    """
    if divisions <= 1 or len(trajectory) < 2:
        return [trajectory]
    
    cum = compute_cumulative_distances(trajectory)
    total_distance = cum[-1]
    segment_length = total_distance / divisions

    boundaries = [trajectory[0]]
    interp_id_counter = 0
    for i in range(1, divisions):
        target = i * segment_length
        for j in range(len(cum) - 1):
            if cum[j] <= target <= cum[j+1]:
                fraction = (target - cum[j]) / (cum[j+1] - cum[j]) if (cum[j+1] - cum[j]) != 0 else 0
                new_wp = interpolate_waypoint(trajectory[j], trajectory[j+1], fraction, f"interp_{interp_id_counter}")
                interp_id_counter += 1
                boundaries.append(new_wp)
                break
    boundaries.append(trajectory[-1])
    
    segments = []
    for i in range(len(boundaries) - 1):
        segments.append([boundaries[i], boundaries[i+1]])
    return segments

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

    # Função custo entre dois waypoints: retorna custo se houver LOS, caso contrário, infinito.
    def cost(wpA, wpB):
        if wpB.id not in wpA.cluster_los:
            return float('inf')
        horz = horizontal_distance(wpA.pose, wpB.pose)
        vert = abs(wpA.pose.position.z - wpB.pose.position.z)
        return horz + vertical_weight * vert

    # Determina os pontos de transição:
    transition_waypoints = [wp for wp in waypoints if hasattr(wp, 'is_transition') and wp.is_transition]
    if len(transition_waypoints) >= 2:
        start_wp = min(transition_waypoints, key=lambda wp: distance_from_origin(wp.pose))
        end_wp = max(transition_waypoints, key=lambda wp: distance_from_origin(wp.pose))
    else:
        start_wp = min(waypoints, key=lambda wp: distance_from_origin(wp.pose))
        end_wp = max(waypoints, key=lambda wp: distance_from_origin(wp.pose))
    
    # Cria a lista inicial com os pontos de transição; os nós restantes são os intermediários.
    remaining = [wp for wp in waypoints if wp not in (start_wp, end_wp)]
    path = [start_wp, end_wp]

    # Enquanto houver nós a inserir, tente inseri-los na posição que cause o menor aumento de custo.
    while remaining:
        best_insertion = None
        best_candidate = None
        best_extra_cost = float('inf')
        for candidate in remaining:
            for i in range(len(path)-1):
                A = path[i]
                B = path[i+1]
                # Verifica se é possível inserir o candidato entre A e B: deve existir LOS de A para candidate e de candidate para B.
                if candidate.id in A.cluster_los and B.id in candidate.cluster_los:
                    extra = cost(A, candidate) + cost(candidate, B) - cost(A, B)
                    if extra < best_extra_cost:
                        best_extra_cost = extra
                        best_candidate = candidate
                        best_insertion = i + 1
        if best_candidate is None:
            # Se nenhum nó puder ser inserido em nenhuma posição com LOS, encerra o loop.
            break
        # Insere o candidato na posição escolhida.
        path.insert(best_insertion, best_candidate)
        remaining.remove(best_candidate)
    
    if remaining:
        for candidate in remaining:
            if candidate.id in path[-1].cluster_los:
                path.append(candidate)
            else:
                path.append(candidate)
        # Por fim, garanta que end_wp esteja no final
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

    #median_pose_distance = distance_walked/total_connections

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

def publish_clusters_markers(node, clusters_data):
    """
    Publica os marcadores dos clusters:
      - Trajetória completa de cada cluster (LINE_STRIP)
      - Texto com o ID do cluster no ponto de transição (início)
    """
    markers = []
    marker_id = 0
    for data in clusters_data:
        cluster_id = data['cluster_id']
        trajectory = data['trajectory']
        # Usa a cor de prioridade baseada no rank
        color = get_priority_color(data['priority_rank'], data['total_clusters'])

        # Marcador da trajetória completa
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = f"cluster_{cluster_id}_full"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.points = []
        marker.colors = []
        for wp in trajectory:
            pt = Point(x=wp.pose.position.x, y=wp.pose.position.y, z=wp.pose.position.z)
            marker.points.append(pt)
            marker.colors.append(color)
        markers.append(marker)

        # Marcador de texto com o ID do cluster no ponto de transição (início)
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = node.get_clock().now().to_msg()
        text_marker.ns = f"cluster_{cluster_id}_id"
        text_marker.id = marker_id
        marker_id += 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = trajectory[0].pose.position.x
        text_marker.pose.position.y = trajectory[0].pose.position.y
        text_marker.pose.position.z = trajectory[0].pose.position.z + 0.3
        text_marker.scale.z = 0.5
        text_marker.color = color
        text_marker.text = f"Cluster {cluster_id}"
        markers.append(text_marker)
    return markers

def publish_transition_markers(node, clusters_data):
    """
    Publica os pontos de transição de cada cluster:
      - Esfera e texto "Start" no ponto inicial (transição de origem)
      - Esfera e texto "Return" no ponto final (ponto de retorno)
    """
    markers = []
    marker_id = 1000  # Offset para IDs
    for data in clusters_data:
        cluster_id = data['cluster_id']
        trajectory = data['trajectory']
        color = get_priority_color(data['priority_rank'], data['total_clusters'])

        # Marcador para o ponto inicial (transição de origem)
        start_marker = Marker()
        start_marker.header.frame_id = "world"
        start_marker.header.stamp = node.get_clock().now().to_msg()
        start_marker.ns = f"cluster_{cluster_id}_transition_start"
        start_marker.id = marker_id
        marker_id += 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.pose.position.x = trajectory[0].pose.position.x
        start_marker.pose.position.y = trajectory[0].pose.position.y
        start_marker.pose.position.z = trajectory[0].pose.position.z
        start_marker.color = color
        markers.append(start_marker)

        # Marcador para o ponto final (ponto de retorno)
        end_marker = Marker()
        end_marker.header.frame_id = "world"
        end_marker.header.stamp = node.get_clock().now().to_msg()
        end_marker.ns = f"cluster_{cluster_id}_transition_end"
        end_marker.id = marker_id
        marker_id += 1
        end_marker.type = Marker.SPHERE
        end_marker.action = Marker.ADD
        end_marker.scale.x = 0.2
        end_marker.scale.y = 0.2
        end_marker.scale.z = 0.2
        end_marker.pose.position.x = trajectory[-1].pose.position.x
        end_marker.pose.position.y = trajectory[-1].pose.position.y
        end_marker.pose.position.z = trajectory[-1].pose.position.z
        end_marker.color = color
        markers.append(end_marker)

        # Texto "Return" para o ponto final
        end_text = Marker()
        end_text.header.frame_id = "world"
        end_text.header.stamp = node.get_clock().now().to_msg()
        end_text.ns = f"cluster_{cluster_id}_transition_end_text"
        end_text.id = marker_id
        marker_id += 1
        end_text.type = Marker.TEXT_VIEW_FACING
        end_text.action = Marker.ADD
        end_text.pose.position.x = trajectory[-1].pose.position.x
        end_text.pose.position.y = trajectory[-1].pose.position.y
        end_text.pose.position.z = trajectory[-1].pose.position.z + 0.3
        end_text.scale.z = 0.5
        end_text.color = color
        end_text.text = "Return"
        markers.append(end_text)
    return markers

def publish_optimal_trajectory_markers(node, clusters_data):
    """
    Publica as trajetórias ótimas de cada cluster conectando
    o ponto de transição (início) e o ponto de retorno (final) da trajetória.
    """
    markers = []
    marker_id = 2000  # Offset para IDs
    for data in clusters_data:
        cluster_id = data['cluster_id']
        trajectory = data['trajectory']
        color = get_priority_color(data['priority_rank'], data['total_clusters'])

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = f"cluster_{cluster_id}_optimal"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        start_pt = Point(x=trajectory[0].pose.position.x,
                         y=trajectory[0].pose.position.y,
                         z=trajectory[0].pose.position.z)
        end_pt = Point(x=trajectory[-1].pose.position.x,
                       y=trajectory[-1].pose.position.y,
                       z=trajectory[-1].pose.position.z)
        marker.points = [start_pt, end_pt]
        marker.colors = [color, color]
        markers.append(marker)
    return markers

def publish_return_path_markers(node, clusters_data):
    """
    Publica os marcadores da rota de retorno de cada cluster:
    uma linha que conecta o ponto de retorno (final) ao ponto de transição (início)
    indicando que esta é a rota de retorno.
    """
    markers = []
    marker_id = 3000  # Offset para IDs
    for data in clusters_data:
        cluster_id = data['cluster_id']
        trajectory = data['trajectory']
        # Define cor para a rota de retorno (ex.: roxo)
        return_color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0)
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = f"cluster_{cluster_id}_return_path"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        start_pt = Point(x=trajectory[-1].pose.position.x,
                         y=trajectory[-1].pose.position.y,
                         z=trajectory[-1].pose.position.z)
        end_pt = Point(x=trajectory[0].pose.position.x,
                       y=trajectory[0].pose.position.y,
                       z=trajectory[0].pose.position.z)
        marker.points = [start_pt, end_pt]
        marker.color = return_color
        markers.append(marker)
    return markers

def publish_pose_id_markers(node, clusters_data):
    """
    Publica marcadores de texto para cada pose na trajetória de cada cluster,
    exibindo o id da pose.
    """
    markers = []
    marker_id = 4000  # Offset para IDs de poses
    for data in clusters_data:
        cluster_id = data['cluster_id']
        trajectory = data['trajectory']
        color = get_priority_color(data['priority_rank'], data['total_clusters'])
        for wp in trajectory:
            pose_marker = Marker()
            pose_marker.header.frame_id = "world"
            pose_marker.header.stamp = node.get_clock().now().to_msg()
            pose_marker.ns = f"cluster_{cluster_id}_pose_ids"
            pose_marker.id = marker_id
            marker_id += 1
            pose_marker.type = Marker.TEXT_VIEW_FACING
            pose_marker.action = Marker.ADD
            pose_marker.pose.position.x = wp.pose.position.x
            pose_marker.pose.position.y = wp.pose.position.y
            pose_marker.pose.position.z = wp.pose.position.z + 0.2  # leve deslocamento vertical
            pose_marker.scale.z = 0.3
            pose_marker.color = color
            pose_marker.text = str(wp.id)
            markers.append(pose_marker)
    return markers

def aggregate_markers(node, clusters_data):
    """Agrupa todos os marcadores para publicação no RViz."""
    markers = []
    markers.extend(publish_clusters_markers(node, clusters_data))
    markers.extend(publish_transition_markers(node, clusters_data))
    markers.extend(publish_optimal_trajectory_markers(node, clusters_data))
    markers.extend(publish_return_path_markers(node, clusters_data))
    markers.extend(publish_pose_id_markers(node, clusters_data))
    return markers

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
