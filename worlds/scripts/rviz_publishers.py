#!/usr/bin/env python3
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def get_priority_color(rank, total):
    """
    Retorna uma cor que varia de verde (maior prioridade, rank=0) a vermelho (menor prioridade, rank=total-1).
    Se houver apenas um cluster, retorna verde.
    """
    if total <= 1:
        return ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    factor = rank / (total - 1)  # factor varia de 0 a 1
    r = factor
    g = 1.0 - factor
    b = 0.0
    return ColorRGBA(r=r, g=g, b=b, a=1.0)

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
