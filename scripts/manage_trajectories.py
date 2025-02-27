#!/usr/bin/env python3
import os
import math
import rclpy
import ast
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path

from crazyflie_interfaces.srv import GoTo, Land
from tf_transformations import euler_from_quaternion

class ManageSwarmNode(Node):
    def __init__(self, drone_ids):
        super().__init__('manage_swarm_node')
        self.drone_ids = drone_ids

        self.poses = {}
        self.batteries = {}
        self.trajectories = {}       # Ação primária (string) de cada drone
        self.trajectories_id = {}    # Elementos que determinam os waypoints
        self.trajectories_path = {}  # Lista de PoseStamped (waypoints)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Clientes dos serviços go_to e land
        self.go_to_clients = {}
        self.land_clients = {}

        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'
            go_to_service = f'/cf_{drone_id}/go_to'
            land_service = f'/cf_{drone_id}/land'

            self.create_subscription(
                PoseStamped,
                pose_topic,
                lambda msg, id=drone_id: self.pose_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                BatteryState,
                battery_topic,
                lambda msg, id=drone_id: self.battery_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                String,
                trajectory_topic,
                lambda msg, id=drone_id: self.trajectory_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                String,
                trajectory_ids_topic,
                lambda msg, id=drone_id: self.trajectory_ids_callback(msg, id),
                qos_profile
            )
            self.create_subscription(
                Path,
                trajectory_path_topic,
                lambda msg, id=drone_id: self.trajectory_path_callback(msg, id),
                qos_profile
            )

            self.go_to_clients[drone_id] = self.create_client(GoTo, go_to_service)
            self.land_clients[drone_id] = self.create_client(Land, land_service)

            self.get_logger().info(f'Inscrito nos tópicos e serviços do drone {drone_id}')

        # Estado de gerenciamento de trajetória (global para todos os drones)
        self.current_step = 0  # índice do passo primário (elemento de trajectories_id)
        # Se o passo for uma lista, indica o índice atual na sublista para cada drone.
        self.current_substep = {drone_id: 0 for drone_id in self.drone_ids}
        # Indica se o comando para o passo/subpasso atual já foi enviado.
        self.command_sent = {drone_id: False for drone_id in self.drone_ids}

        hz = 10
        T = 1 / hz
        self.timer = self.create_timer(T, self.timer_callback)

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg

    def trajectory_callback(self, msg, drone_id):
        # Armazena a ação primária como uma string
        self.trajectories[drone_id] = msg.data

    def trajectory_ids_callback(self, msg, drone_id):
        self.trajectories_id[drone_id] = ast.literal_eval(msg.data)

    def trajectory_path_callback(self, msg, drone_id):
        self.trajectories_path[drone_id] = msg.poses

    def get_yaw_from_pose(self, pose: PoseStamped) -> float:
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        # Converte quaternion para Euler (roll, pitch, yaw) em radianos
        _, _, yaw = euler_from_quaternion(quat)
        return yaw

    def send_go_to(self, drone_id, pose: PoseStamped, duration_sec, group_mask=0, relative=False):
        if drone_id not in self.go_to_clients:
            self.get_logger().error(f'Cliente go_to não encontrado para o drone {drone_id}')
            return

        client = self.go_to_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Serviço /cf_{drone_id}/go_to indisponível')
            return

        goal = pose.pose.position
        yaw = self.get_yaw_from_pose(pose)

        request = GoTo.Request()
        request.group_mask = group_mask
        request.relative = relative
        request.goal = goal
        request.yaw = yaw
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        client.call_async(request)

    def send_land(self, drone_id, height, duration_sec, group_mask=0):
        if drone_id not in self.land_clients:
            self.get_logger().error(f'Cliente Land não encontrado para o drone {drone_id}')
            return

        client = self.land_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Serviço /cf_{drone_id}/land indisponível')
            return

        request = Land.Request()
        request.group_mask = group_mask
        request.height = height
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        client.call_async(request)

    def is_pose_reached(self, current_pose: PoseStamped, target_pose: PoseStamped, tolerance=0.2) -> bool:
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        dz = current_pose.pose.position.z - target_pose.pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < tolerance

    def get_flat_index(self, drone_id, step, substep):
        """
        Calcula o índice flattened na lista trajectories_path com base no step (elemento de trajectories_id)
        e no substep. Para cada elemento anterior, soma 1 se for um int ou o tamanho da lista se for uma lista.
        No elemento atual, adiciona o valor de substep (ou 0 se for int).
        """
        steps = self.trajectories_id[drone_id]
        flat_index = 0
        for i in range(step):
            elem = steps[i]
            if isinstance(elem, list):
                flat_index += len(elem)
            else:
                flat_index += 1
        current_elem = steps[step]
        if isinstance(current_elem, list):
            flat_index += substep
        else:
            flat_index += 0
        return flat_index

    def timer_callback(self):
        # Verifica se os dados necessários já foram recebidos para TODOS os drones.
        for drone_id in self.drone_ids:
            if (drone_id not in self.poses or 
                drone_id not in self.batteries or 
                drone_id not in self.trajectories or
                drone_id not in self.trajectories_id or 
                drone_id not in self.trajectories_path):
                return

        total_steps = len(self.trajectories_id[self.drone_ids[0]])
        if self.current_step >= total_steps:
            self.get_logger().info("Todas as trajetórias foram concluídas.")
            return

        finished_current_step = {}

        for drone_id in self.drone_ids:
            steps = self.trajectories_id[drone_id]
            current_command = steps[self.current_step]
            current_pose = self.poses[drone_id]

            if isinstance(current_command, list):
                substep = self.current_substep[drone_id]
                flat_index = self.get_flat_index(drone_id, self.current_step, substep)
            else:
                flat_index = self.get_flat_index(drone_id, self.current_step, 0)

            target_pose = self.trajectories_path[drone_id][flat_index]

            # Envia o comando apenas se ainda não foi enviado
            if not self.command_sent[drone_id]:
                self.send_go_to(drone_id, target_pose, duration_sec=20.0)
                # Imprime a ação primária utilizando self.trajectories (string)
                self.command_sent[drone_id] = True

            # Verifica se o drone alcançou o waypoint alvo
            if self.is_pose_reached(current_pose, target_pose):
                if isinstance(current_command, list):
                    # Para ações secundárias, cada drone avança para o próximo waypoint imediatamente.
                    if self.current_substep[drone_id] < len(current_command) - 1:
                        self.current_substep[drone_id] += 1
                        self.command_sent[drone_id] = False
                        finished_current_step[drone_id] = False
                    else:
                        finished_current_step[drone_id] = True
                else:
                    finished_current_step[drone_id] = True
            else:
                finished_current_step[drone_id] = False

        # Para as ações primárias, aguarda que TODOS os drones tenham finalizado o passo
        if all(finished_current_step.get(d, False) for d in self.drone_ids):
            self.get_logger().info(f"Passo {self.current_step} concluído para todos os drones.")
            self.current_step += 1
            for drone_id in self.drone_ids:
                self.command_sent[drone_id] = False
                self.current_substep[drone_id] = 0

def main(args=None):
    rclpy.init(args=args)
    
    num_robots = int(os.getenv("NUM_ROBOTS", "5"))
    drone_ids = list(range(1, num_robots + 1))
    
    node = ManageSwarmNode(drone_ids)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
