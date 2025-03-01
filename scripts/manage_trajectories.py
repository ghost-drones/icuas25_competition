#!/usr/bin/env python3
import os
import math
import rclpy
import ast
import numpy as np
from copy import deepcopy
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

        self.current_step = 0
        self.current_substep = {drone_id: 0 for drone_id in self.drone_ids}
        self.command_sent = {drone_id: False for drone_id in self.drone_ids}
        self.avr_vel = 0.7

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

    def calc_duration(self, pos_1, pos_2):

        pos_1 = np.array([pos_1.pose.position.x, pos_1.pose.position.y, pos_1.pose.position.z], dtype='float32')
        pos_2 = np.array([pos_2.pose.position.x, pos_2.pose.position.y, pos_2.pose.position.z], dtype='float32')

        duration = np.linalg.norm(pos_2 - pos_1) / self.avr_vel

        return duration

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

    def is_pose_reached(self, current_pose: PoseStamped, target_pose: PoseStamped, tolerance=1.0) -> bool:
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

    def get_offset_for_drone(self, drone_id, sorted_primary, layer_gap=0.8):
        """
        Para drones que compartilham o mesmo waypoint primário simples,
        distribui os destinos em camadas verticais. O drone com menor id (em sorted_primary)
        não recebe offset; os demais terão seu destino elevado em z de forma incremental.
        """
        idx = sorted_primary.index(drone_id)
        if idx == 0:
            return (0.0, 0.0, 0.3)
        else:
            return (0.0, 0.0, 0.3 + idx * layer_gap)

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

        # Para comandos primários simples, determina quais drones compartilham o mesmo waypoint.
        primary_drone_ids = []
        for drone_id in self.drone_ids:
            current_command = self.trajectories_id[drone_id][self.current_step]
            if not isinstance(current_command, list):
                primary_drone_ids.append(drone_id)
        sorted_primary = sorted(primary_drone_ids) if primary_drone_ids else []

        finished_current_step = {}

        for drone_id in self.drone_ids:
            steps = self.trajectories_id[drone_id]
            current_command = steps[self.current_step]
            current_pose = self.poses[drone_id]

            if isinstance(current_command, list):
                substep = self.current_substep[drone_id]
                flat_index = self.get_flat_index(drone_id, self.current_step, substep)
                # Para substeps, nenhum offset é aplicado.
                target_pose = self.trajectories_path[drone_id][flat_index]
            else:
                flat_index = self.get_flat_index(drone_id, self.current_step, 0)
                target_pose = deepcopy(self.trajectories_path[drone_id][flat_index])
                # Se o comando for primário e compartilhado, aplica offset vertical em camadas.
                if drone_id in sorted_primary:
                    offset = self.get_offset_for_drone(drone_id, sorted_primary)
                    target_pose.pose.position.x += offset[0]
                    target_pose.pose.position.y += offset[1]
                    target_pose.pose.position.z += offset[2]

            if not self.command_sent[drone_id]:
                duration = self.calc_duration(self.poses[drone_id], target_pose)
                self.send_go_to(drone_id, target_pose, duration_sec=duration)
                self.command_sent[drone_id] = True

            if self.is_pose_reached(current_pose, target_pose):
                if isinstance(current_command, list):
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

            print("-------")
            print("Drone Id: ", drone_id)
            print("Current: ", current_command)
            print("Reached: ", finished_current_step[drone_id])

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