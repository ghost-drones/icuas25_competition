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

from crazyflie_interfaces.srv import GoTo, Land, Takeoff
from tf_transformations import euler_from_quaternion
from icuas25_msgs.srv import PathService
from trajectory_utils import process_clusters, get_waypoint_by_id
from icuas25_msgs.msg import Waypoints

class ManageSwarmNode(Node):
    def __init__(self, drone_ids):
        super().__init__('manage_swarm_node')
        self.drone_ids = drone_ids

        self.poses = {}
        self.batteries = {}
        self.trajectories = {}
        self.trajectories_id = {}
        self.trajectories_path = {}
        self.clusters_data = []  # Dados processados dos clusters
        self.base_position = np.array([0.0, 0.0, 0.0])
        self.battery_threshold = 20.0  # Battery threshold in percentage

        self.charging_time_per_percentage = 10.0  # Time (in seconds) to charge 1% of the battery
        self.charging_timers = {drone_id: None for drone_id in self.drone_ids}
        self.charging_target = 100.0  # Target battery percentage for full charge
        
        self.pending_trajectories = {drone_id: None for drone_id in self.drone_ids}

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.go_to_clients = {}
        self.land_clients = {}
        self.takeoff_clients = {}  # Cliente para o serviço takeoff

        self.client_path_planner = self.create_client(PathService, '/ghost/path_planner')
        while not self.client_path_planner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ghost/path_planner not available, waiting again...')

        self.subscription = self.create_subscription(
            Waypoints,
            '/ghost/waypoints',
            self.waypoints_callback,
            10
        )

        self.save_first_value = {}
        self.save_target_value = {}

        for drone_id in self.drone_ids:
            self.save_first_value[drone_id] = None
            self.save_target_value[drone_id] = None

            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'
            go_to_service = f'/cf_{drone_id}/go_to'
            land_service = f'/cf_{drone_id}/land'
            takeoff_service = f'/cf_{drone_id}/takeoff'

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
            self.takeoff_clients[drone_id] = self.create_client(Takeoff, takeoff_service)

            self.get_logger().info(f'Subscribed to topics and services for drone {drone_id}')

        self.current_step = 0
        self.current_substep = {drone_id: 0 for drone_id in self.drone_ids}
        self.command_sent = {drone_id: False for drone_id in self.drone_ids}
        self.avr_vel = 1.1

        self.initial_takeoff = True
        self.layer_gap = 0.8  # Agora este valor é usado para definir o offset fixo em z
        self.takeoff_targets = {}

        # Dicionário para armazenar offsets fixos para cada drone
        self.fixed_offsets = {}

        # Atributo para gerenciar o delay pós-takeoff
        self.takeoff_done_time = None

        origin = Point(x=-1.0, y=-1.0, z=-1.0)

        # Variável para armazenar a resposta do ghost/path_planner
        self.path_response = None
        self.request_traverse_origin(origin, origin, origin)

        hz = 10
        T = 1 / hz
        self.timer = self.create_timer(T, self.timer_callback)

    def waypoints_callback(self, msg):
        num_robots = len(self.drone_ids)
        self.clusters_data = process_clusters(msg, num_robots)

    def send_takeoff(self, drone_id, height, duration_sec, group_mask=0):
        if drone_id not in self.takeoff_clients:
            self.get_logger().error(f'Takeoff client not found for drone {drone_id}')
            return

        client = self.takeoff_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service /cf_{drone_id}/takeoff unavailable')
            return

        request = Takeoff.Request()
        request.group_mask = group_mask
        request.height = height
        request.duration.sec = int(duration_sec)
        request.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        client.call_async(request)

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg

    def trajectory_callback(self, msg, drone_id):
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
        _, _, yaw = euler_from_quaternion(quat)
        return yaw

    def calc_duration(self, pos_1, pos_2):
        pos_1 = np.array([pos_1.pose.position.x, pos_1.pose.position.y, pos_1.pose.position.z], dtype='float32')
        pos_2 = np.array([pos_2.pose.position.x, pos_2.pose.position.y, pos_2.pose.position.z], dtype='float32')
        duration = np.linalg.norm(pos_2 - pos_1) / self.avr_vel
        return duration

    def send_go_to(self, drone_id, pose: PoseStamped, duration_sec, group_mask=0, relative=False):
        if drone_id not in self.go_to_clients:
            self.get_logger().error(f'GoTo client not found for drone {drone_id}')
            return

        client = self.go_to_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service /cf_{drone_id}/go_to unavailable')
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
            self.get_logger().error(f'Land client not found for drone {drone_id}')
            return

        client = self.land_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service /cf_{drone_id}/land unavailable')
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

    def extract_unique_ids(self, data):
        unique_ids = []
        for values in data.values():
            for item in values:
                if not isinstance(item, list):
                    if item not in unique_ids:
                        unique_ids.append(item)
        return unique_ids

    def add_offset_2_pose(self, drone_id, target_pose):
        # Calcula e armazena offsets fixos para cada drone (se ainda não calculados)
        if drone_id not in self.fixed_offsets:
            num_drones = len(self.drone_ids)
            angle = 2 * math.pi * (drone_id - 1) / (num_drones - 1) if num_drones > 1 else 0.0
            x_offset = 1.0 * math.cos(angle)
            y_offset = 1.0 * math.sin(angle)
            z_offset = drone_id * self.layer_gap

            if drone_id == 1:
                self.fixed_offsets[drone_id] = (0, 0, 0)
            else:
                self.fixed_offsets[drone_id] = (x_offset, y_offset, z_offset)
        else:
            x_offset, y_offset, z_offset = self.fixed_offsets[drone_id]
        # Aplica os offsets fixos
        target_pose.pose.position.x += x_offset
        target_pose.pose.position.y += y_offset
        target_pose.pose.position.z += z_offset
        return target_pose
    
    def request_traverse_origin(self, origin: Point, destination: Point, support: Point):
        request = PathService.Request()
        request.origin = Point(x=origin.x, y=origin.y, z=origin.z)
        request.destination = Point(x=destination.x, y=destination.y, z=destination.z)
        request.support = Point(x=support.x, y=support.y, z=support.z)

        future = self.client_path_planner.call_async(request)
        future.add_done_callback(self.handle_path_response)
    
    def handle_path_response(self, future):
        try:
            self.path_response = future.result()
        except Exception as e:
            self.get_logger().error(f"Ghost/path_planner service call failed: {e}")

    def timer_callback(self):
        # Verifica se os dados necessários já foram recebidos para TODOS os drones.
        for drone_id in self.drone_ids:
            if (drone_id not in self.poses or 
                drone_id not in self.batteries or 
                drone_id not in self.trajectories or
                drone_id not in self.trajectories_id or 
                drone_id not in self.trajectories_path):
                return

        # Se o takeoff já foi concluído, aguarda 3 segundos antes de processar o restante.
        if self.takeoff_done_time is not None:
            now = self.get_clock().now()
            if (now - self.takeoff_done_time).nanoseconds / 1e9 < 3.0:
                return  # Pula esta iteração até os 3 segundos passarem
            else:
                # Após 3 segundos, reseta o delay para continuar o processamento normalmente.
                self.takeoff_done_time = None

        # FASE 1: Decolagem inicial para altitude de takeoff específica.
        if self.initial_takeoff:
            all_reached = True
            for drone_id in self.drone_ids:
                current_pose = self.poses[drone_id]
                if drone_id not in self.takeoff_targets:
                    target_pose = deepcopy(current_pose)
                    target_pose.pose.position.z = self.layer_gap * drone_id
                    self.takeoff_targets[drone_id] = target_pose
                    duration = self.calc_duration(current_pose, target_pose)
                    # Utiliza o serviço de takeoff em vez de go_to
                    self.send_takeoff(drone_id, target_pose.pose.position.z, duration_sec=duration)
                else:
                    target_pose = self.takeoff_targets[drone_id]
                if not self.is_pose_reached(current_pose, target_pose):
                    all_reached = False
            if all_reached:
                self.get_logger().info("Fase de decolagem concluída para todos os drones. Aguardando 3 segundos...")
                self.initial_takeoff = False
                self.takeoff_done_time = self.get_clock().now()
                for drone_id in self.drone_ids:
                    self.command_sent[drone_id] = False
                return  # Pula processamento adicional nesta iteração
          
        # FASE 2: Processamento normal da trajetória
        unique_ids = self.extract_unique_ids(self.trajectories_id)
        total_steps = len(self.trajectories_id[self.drone_ids[0]])
        if self.current_step >= total_steps:
            self.get_logger().info("Todas as trajetórias foram completadas.")
            return

        finished_current_step = {}

        for drone_id in self.drone_ids:
            steps = self.trajectories_id[drone_id]
            current_command = steps[self.current_step]
            future_command = steps[self.current_step+1]
            current_pose = self.poses[drone_id]

            # Se for ação secundária, verifica se o waypoint deve ser ignorado
            if isinstance(current_command, list):
                substep = self.current_substep[drone_id]
                flat_index = self.get_flat_index(drone_id, self.current_step, substep)
                if current_command[substep] in unique_ids:
                    self.get_logger().info(f"Skipping waypoint {current_command[substep]} for drone {drone_id}")
                    # Marca esse waypoint como concluído sem enviá-lo
                    finished_current_step[drone_id] = True
                    self.command_sent[drone_id] = True
                    if self.current_substep[drone_id] < len(current_command) - 1:
                        self.current_substep[drone_id] += 1
                    target_pose = deepcopy(current_pose)
                else:
                    target_pose = self.trajectories_path[drone_id][flat_index]
            else:  # Comando primário (suporte)
                flat_index = self.get_flat_index(drone_id, self.current_step, 0)
                if current_command == 0:  # Se for retorno à base (traverse_origin)
                    if self.save_first_value[drone_id] is None:
                        self.save_first_value[drone_id] = self.poses[drone_id].pose.position
                    
                    self.request_traverse_origin(
                        origin=self.save_first_value[drone_id], 
                        destination=get_waypoint_by_id(self.clusters_data, future_command).pose.position, 
                        support=get_waypoint_by_id(self.clusters_data, current_command).pose.position
                    )

                    if self.path_response.path == Point(x=-1.0, y=-1.0, z=-1.0):
                        return

                    if not self.command_sent[drone_id]:
                        target_pose = PoseStamped()
                        target_pose.pose.position = deepcopy(self.path_response.path)
                        target_pose = self.add_offset_2_pose(drone_id, target_pose)
                else:  # Se não for retorno à base, continua normalmente e adiciona offset
                    self.save_first_value[drone_id] = None
                    target_pose = deepcopy(self.trajectories_path[drone_id][flat_index])
                    target_pose = self.add_offset_2_pose(drone_id, target_pose)

            if not self.command_sent[drone_id]:
                duration = self.calc_duration(self.poses[drone_id], target_pose)
                self.send_go_to(drone_id, target_pose, duration_sec=duration)
                self.command_sent[drone_id] = True

                if current_command == 0:
                    self.save_target_value[drone_id] = deepcopy(target_pose)
                else:
                    self.save_target_value[drone_id] = Point(x=0.0, y=0.0, z=0.0)
            
            if current_command == 0:
                target_pose = deepcopy(self.save_target_value[drone_id])

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
            self.get_logger().info(f"Step {self.current_step} completed for all drones.")
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
