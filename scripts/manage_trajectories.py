#!/usr/bin/env python3
import os
import ast
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from nav_msgs.msg import Path

# Biblioteca para comunicação com o swarm
from crazyflie_py import Crazyswarm

class ManageTrajectoriesNode(Node):
    def __init__(self, drone_ids):
        #super().__init__('manage_trajectories_node')
        self.swarm = Crazyswarm()
        self.drone_ids = drone_ids

        # Dicionários para armazenar as informações recebidas
        self.poses = {}               # Última PoseStamped de cada drone
        self.batteries = {}           # Último status de bateria
        self.trajectories = {}        # Dados “crus” do tópico trajectory_encoded
        self.actions = {}             # Lista de macro ações (ex: ['E0','RS0',...])
        self.trajectory_ids_data = {} # Lista (possivelmente aninhada) de ints para cada ação
        self.trajectory_paths_data = {}  # Lista “flattened” de PoseStamped (Path)
        self.action_waypoints = {}    # Computado: para cada drone, lista de listas de waypoints por macro ação

        # Variáveis para controle de execução
        self.global_action_index = 0          # Macro ação atual (comum a todos os drones)
        self.sub_indices = {d: 0 for d in self.drone_ids}  # Para cada drone, qual waypoint (dentro da macro ação) está em execução
        self.last_commanded = {}              # Para evitar reenvio do mesmo comando
        self.position_threshold = 0.3         # Tolerância para considerar que o drone alcançou o waypoint

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Inscrições para cada drone
        for drone_id in self.drone_ids:
            pose_topic = f'/cf_{drone_id}/pose'
            battery_topic = f'/cf_{drone_id}/battery_status'
            trajectory_topic = f'/ghost/cf_{drone_id}/trajectory_encoded'
            trajectory_ids_topic = f'/ghost/cf_{drone_id}/trajectory_ids'
            trajectory_path_topic = f'/ghost/cf_{drone_id}/trajectory_path'

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
            self.get_logger().info(f'Inscrito nos tópicos do drone {drone_id}')

        # Inicializa comunicação com o swarm via crazyflie-lib-python

        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.cfs = {}
        # Mapeia cada drone à sua URI (note que os drones são numerados de 1 a N)
        uri_mapping = {
            1: 'udp://0.0.0.0:19850',
            2: 'udp://0.0.0.0:19851',
            3: 'udp://0.0.0.0:19852',
            4: 'udp://0.0.0.0:19853',
            5: 'udp://0.0.0.0:19854',
            6: 'udp://0.0.0.0:19855'
        }
        for drone_id in self.drone_ids:
            uri = uri_mapping.get(drone_id)
            found = False
            for cf in self.allcfs.crazyflies:
                if cf.uri == uri:
                    self.cfs[drone_id] = cf
                    self.get_logger().info(f'Drone {drone_id} com URI {uri} encontrado.')
                    found = True
                    break
            if not found:
                self.get_logger().error(f'CF {drone_id} com URI {uri} não encontrado.')

        # Timer para processamento periódico (a cada 1s)
        self.timer = self.create_timer(1.0, self.timer_callback)

    # Callbacks das assinaturas

    def pose_callback(self, msg, drone_id):
        self.poses[drone_id] = msg
        self.get_logger().debug(f'Nova pose recebida do drone {drone_id}')

    def battery_callback(self, msg, drone_id):
        self.batteries[drone_id] = msg
        self.get_logger().debug(f'Novo status de bateria recebido do drone {drone_id}')

    def trajectory_callback(self, msg, drone_id):
        try:
            # O dado recebido é uma string que representa uma lista (ex: "['E0', 'RS0', ...]")
            self.actions[drone_id] = ast.literal_eval(msg.data)
            self.get_logger().debug(f'Nova trajetória codificada recebida do drone {drone_id}: {self.actions[drone_id]}')
        except Exception as e:
            self.get_logger().error(f'Erro ao processar trajetória codificada do drone {drone_id}: {e}')

    def trajectory_ids_callback(self, msg, drone_id):
        try:
            # A mensagem contém uma lista (possivelmente com listas internas)
            self.trajectory_ids_data[drone_id] = ast.literal_eval(msg.data)
            self.get_logger().debug(f'Nova trajectory_ids recebida do drone {drone_id}: {self.trajectory_ids_data[drone_id]}')
        except Exception as e:
            self.get_logger().error(f'Erro ao processar trajectory_ids do drone {drone_id}: {e}')

    def trajectory_path_callback(self, msg, drone_id):
        # msg.poses é a lista de PoseStamped
        self.trajectory_paths_data[drone_id] = msg.poses
        self.get_logger().debug(f'Nova trajectory_path recebida do drone {drone_id}')

    def compute_action_waypoints(self, trajectory_ids, trajectory_path):
        """
        A partir dos trajectory_ids e do trajectory_path “flattened”, computa para cada macro ação
        a lista de waypoints (cada waypoint é um PoseStamped). Se o elemento em trajectory_ids for um int,
        a macro ação tem um waypoint; se for uma lista, há vários waypoints.
        """
        action_waypoints = []
        index = 0
        for elem in trajectory_ids:
            if isinstance(elem, int):
                if index < len(trajectory_path):
                    action_waypoints.append([trajectory_path[index]])
                    index += 1
                else:
                    self.get_logger().error("Índice fora do alcance em trajectory_path para int")
            elif isinstance(elem, list):
                n = len(elem)
                if index + n <= len(trajectory_path):
                    action_waypoints.append(trajectory_path[index:index+n])
                    index += n
                else:
                    self.get_logger().error("Índice fora do alcance em trajectory_path para lista")
            else:
                self.get_logger().error("Tipo inesperado em trajectory_ids")
        return action_waypoints

    def timer_callback(self):
        # Tenta computar os waypoints das macro ações assim que todos os dados necessários forem recebidos
        for drone_id in self.drone_ids:
            if drone_id not in self.action_waypoints:
                if (drone_id in self.actions and 
                    drone_id in self.trajectory_ids_data and 
                    drone_id in self.trajectory_paths_data):
                    self.action_waypoints[drone_id] = self.compute_action_waypoints(
                        self.trajectory_ids_data[drone_id],
                        self.trajectory_paths_data[drone_id]
                    )
                    self.get_logger().info(f'Waypoints de ações computados para drone {drone_id}')

        # Para cada drone, verifique se a macro ação corrente já foi concluída
        all_drones_completed = True
        for drone_id in self.drone_ids:
            if drone_id not in self.action_waypoints:
                # Ainda não foram recebidos todos os dados para este drone
                all_drones_completed = False
                continue

            waypoints_list = self.action_waypoints[drone_id]
            if self.global_action_index >= len(waypoints_list):
                # Todas as macro ações já foram executadas para este drone
                continue

            # Obtenha os waypoints da macro ação atual
            current_waypoints = waypoints_list[self.global_action_index]
            sub_index = self.sub_indices[drone_id]

            # Se ainda há waypoints a serem executados nesta macro ação…
            if sub_index < len(current_waypoints):
                all_drones_completed = False
                # Se ainda não foi enviado o comando para este sub-waypoint, envie-o
                if self.last_commanded.get(drone_id, -1) < sub_index:
                    target_pose = current_waypoints[sub_index]
                    goal = np.array([
                        target_pose.pose.position.x,
                        target_pose.pose.position.y,
                        target_pose.pose.position.z
                    ])
                    yaw = 0      # Valor fixo; ajuste se necessário
                    duration = 5.0  # Pode ser adaptado conforme a distância
                    if drone_id in self.cfs:
                        self.cfs[drone_id].goTo(goal, yaw, duration)
                        self.get_logger().info(
                            f'Drone {drone_id}: goTo para waypoint {sub_index} da ação {self.global_action_index} enviado.'
                        )
                        self.last_commanded[drone_id] = sub_index
                    else:
                        self.get_logger().error(f'CF para drone {drone_id} não encontrado.')

                # Verifica se o drone já alcançou o waypoint corrente
                current_pose = self.poses.get(drone_id)
                if current_pose is not None:
                    current_pos = np.array([
                        current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z
                    ])
                    target_pos = np.array([
                        current_waypoints[sub_index].pose.position.x,
                        current_waypoints[sub_index].pose.position.y,
                        current_waypoints[sub_index].pose.position.z
                    ])
                    if np.linalg.norm(current_pos - target_pos) < self.position_threshold:
                        self.get_logger().info(
                            f'Drone {drone_id}: waypoint {sub_index} da ação {self.global_action_index} alcançado.'
                        )
                        self.sub_indices[drone_id] += 1
            # Se os waypoints da macro ação já foram percorridos, nada a fazer neste drone

        # Se todos os drones concluíram a macro ação corrente, avança para a próxima
        if all(
            (d in self.action_waypoints and 
             self.global_action_index < len(self.action_waypoints[d]) and 
             self.sub_indices[d] >= len(self.action_waypoints[d][self.global_action_index]))
            for d in self.drone_ids if d in self.action_waypoints
        ):
            self.get_logger().info(f'Macro ação {self.global_action_index} concluída por todos os drones.')
            self.global_action_index += 1
            # Reinicia os índices para os waypoints das próximas macro ações
            for d in self.drone_ids:
                self.sub_indices[d] = 0
                if d in self.last_commanded:
                    del self.last_commanded[d]

        # Apenas para debug, imprime os estados atuais
        for drone_id in self.drone_ids:
            pose = self.poses.get(drone_id)
            battery = self.batteries.get(drone_id)
            action_info = self.actions.get(drone_id, None)
            self.get_logger().debug(
                f'Drone {drone_id}: Pose: {pose}, Bateria: {battery}, Ação: {action_info}, Macro Ação: {self.global_action_index}'
            )

def main(args=None):
    #rclpy.init(args=args)
    # Obtém o número de drones a partir da variável de ambiente (default: 5)
    num_robots = int(os.getenv("NUM_ROBOTS", "5"))
    # Os drones são numerados de 1 a num_robots
    drone_ids = list(range(1, num_robots+1))
    
    node = ManageTrajectoriesNode(drone_ids)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
