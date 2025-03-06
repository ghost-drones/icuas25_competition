#!/usr/bin/env python

from crazyflie_py import Crazyswarm
from crazyflie_py.crazyflie import Crazyflie
from crazyflie_py.uav_trajectory import Trajectory, Polynomial4D

import numpy as np
from typing import List, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CrazyfliePositionManager(Node):
    def __init__(self, cf_ids: List[str]):
        super().__init__('crazyflie_position_manager')
        self.positions: Dict[str, PoseStamped] = {}
        self.cf_ids = cf_ids
        # Cria uma inscrição para cada drone
        for cf_id in cf_ids:
            self.create_subscription(
                PoseStamped,
                f'/cf_{cf_id}/pose',
                lambda msg, id=cf_id: self.pose_callback,
                10
            )

    def pose_callback(self, msg: PoseStamped):
        # Atualiza a posição do drone correspondente
        for cf_id in self.cf_ids:
            self.positions[cf_id] = msg.pose.position

    def get_position(self ,cf_id: str) -> PoseStamped:
        # Retorna a posição do drone especificado
        self.get_logger().error("TEM X POSITIONS, X:",len(self.positions))
        for key in self.positions:
            self.get_logger().error(str(key)," , ",type(key)," , ",self.positions[key])
            
        return self.positions[cf_id]

def get_position(node:CrazyfliePositionManager ,cf_id: str) -> PoseStamped:
    # Retorna a posição do drone especificado
    rclpy.spin_once(node, timeout_sec=0.1)
    
    return node.get_position(cf_id)

def create_linear_trajectory(start_pos:np.array, end_pos:np.array, yaw:float, duration:float) -> Trajectory:
    """Cria uma trajetória linear entre dois pontos com yaw constante.

    Args:
        start_pos (list): Posição inicial [x, y, z].
        end_pos (list): Posição final [x, y, z].
        yaw (float): Ângulo de yaw (em radianos).
        duration (float): Duração da trajetória (em segundos).

    Returns:
        Trajectory: Objeto de trajetória.
    """
    # Cria uma nova trajetória
    traj = Trajectory()

    # Coeficientes para cada eixo (4º grau)
    def calculate_coeff(start:np.array, end:np.array, duration:float) -> np.array:
        return np.array([
            (end - start) / duration,  # Termo linear
            0.0,  # Termo quadrático
            0.0,  # Termo cúbico
            0.0,  # Termo quártico
            0.0   # Termo quíntico
        ])

    # Calcula os coeficientes para x, y, z
    px = calculate_coeff(start_pos[0], end_pos[0], duration)
    py = calculate_coeff(start_pos[1], end_pos[1], duration)
    pz = calculate_coeff(start_pos[2], end_pos[2], duration)

    # Coeficientes para yaw (constante)
    pyaw = np.array([yaw] + [0.0] * 5)

    
    # Cria um polinômio 4D com os coeficientes calculados
    poly = Polynomial4D(
        duration=duration,
        px=px,
        py=py,
        pz=pz,
        pyaw=pyaw
    )
    # Define os polinômios da trajetória
    traj.polynomials = [poly]
    traj.duration = duration

    return traj

def main():
    # Inicializa o Crazyswarm
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
     = swarm.allcfs

    # Lista para armazenar os drones
    cfs: List[Crazyflie] = []
    for cf in allcfs.crazyflies:
        cfs.append(cf)

    cf_ids = [str(i + 1) for i in range(len(cfs))]

    position_manager = CrazyfliePositionManager(cf_ids)
    # Aguarda um pouco para garantir que as poses sejam atualizadas
    timeHelper.sleep(2.0)

    # Define a posição de destino
    pos_1 = np.array([5.0, 1.2, 4.0])  # Exemplo de posição

    # Cria e executa a primeira trajetória
    print("Preparando trajetória 0")
    traj1 = create_linear_trajectory(
        start_pos=np.array([0.0, 0.0, 0.0]),
        end_pos=pos_1,
        yaw=np.pi / 2,
        duration=15.0
    )

    print("Upando trajetória 0")
    cfs[0].uploadTrajectory(
        trajectoryId=0,
        pieceOffset=0,
        trajectory=traj1
    )
    
    print("Começando trajetória 0")
    cfs[0].startTrajectory(
        trajectoryId=0,
        timescale=1.0,
        relative=False
    )
    print("Enviado o comando de inicio da trajetória 0")
    timeHelper.sleep(15.0)
    pos_2 = np.array([5.0, 1.2, 4.0]) 
    
    print("Pegando posicao")
    current_pos = get_position(position_manager,'1').pose.position
    current_pos_array = np.array([current_pos.x, current_pos.y, current_pos.z])
    print("Posicao: ", current_pos_array)
    
    print("Preparando trajetória 1")
    traj2 = create_linear_trajectory(
        start_pos=current_pos_array,
        end_pos=pos_2,
        yaw=np.pi / 2,
        duration=15.0
    )

    print("Upando trajetória 1")
    cfs[0].uploadTrajectory(
        trajectoryId=1,
        pieceOffset=0,
        trajectory=traj2
    )
    
    print("Iniciando trajetória 1")
    cfs[0].startTrajectory(
        trajectoryId=1,
        timescale=1.0,
        relative=False
    )
    print("Enviado o comando de inicio da trajetória 1")
    timeHelper.sleep(15.0)
    
    # Pouso
    print("Inicio do pouso")
    cfs[0].land(targetHeight=0.02, duration=2.0)
    print("Pousando")
    timeHelper.sleep(3.0)
    print("Pousado")
    # Encerra o nó ROS2
    print("Destruindo Nó")
    position_manager.destroy_node()
    print("Destruindo Nó concluido")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# take off?  parece que eu envio o trajeto, ele não reclama, só não faz kkkk
