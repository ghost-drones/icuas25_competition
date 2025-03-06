#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,PoseStamped
from icuas25_msgs.srv import PathService
import time

class Teste(Node):
    def __init__(self):
        super().__init__('teste_node')  # Inicializa o nó

        # Cria o cliente para o serviço /ghost/path_planner
        self.client_path_planner = self.create_client(PathService, '/ghost/path_planner')
        while not self.client_path_planner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.response = None 
        self.request_comand()
        self.get_logger().info(f"Precisa fazer uma requisição para que todas as proximas estejam limpas de valor, senão ele printa None na primeira.")
        # Cria um timer para chamar request_comand a cada 1 segundo
        self.create_timer(1.0, self.request_comand)
        self.target_pose = PoseStamped()
    
    def request_comand(self):
        # Define os pontos de origem, destino e suporte
        o = Point(x=1.0, y=1.0, z=1.0)
        d = Point(x=2.0, y=2.0, z=2.0)
        s = Point(x=0.0, y=0.0, z=0.0)

        # Envia a requisição
        self.send_request(o, d, s)
        time.sleep(0.1)
        self.get_logger().info(f"AAAAAAAAAAAAAAA\nService response: {self.response}")
        
        if self.response is not None:
            self.target_pose.pose.position = self.response.path
        
    def send_request(self, origin: Point, destination: Point, support: Point) -> Point:
        """
        Envia requisição para achar o melhor ponto para uma trajetória alinhada.
        Args:
            origin: ponto de origem, Point 
            destination: ponto de destino, Point
            support: ponto de suporte, Point
        
        Return:
            path: melhor ponto para fazer um caminho mais otimizado
        """
        # Cria a requisição
        request = PathService.Request()
        request.origin = Point(x=origin.x, y=origin.y, z=origin.z)
        request.destination = Point(x=destination.x, y=destination.y, z=destination.z)  # Corrigido
        request.support = Point(x=support.x, y=support.y, z=support.z)

        # Log da requisição
        self.get_logger().info(f"Sending request: {request}")

        # Envia a requisição de forma assíncrona
        future = self.client_path_planner.call_async(request)

        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        """
        Callback para processar a resposta do serviço.
        """
        try: 
            self.response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        print(id)
    
def main(args=None):
    rclpy.init(args=args)
    
    node = Teste()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()