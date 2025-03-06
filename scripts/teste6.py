#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from icuas25_msgs.srv import PathService
import time

class Teste(Node):
    def __init__(self):
        super().__init__('teste_node')
        self.client = self.create_client(PathService, '/ghost/path_planner')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.create_timer(1.0, self.request_comand)
        self.service_response = None  # Armazena a resposta
        self.response_received = False  # Flag de controle

    def request_comand(self):
        response = self.send_request(
            origin=Point(x=1.0, y=1.0, z=1.0),
            destination=Point(x=2.0, y=2.0, z=2.0),
            support=Point(x=0.0, y=0.0, z=0.0)
        )
        if response:
            self.get_logger().info(f"Resposta: {response.path}")

    def send_request(self, origin: Point, destination: Point, support: Point):
        """Envia a requisição e retorna a resposta (bloqueante)."""
        req = PathService.Request()
        req.origin = origin
        req.destination = destination
        req.support = support

        future = self.client.call_async(req)
        self.service_response = None
        self.response_received = False

        # Callback para processar a resposta
        def callback(f):
            try:
                self.service_response = f.result()
            except Exception as e:
                self.get_logger().error(f"Erro: {e}")
                self.service_response = None
            self.response_received = True  # Sinaliza que a resposta chegou

        future.add_done_callback(callback)

        # Loop de espera controlado
        start_time = time.time()
        timeout = 5.0  # 5 segundos de timeout

        while rclpy.ok() and not self.response_received and (time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)  # Processa eventos ROS2

        if self.response_received:
            return self.service_response
        else:
            self.get_logger().error("Timeout!")
            return None

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