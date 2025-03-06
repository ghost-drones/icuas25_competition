#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from icuas25_msgs.srv import PathService

class Teste(Node):
    def __init__(self):
        super().__init__('teste_node')
        self.client = self.create_client(PathService, '/ghost/path_planner')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.create_timer(1.0, self.request_comand)
        self.target_pose = PoseStamped()

    def request_comand(self):
        o = Point(x=1.0, y=1.0, z=1.0)
        d = Point(x=2.0, y=2.0, z=2.0)
        s = Point(x=0.0, y=0.0, z=0.0)
        
        response = self.send_request(o, d, s)  # Requisição bloqueante
        self.get_logger().info(f"Resposta: {response.path if response else None}")

    def send_request(self, origin: Point, destination: Point, support: Point):
        """Envia a requisição e espera a resposta de forma bloqueante."""
        req = PathService.Request()
        req.origin = origin
        req.destination = destination
        req.support = support

        future = self.client.call_async(req)
        
        # Loop manual para processar eventos enquanto espera
        start_time = self.get_clock().now()
        timeout = 5.0  # 5 segundos de timeout
        
        while rclpy.ok() and (self.get_clock().now() - start_time).nanoseconds < timeout * 1e9:
            rclpy.spin_once(self, timeout_sec=0.1)  # Processa eventos
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f"Erro: {e}")
                    return None
        
        self.get_logger().error("Timeout!")
        return None

def main(args=None):
    rclpy.init(args=args)
    node = Teste()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()