from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Point
from icuas25_msgs.srv import PathService

class Teste(Node):
    def __init__(self):
        super().__init__('teste_node')
        self.cli = self.create_client(PathService, '/ghost/path_planner')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PathService.Request()

    def send_request(self):
        origin = Point(x=1.0, y=1.0, z=1.0)
        destination = Point(x=2.0, y=2.0, z=2.0)
        support = Point(x=0.0, y=0.0, z=0.0)
        self.req.origin = Point(x=origin.x, y=origin.y, z=origin.z)
        self.req.destination = Point(x=destination.x, y=destination.y, z=destination.z)  # Corrigido
        self.req.support = Point(x=support.x, y=support.y, z=support.z)

        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    node = Teste()
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.get_logger().info(f"res: {response}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()