import rclpy
from rclpy.node import Node
from octomap_msgs.srv import GetOctomap
from octomap_msgs.msg import Octomap

class OctomapPublisher(Node):
    def __init__(self):
        super().__init__('octomap_publisher')
        
        # Create a client for the GetOctomap service
        self.cli = self.create_client(GetOctomap, '/octomap_binary')
        # Create a publisher for the Octomap messages
        self.pub = self.create_publisher(Octomap, '/octomap', 10)
        # Create a timer to periodically trigger service requests
        self.timer = self.create_timer(1.0, self.timer_callback)  # Adjust the interval as needed

    def timer_callback(self):
        """Timer callback to send service requests periodically."""
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service /octomap_binary not available, waiting...')
            return

        # Create and send the request
        request = GetOctomap.Request()
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        """Handle the response from the GetOctomap service."""
        try:
            response = future.result()
            self.process_octomap(response)
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

    def process_octomap(self, response):
        """Process the octomap and publish it."""
        octomap = response.map
        self.pub.publish(octomap)
        self.get_logger().info('Octomap received and published.')

def main(args=None):
    rclpy.init(args=args)
    octomap_publisher = OctomapPublisher()
    try:
        rclpy.spin(octomap_publisher)
    except KeyboardInterrupt:
        octomap_publisher.get_logger().info('Shutting down octomap publisher...')
    finally:
        octomap_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()