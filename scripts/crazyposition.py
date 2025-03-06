import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CrazyfliePosition:
    def __init__(self, cf_id):
        self.cf_id = cf_id
        self.position = None

        # Inicialize o ROS2 e crie um nó
        rclpy.init()
        self.node = Node('crazyflie_position_node')

        # Inscreva-se no tópico de pose do drone
        self.subscription = self.node.create_subscription(
            PoseStamped,
            f'/cf_{self.cf_id}/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg:PoseStamped):
        # Atualize a posição atual do drone
        self.position = msg.pose.position

    def get_position(self) -> PoseStamped:
        # Processe uma mensagem ROS2 para atualizar a posição
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.position

    def shutdown(self):
        # Encerre o nó ROS2
        self.node.destroy_node()
        rclpy.shutdown()