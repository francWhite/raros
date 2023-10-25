import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.get_logger().info('Simple Subscriber started')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Receiving: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
