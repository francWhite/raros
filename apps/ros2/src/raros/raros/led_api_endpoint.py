import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class LedApiEndpoint(Node):
    def __init__(self):
        super().__init__('led_api_endpoint')
        self.get_logger().info('Led API Endpoint started')
        self.subscriber = self.create_subscription(String, 'api_led', self.listener_callback, 10)
        self.publisher = self.create_publisher(Bool, 'led', 10)

    def listener_callback(self, req_msg):
        self.get_logger().info(f'receiving: "{req_msg.data}"')
        msg = Bool()
        msg.data = req_msg.data == 'on'
        self.publisher.publish(msg)
        self.get_logger().info(f'published: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = LedApiEndpoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
