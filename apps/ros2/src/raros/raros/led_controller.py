import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.get_logger().info('Led Controller started')
        self.publisher = self.create_publisher(Bool, 'led', 10)
        self.toggle_state = False

    def toggle(self):
        self.toggle_state = not self.toggle_state
        msg = Bool()
        msg.data = self.toggle_state
        self.publisher.publish(msg)
        self.get_logger().info(f'published: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = LedController()

    print('Press enter to toggle the led and "q" or Ctrl+C to quit ')
    while input() != 'q':
        node.toggle()

    node.destroy_node()
    rclpy.shutdown()
