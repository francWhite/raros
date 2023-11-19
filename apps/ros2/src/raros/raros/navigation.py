import rclpy
from rclpy.node import Node


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.get_logger().info('navigation node started')


def main(args=None):
    rclpy.init(args=args)
    node = Navigation()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
