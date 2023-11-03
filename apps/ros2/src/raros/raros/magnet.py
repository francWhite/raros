import rclpy
import RPi.GPIO as GPIO

from rclpy.node import Node
from std_msgs.msg import Bool


class Magnet(Node):
    def __init__(self):
        super().__init__('magnet')
        self.get_logger().info('magnet node started')
        self.subscriber = self.create_subscription(Bool, 'magnet', self.listener_callback, 10)
        self.pin = 22

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)

    def listener_callback(self, msg):
        self.get_logger().info(f'received: "{msg.data}"')
        GPIO.output(self.pin, GPIO.HIGH if msg.data else GPIO.LOW)


def main(args=None):
    rclpy.init(args=args)

    node = Magnet()
    node.setup_gpio()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    GPIO.cleanup()
