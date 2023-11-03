import rclpy
import RPi.GPIO as GPIO

from rclpy.node import Node
from raros_interfaces.srv import SetMagnetState


class Magnet(Node):
    def __init__(self):
        super().__init__('magnet')
        self.get_logger().info('magnet node started')
        self.service = self.create_service(SetMagnetState, 'set_magnet', self.set_magnet_state_callback)
        self.pin = 22

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)

    def set_magnet_state_callback(self, request, response):
        # TODO: add error handling
        self.get_logger().info(f'received: "{request.state}"')
        GPIO.output(self.pin, GPIO.HIGH if request.state else GPIO.LOW)
        return response


def main(args=None):
    rclpy.init(args=args)

    node = Magnet()
    node.setup_gpio()

    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
    finally:
        GPIO.cleanup()
