import rclpy
import RPi.GPIO as GPIO

from rclpy.node import Node
from std_srvs.srv import SetBool


class Magnet(Node):
    def __init__(self):
        super().__init__('magnet')
        self.get_logger().info('magnet node started')
        self.service = self.create_service(SetBool, 'magnet/set_state', self.set_magnet_state_callback)
        self.pin = 22

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)

    def set_magnet_state_callback(self, request, response):
        self.get_logger().info(f'received: "{request.data}"')
        GPIO.output(self.pin, GPIO.HIGH if request.data else GPIO.LOW)

        response.success = True
        response.message = 'Magnet is on' if request.data else 'Magnet is off'
        return response


def main(args=None):
    rclpy.init(args=args)

    node = Magnet()

    try:
        node.setup_gpio()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
    finally:
        GPIO.cleanup()
