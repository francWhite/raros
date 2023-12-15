import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class Magnet(Node):
    def __init__(self):
        super().__init__('magnet')
        self.get_logger().info('magnet node started')
        self.service = self.create_service(SetBool, 'magnet/set_state', self.set_magnet_state_callback)
        self.update_magnet_status_publisher = self.create_publisher(Bool, 'status/magnet_active', 10)
        self.pin = 22

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)

    def set_magnet_state_callback(self, request, response):
        self.get_logger().info(f'received: "{request.data}"')
        GPIO.output(self.pin, GPIO.HIGH if request.data else GPIO.LOW)
        self.update_magnet_status_publisher.publish(Bool(data=request.data))

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
