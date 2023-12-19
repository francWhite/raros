import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class Magnet(Node):
    def __init__(self):
        super().__init__('magnet')
        self.active, self.pin = self.init_params()
        if not self.active:
            return
        self.get_logger().info('magnet node started')

        self.service = self.create_service(SetBool, 'magnet/set_state', self.set_magnet_state_callback)
        self.update_magnet_status_publisher = self.create_publisher(Bool, 'status/magnet_active', 10)

    def init_params(self):
        self.declare_parameter('active', True)
        self.declare_parameter('pin', 25)
        active = self.get_parameter('active').get_parameter_value().bool_value
        pin = self.get_parameter('pin').get_parameter_value().integer_value
        return active, pin

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
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
    if not node.active:
        node.get_logger().info('magnet node not active, exiting')
        return

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
