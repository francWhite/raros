from typing import Optional

import board
import pwmio
from adafruit_motor import servo

import rclpy
from raros_interfaces.srv import RotateCamera
from rclpy.node import Node
from rclpy.timer import Timer


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.get_logger().info('camera node started')

        self.service = self.create_service(RotateCamera, 'camera/rotate', self.rotate_callback)

        self.pwm_servo_horizontal: Optional[pwmio.PWMOut] = None
        self.pwm_servo_vertical: Optional[pwmio.PWMOut] = None
        self.servo_horizontal: Optional[servo.Servo] = None
        self.servo_vertical: Optional[servo.Servo] = None
        self.rotate_cleanup_timer: Optional[Timer] = None

    def setup(self):
        # BCM numbering
        self.pwm_servo_horizontal = pwmio.PWMOut(pin=board.D5, frequency=50)
        self.pwm_servo_vertical = pwmio.PWMOut(pin=board.D6, frequency=50)
        self.servo_horizontal = servo.Servo(self.pwm_servo_horizontal, min_pulse=600, max_pulse=2300,
                                            actuation_range=180)
        self.servo_vertical = servo.Servo(self.pwm_servo_vertical, min_pulse=600, max_pulse=2300, actuation_range=180)

    def cleanup(self):
        self.pwm_servo_horizontal.deinit()
        self.pwm_servo_vertical.deinit()

    def rotate_callback(self, request: RotateCamera.Request, response):
        self.get_logger().info(f'rotating to {request.angle_horizontal}°, {request.angle_vertical}°')
        # map from [-90, 90] to [0, 180]
        transformed_angle_horizontal = request.angle_horizontal + 90
        transformed_angle_vertical = request.angle_vertical + 90
        self.servo_horizontal.angle = transformed_angle_horizontal
        self.servo_vertical.angle = transformed_angle_vertical

        self.rotate_cleanup_timer = self.create_timer(0.5, self.rotate_cleanup_callback)
        return response

    def rotate_cleanup_callback(self):
        self.rotate_cleanup_timer.cancel()
        self.pwm_servo_horizontal.duty_cycle = 0
        self.pwm_servo_vertical.duty_cycle = 0


def main(args=None):
    rclpy.init(args=args)
    node = Camera()

    try:
        node.setup()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
    finally:
        node.cleanup()
