import base64
from typing import Optional

import cv2
import pwmio
from adafruit_blinka.board.raspberrypi.raspi_4b import pin
from adafruit_motor import servo
from cv2 import VideoCapture

import rclpy
from raros_interfaces.srv import RotateCamera, CaptureImage
from rclpy.node import Node
from rclpy.timer import Timer


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.active, self.pwm_pin_servo_horizontal, self.pwm_pin_servo_vertical = self.init_params()
        if not self.active:
            return
        self.get_logger().info('camera node started')

        self.rotate_service = self.create_service(RotateCamera, 'camera/rotate', self.rotate_callback)
        self.capture_service = self.create_service(CaptureImage, 'camera/capture', self.capture_callback)

        self.camera: Optional[VideoCapture] = None
        self.pwm_servo_horizontal: Optional[pwmio.PWMOut] = None
        self.pwm_servo_vertical: Optional[pwmio.PWMOut] = None
        self.servo_horizontal: Optional[servo.Servo] = None
        self.servo_vertical: Optional[servo.Servo] = None
        self.rotate_cleanup_timer: Optional[Timer] = None

    def init_params(self):
        self.declare_parameter('active', True)
        self.declare_parameter('pwm_pin_servo_horizontal', 29)
        self.declare_parameter('pwm_pin_servo_vertical', 31)
        active = self.get_parameter('active').get_parameter_value().bool_value
        pwm_pin_servo_vertical = self.get_parameter('pwm_pin_servo_horizontal').get_parameter_value().integer_value
        pwm_pin_servo_horizontal = self.get_parameter('pwm_pin_servo_vertical').get_parameter_value().integer_value
        return active, pwm_pin_servo_horizontal, pwm_pin_servo_vertical

    def setup(self):
        # BCM numbering
        self.pwm_servo_horizontal = pwmio.PWMOut(pin=pin.Pin(self.pwm_pin_servo_horizontal), frequency=50)
        self.pwm_servo_vertical = pwmio.PWMOut(pin=pin.Pin(self.pwm_pin_servo_vertical), frequency=50)
        self.servo_horizontal = servo.Servo(self.pwm_servo_horizontal, min_pulse=600, max_pulse=2300,
                                            actuation_range=180)
        self.servo_vertical = servo.Servo(self.pwm_servo_vertical, min_pulse=600, max_pulse=2300, actuation_range=180)
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def cleanup(self):
        self.pwm_servo_horizontal.deinit()
        self.pwm_servo_vertical.deinit()
        self.camera.release()

    def capture_callback(self, request, response: CaptureImage.Response):
        self.get_logger().info(f'capturing image')

        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warn('could not read frame')
            return response

        ret, encoded_frame = cv2.imencode('.jpg', frame)
        base64_str = base64.b64encode(encoded_frame.tobytes()).decode('utf-8')
        response.image_base64 = base64_str
        return response

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
    if not node.active:
        node.get_logger().info('camera node not active, exiting')
        return

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
