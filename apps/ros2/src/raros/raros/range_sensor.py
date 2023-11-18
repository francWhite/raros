import math
from typing import Optional

import board
import pwmio
from adafruit_hcsr04 import HCSR04
from adafruit_motor import servo

import rclpy
from raros_interfaces.msg import Distance
from raros_interfaces.srv import RotateRangeSensor
from rclpy.node import Node
from rclpy.timer import Timer


class RangeSensor(Node):
    def __init__(self):
        super().__init__('range_sensor')
        self.get_logger().info('range_sensor node started')
        self.publisher = self.create_publisher(Distance, 'range_sensor/distance', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.service = self.create_service(RotateRangeSensor, 'range_sensor/rotate', self.rotate_callback)

        self.pwm_servo: Optional[pwmio.PWMOut] = None
        self.servo: Optional[servo.Servo] = None
        self.sensor_front: Optional[HCSR04] = None
        self.sensor_back: Optional[HCSR04] = None
        self.rotate_cleanup_timer: Optional[Timer] = None

    def setup(self):
        # BCM numbering
        self.pwm_servo = pwmio.PWMOut(pin=board.D12, frequency=50)
        self.servo = servo.Servo(self.pwm_servo, min_pulse=600, max_pulse=2300, actuation_range=180)
        self.sensor_front = HCSR04(trigger_pin=board.D23, echo_pin=board.D24)
        self.sensor_back = HCSR04(trigger_pin=board.D8, echo_pin=board.D7)

    def cleanup(self):
        self.sensor_front.deinit()
        self.sensor_back.deinit()
        self.pwm_servo.deinit()

    def timer_callback(self):
        distance_front = self._read_distance(self.sensor_front)
        distance_back = self._read_distance(self.sensor_back)
        msg = Distance()
        msg.front = distance_front if distance_front is not None else math.inf
        msg.back = distance_back if distance_back is not None else math.inf
        self.publisher.publish(msg)

    def rotate_callback(self, request, response: RotateRangeSensor.Response):
        self.get_logger().info(f'rotating to {request.angle}°')
        transformed_angle = request.angle + 90  # map from [-90, 90] to [0, 180]
        self.servo.angle = transformed_angle
        self.rotate_cleanup_timer = self.create_timer(1, self.rotate_cleanup_callback)
        return response

    def rotate_cleanup_callback(self):
        self.rotate_cleanup_timer.cancel()
        self.pwm_servo.duty_cycle = 0

    @staticmethod
    def _read_distance(sensor: HCSR04):
        try:
            return sensor.distance
        except RuntimeError:
            return None


def main(args=None):
    rclpy.init(args=args)

    node = RangeSensor()

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
