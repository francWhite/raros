import math

import rclpy
import board

from typing import Optional
from adafruit_hcsr04 import HCSR04

from rclpy.node import Node
from raros_interfaces.msg import Distance


class RangeSensor(Node):
    def __init__(self):
        super().__init__('range_sensor')
        self.get_logger().info('range_sensor node started')
        self.publisher = self.create_publisher(Distance, 'range_sensor/distance', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.sensor_front: Optional[HCSR04] = None
        self.sensor_back: Optional[HCSR04] = None

    def setup_sensors(self):
        # BCM numbering
        self.sensor_front = HCSR04(trigger_pin=board.D23, echo_pin=board.D24)
        self.sensor_back = HCSR04(trigger_pin=board.D8, echo_pin=board.D7)

    def cleanup_sensors(self):
        self.sensor_front.deinit()
        self.sensor_back.deinit()

    def timer_callback(self):
        distance_front = self._read_distance(self.sensor_front)
        distance_back = self._read_distance(self.sensor_back)

        msg = Distance()
        msg.front = distance_front if distance_front is not None else math.inf
        msg.back = distance_back if distance_back is not None else math.inf
        self.publisher.publish(msg)

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
        node.setup_sensors()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
    finally:
        node.cleanup_sensors()
