from typing import Optional

import board
from adafruit_tcs34725 import TCS34725
from busio import I2C

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA


class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.active, self.gain, self.integration_time = self.init_params()
        if not self.active:
            return
        self.get_logger().info('color_sensor node started')

        self.publisher = self.create_publisher(ColorRGBA, 'color_sensor/color', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i2c: Optional[I2C] = None
        self.sensor: Optional[TCS34725] = None

    def init_params(self):
        self.declare_parameter('active', True)
        self.declare_parameter('gain', 16)
        self.declare_parameter('integration_time', 120)
        active = self.get_parameter('active').get_parameter_value().bool_value
        gain = self.get_parameter('gain').get_parameter_value().integer_value
        integration_time = self.get_parameter('integration_time').get_parameter_value().integer_value
        return active, gain, integration_time

    def setup_i2c(self):
        self.i2c = board.I2C()
        self.sensor = TCS34725(self.i2c)
        self.sensor.gain = self.gain
        self.sensor.integration_time = self.integration_time

    def timer_callback(self):
        color_rgb = self.sensor.color_rgb_bytes
        msg = ColorRGBA()
        msg.r = float(color_rgb[0])
        msg.g = float(color_rgb[1])
        msg.b = float(color_rgb[2])
        msg.a = 1.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ColorSensor()
    if not node.active:
        node.get_logger().info('color_sensor node not active, exiting')
        return

    try:
        node.setup_i2c()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
    finally:
        board.I2C().deinit()
