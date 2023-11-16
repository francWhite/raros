import rclpy
import time
import RPi.GPIO as GPIO

from rclpy.node import Node
from raros_interfaces.srv import PlayTone


class Buzzer(Node):
    def __init__(self):
        super().__init__('buzzer')
        self.get_logger().info('buzzer node started')
        self.service = self.create_service(PlayTone, 'buzzer/play_tone', self.play_tone_callback)
        self.pin = 33
        self.buzzer = None

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.buzzer = GPIO.PWM(self.pin, 1000)

    def play_tone_callback(self, request, response):
        self.get_logger().debug(f'received: "{request}"')

        self.buzzer.ChangeFrequency(request.frequency)
        self.buzzer.start(50)
        time.sleep(request.duration / 1000)
        self.buzzer.stop()

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Buzzer()

    try:
        node.setup_gpio()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
    finally:
        GPIO.cleanup()
