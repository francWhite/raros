import time
from typing import Optional

import RPi.GPIO as GPIO

import rclpy
from raros_interfaces.action import PlayTone
from rclpy.action import ActionServer
from rclpy.node import Node


class Buzzer(Node):
    def __init__(self):
        super().__init__('buzzer')
        self.get_logger().info('buzzer node started')
        self.action_server = ActionServer(self, PlayTone, 'buzzer/play_tone', self.play_tone_callback)
        self.pin = 33
        self.buzzer: Optional[GPIO.PWM] = None

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.buzzer = GPIO.PWM(self.pin, 1000)

    def play_tone_callback(self, goal_handle):
        request = goal_handle.request
        self.get_logger().debug(f'executing goal for request: "{request}"')

        feedback_msg = PlayTone.Feedback()
        feedback_msg.remaining = request.duration

        self.buzzer.ChangeFrequency(request.frequency)
        self.buzzer.start(50)

        for i in range(0, request.duration, 100):
            sleep_duration = 100 if request.duration - i > 100 else request.duration - i
            time.sleep(sleep_duration / 1000)
            feedback_msg.remaining -= sleep_duration
            goal_handle.publish_feedback(feedback_msg)

        self.buzzer.stop()
        goal_handle.succeed()

        result = PlayTone.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Buzzer()

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
