import raros.navigation.step_converter as converter
from raros_interfaces.action import Move
from raros_interfaces.msg import StepperFeedback, StepperStatus
from rclpy import qos
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node


class ObserveAction(Node):
    def __init__(self, goal_handle: ServerGoalHandle, feedback_msg: Move.Feedback):
        super().__init__('wait')
        self.goal_handle = goal_handle
        self.action_feedback_msg = feedback_msg

        self.status_subscriber = self.create_subscription(
            StepperStatus,
            '/raros/arduino_stepper/status',
            self.status_callback,
            10)

        self.feedback_subscriber = self.create_subscription(
            StepperFeedback,
            '/raros/arduino_stepper/feedback',
            self.feedback_callback,
            qos_profile=qos.qos_profile_sensor_data)

    def status_callback(self, msg: StepperStatus):
        self.get_logger().info(f'received: "{msg}"')
        if msg.moving is False:
            self.goal_handle.succeed()

    def feedback_callback(self, msg: StepperFeedback):
        remaining_steps = max(msg.remaining_steps_left, msg.remaining_steps_right)
        self.action_feedback_msg.remaining_distance = converter.steps_to_distance(remaining_steps)
        self.goal_handle.publish_feedback(self.action_feedback_msg)
