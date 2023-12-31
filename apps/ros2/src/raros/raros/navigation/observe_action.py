import uuid
from typing import Callable, Any

from action_msgs.msg import GoalStatus
from raros_interfaces.msg import StepperFeedback, StepperStatus
from rclpy import qos
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node


class ObserveAction(Node):
    def __init__(self, goal_handle: ServerGoalHandle, create_feedback_msg_callback: Callable[[StepperFeedback], Any]):
        super().__init__(f'observe_action_{uuid.uuid4().hex}', use_global_arguments=False)
        self.goal_handle = goal_handle
        self.create_feedback_msg_callback = create_feedback_msg_callback

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
        if msg.moving is False and self.goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self.goal_handle.succeed()

    def feedback_callback(self, msg: StepperFeedback):
        feedback_msg = self.create_feedback_msg_callback(msg)
        self.goal_handle.publish_feedback(feedback_msg)
