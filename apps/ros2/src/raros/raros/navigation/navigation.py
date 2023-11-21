from raros.navigation.step_converter import Converter
import rclpy
from action_msgs.msg import GoalStatus
from raros.navigation.observe_action import ObserveAction
from raros_interfaces.action import Move, Rotate
from raros_interfaces.msg import StepperMovement, StepperFeedback
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.get_logger().info('navigation node started')

        self.converter = Converter()
        self.stop_service = self.create_service(EmptySrv, 'navigation/stop', self.stop_callback)
        self.move_action_server = ActionServer(self, Move, 'navigation/move', self.move_action_callback)
        self.rotate_action_server = ActionServer(self, Rotate, 'navigation/rotate', self.rotate_action_callback)

        self.stepper_publisher = self.create_publisher(StepperMovement, '/raros/arduino_stepper/move', 10)
        self.stop_publisher = self.create_publisher(EmptyMsg, '/raros/arduino_stepper/stop', 10)

    def stop_callback(self, request, response):
        self.get_logger().info(f'received stop request')
        self.stop_publisher.publish(EmptyMsg())
        return response

    def move_action_callback(self, goal_handle: ServerGoalHandle):
        request: Move.Goal = goal_handle.request
        self.get_logger().info(f'executing goal: "{request}"')

        feedback_msg = Move.Feedback()
        feedback_msg.remaining_distance = request.distance
        goal_handle.publish_feedback(feedback_msg)

        stepper_msg = StepperMovement()
        stepper_msg.left.steps = self.converter.distance_to_steps(request.distance, request.direction)
        stepper_msg.left.speed = int(request.speed) if request.speed != -1 else 30
        stepper_msg.right.steps = self.converter.distance_to_steps(request.distance, request.direction)
        stepper_msg.right.speed = int(request.speed) if request.speed != -1 else 30

        self.stepper_publisher.publish(stepper_msg)

        observe_node = ObserveAction(goal_handle, self.move_action_create_feedback_msg)
        while goal_handle.status == GoalStatus.STATUS_EXECUTING:
            rclpy.spin_once(observe_node)

        observe_node.destroy_node()
        result = Move.Result()
        return result

    def move_action_create_feedback_msg(self, msg: StepperFeedback):
        feedback_msg = Move.Feedback()
        remaining_steps = max(msg.remaining_steps_left, msg.remaining_steps_right)
        feedback_msg.remaining_distance = self.converter.steps_to_distance(remaining_steps)
        return feedback_msg

    def rotate_action_callback(self, goal_handle: ServerGoalHandle):
        request: Rotate.Goal = goal_handle.request
        self.get_logger().info(f'executing goal: "{request}"')

        feedback_msg = Rotate.Feedback()
        feedback_msg.remaining_angle = request.angle
        goal_handle.publish_feedback(feedback_msg)

        steps_left, steps_right = self.converter.angle_to_steps_for_rotation(request.angle, request.direction)
        stepper_msg = StepperMovement()
        stepper_msg.left.steps = steps_left
        stepper_msg.left.speed = 15
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = 15

        self.stepper_publisher.publish(stepper_msg)

        observe_node = ObserveAction(goal_handle, self.rotate_action_create_feedback_msg)
        while goal_handle.status == GoalStatus.STATUS_EXECUTING:
            rclpy.spin_once(observe_node)

        observe_node.destroy_node()
        result = Rotate.Result()
        return result

    def rotate_action_create_feedback_msg(self, msg: StepperFeedback):
        feedback_msg = Rotate.Feedback()
        remaining_steps = max(msg.remaining_steps_left, msg.remaining_steps_right)
        feedback_msg.remaining_angle = self.converter.steps_to_angle(remaining_steps)
        return feedback_msg


def main(args=None):
    rclpy.init(args=args)
    node = Navigation()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
