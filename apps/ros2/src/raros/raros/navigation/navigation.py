from raros.navigation.step_converter import Converter
import rclpy
from action_msgs.msg import GoalStatus
from raros.navigation.observe_action import ObserveAction
from raros_interfaces.action import Move, Rotate, Turn
from raros_interfaces.msg import StepperMovement, StepperFeedback
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv

DEFAULT_MOVE_SPEED = 30
DEFAULT_TURN_SPEED = 20
DEFAULT_ROTATE_SPEED = 15


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.get_logger().info('navigation node started')

        self.converter = Converter()
        self.stop_service = self.create_service(EmptySrv, 'navigation/stop', self.stop_callback)
        self.move_action_server = ActionServer(self, Move, 'navigation/move', self.move_action_callback)
        self.rotate_action_server = ActionServer(self, Rotate, 'navigation/rotate', self.rotate_action_callback)
        self.turn_action_server = ActionServer(self, Turn, 'navigation/turn', self.turn_action_callback)

        self.stepper_move_publisher = self.create_publisher(StepperMovement, 'arduino_stepper/move', 10)
        self.stepper_turn_publisher = self.create_publisher(StepperMovement, 'arduino_stepper/turn', 10)
        self.stop_publisher = self.create_publisher(EmptyMsg, 'arduino_stepper/stop', 10)

    def stop_callback(self, request, response):
        self.get_logger().info(f'received stop request')
        self.stop_publisher.publish(EmptyMsg())
        return response

    def move_action_callback(self, goal_handle: ServerGoalHandle):
        request: Move.Goal = goal_handle.request
        self.get_logger().info(f'executing goal: "{request}"')

        self.stop_publisher.publish(EmptyMsg())

        feedback_msg = Move.Feedback()
        feedback_msg.remaining_distance = request.distance
        goal_handle.publish_feedback(feedback_msg)

        stepper_msg = StepperMovement()
        stepper_msg.left.steps = self.converter.distance_to_steps(request.distance, request.direction)
        stepper_msg.left.speed = request.speed if request.speed != -1 else DEFAULT_MOVE_SPEED
        stepper_msg.left.speed_start = request.speed_start if request.speed_start != -1 else DEFAULT_MOVE_SPEED
        stepper_msg.right.steps = self.converter.distance_to_steps(request.distance, request.direction)
        stepper_msg.right.speed = request.speed if request.speed != -1 else DEFAULT_MOVE_SPEED
        stepper_msg.right.speed_start = request.speed_start if request.speed_start != -1 else DEFAULT_MOVE_SPEED

        self.stepper_move_publisher.publish(stepper_msg)

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

        self.stop_publisher.publish(EmptyMsg())

        feedback_msg = Rotate.Feedback()
        feedback_msg.remaining_angle = request.angle
        goal_handle.publish_feedback(feedback_msg)

        steps_left, steps_right = self.converter.angle_to_steps_for_rotation(request.angle, request.direction)
        stepper_msg = StepperMovement()
        stepper_msg.left.steps = steps_left
        stepper_msg.left.speed = DEFAULT_ROTATE_SPEED
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = DEFAULT_ROTATE_SPEED
        stepper_msg.right.speed_start = stepper_msg.right.speed

        self.stepper_move_publisher.publish(stepper_msg)

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

    def turn_action_callback(self, goal_handle: ServerGoalHandle):
        request: Turn.Goal = goal_handle.request
        self.get_logger().info(f'executing goal: "{request}"')

        self.stop_publisher.publish(EmptyMsg())

        feedback_msg = Turn.Feedback()
        feedback_msg.remaining_angle = request.angle
        goal_handle.publish_feedback(feedback_msg)

        if request.radius != 0:
            self.publish_stepper_movement_for_turn_with_radius(request)
        else:
            self.publish_stepper_movement_for_turn(request)

        observe_node = ObserveAction(goal_handle, self.turn_action_create_feedback_msg)
        while goal_handle.status == GoalStatus.STATUS_EXECUTING:
            rclpy.spin_once(observe_node)

        observe_node.destroy_node()
        result = Turn.Result()
        return result

    def publish_stepper_movement_for_turn_with_radius(self, request: Turn.Goal):
        steps_left, steps_right = self.converter.angle_and_radius_to_steps_for_turn(request.angle,
                                                                                    request.radius,
                                                                                    request.direction)

        speed_left, speed_right = self.converter.turning_steps_to_speed(steps_left, steps_right,
                                                                        DEFAULT_MOVE_SPEED, request.direction)
        stepper_msg = StepperMovement()
        stepper_msg.left.steps = steps_left
        stepper_msg.left.speed = speed_left
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = speed_right
        stepper_msg.right.speed_start = stepper_msg.right.speed

        self.stepper_turn_publisher.publish(stepper_msg)

    def publish_stepper_movement_for_turn(self, request: Turn.Goal):
        steps_left, steps_right = self.converter.angle_to_steps_for_turn(request.angle, request.direction)

        stepper_msg = StepperMovement()
        stepper_msg.left.steps = steps_left
        stepper_msg.left.speed = DEFAULT_TURN_SPEED
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = DEFAULT_TURN_SPEED
        stepper_msg.right.speed_start = stepper_msg.right.speed

        # as the robot is turning on the spot and therefore only one motor is active,
        # the normal stepper movement publisher is used
        self.stepper_move_publisher.publish(stepper_msg)

    @staticmethod
    def turn_action_create_feedback_msg(msg: StepperFeedback):
        feedback_msg = Turn.Feedback()
        # TODO actually calculate remaining angle
        feedback_msg.remaining_angle = float(msg.remaining_steps_left)
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
