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


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        (self.active, self.steps_per_revolution, self.micro_steps,
         self.wheel_distance, self.wheel_radius, self.default_speed_linear, self.default_speed_rotation,
         self.default_speed_turn_with_radius, self.default_speed_turn_on_spot) = self.init_params()
        if not self.active:
            return
        self.get_logger().info('navigation node started')

        self.converter = Converter(self.steps_per_revolution * self.micro_steps, self.wheel_radius, self.wheel_distance)
        self.stop_service = self.create_service(EmptySrv, 'navigation/stop', self.stop_callback)
        self.move_action_server = ActionServer(self, Move, 'navigation/move', self.move_action_callback)
        self.rotate_action_server = ActionServer(self, Rotate, 'navigation/rotate', self.rotate_action_callback)
        self.turn_action_server = ActionServer(self, Turn, 'navigation/turn', self.turn_action_callback)

        self.stepper_move_publisher = self.create_publisher(StepperMovement, 'arduino_stepper/move', 10)
        self.stepper_turn_publisher = self.create_publisher(StepperMovement, 'arduino_stepper/turn', 10)
        self.stop_publisher = self.create_publisher(EmptyMsg, 'arduino_stepper/stop', 10)

    def init_params(self):
        self.declare_parameter('active', True)
        self.declare_parameter('steps_per_revolution', 1600)
        self.declare_parameter('micro_steps', 4)
        self.declare_parameter('wheel.distance', 22.0)
        self.declare_parameter('wheel.radius', 4.2)
        self.declare_parameter('default_speed.linear', 30)
        self.declare_parameter('default_speed.rotation', 15)
        self.declare_parameter('default_speed.turn_with_radius', 40)
        self.declare_parameter('default_speed.turn_on_spot', 20)
        active = self.get_parameter('active').get_parameter_value().bool_value
        steps_per_revolution = self.get_parameter('steps_per_revolution').get_parameter_value().integer_value
        micro_steps = self.get_parameter('micro_steps').get_parameter_value().integer_value
        wheel_distance = self.get_parameter('wheel.distance').get_parameter_value().double_value
        wheel_radius = self.get_parameter('wheel.radius').get_parameter_value().double_value
        default_speed_linear = self.get_parameter('default_speed.linear').get_parameter_value().integer_value
        default_speed_rotation = self.get_parameter('default_speed.rotation').get_parameter_value().integer_value
        default_speed_turn_with_radius = self.get_parameter(
            'default_speed.turn_with_radius').get_parameter_value().integer_value
        default_speed_turn_on_spot = self.get_parameter(
            'default_speed.turn_on_spot').get_parameter_value().integer_value
        return (active, steps_per_revolution, micro_steps, wheel_distance, wheel_radius,
                default_speed_linear, default_speed_rotation, default_speed_turn_with_radius,
                default_speed_turn_on_spot)

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
        stepper_msg.left.speed = self.get_linear_speed(request.speed)
        stepper_msg.left.speed_start = self.get_linear_speed(request.speed_start)
        stepper_msg.right.steps = self.converter.distance_to_steps(request.distance, request.direction)
        stepper_msg.right.speed = self.get_linear_speed(request.speed)
        stepper_msg.right.speed_start = self.get_linear_speed(request.speed_start)

        self.stepper_move_publisher.publish(stepper_msg)

        observe_node = ObserveAction(goal_handle, self.move_action_create_feedback_msg)
        while goal_handle.status == GoalStatus.STATUS_EXECUTING:
            rclpy.spin_once(observe_node)

        observe_node.destroy_node()
        result = Move.Result()
        return result

    def get_linear_speed(self, requested_speed):
        if requested_speed == -1:
            return self.default_speed_linear
        return requested_speed

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
        stepper_msg.left.speed = self.default_speed_rotation
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = self.default_speed_rotation
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
                                                                        self.default_speed_turn_with_radius,
                                                                        request.direction)
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
        stepper_msg.left.speed = self.default_speed_turn_on_spot
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = self.default_speed_turn_on_spot
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
    if not node.active:
        node.get_logger().info('navigation node not active, exiting')
        return

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
