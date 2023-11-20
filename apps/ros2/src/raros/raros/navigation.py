import rclpy
from action_msgs.msg import GoalStatus
from raros_interfaces.action import Move
from raros_interfaces.msg import StepperMovement, StepperFeedback, StepperStatus
from rclpy import qos
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv


# TODO rename/refactor to a more generic way
class WaitNode(Node):
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
        self.action_feedback_msg.remaining_distance = convert_steps_to_distance(remaining_steps)
        self.goal_handle.publish_feedback(self.action_feedback_msg)


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.get_logger().info('navigation node started')

        self.stop_service = self.create_service(EmptySrv, 'navigation/stop', self.stop_callback)
        self.move_action_server = ActionServer(self, Move, 'navigation/move', self.move_action_callback)

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
        # TODO convert m/s to rpm and respect default speed
        stepper_msg.left.steps = convert_distance_to_steps(request.distance, request.direction)
        stepper_msg.left.speed = int(request.speed) if request.speed != -1 else 30
        stepper_msg.right.steps = convert_distance_to_steps(request.distance, request.direction)
        stepper_msg.right.speed = int(request.speed) if request.speed != -1 else 30

        self.stepper_publisher.publish(stepper_msg)

        wait_node = WaitNode(goal_handle, feedback_msg)
        while goal_handle.status == GoalStatus.STATUS_EXECUTING:
            rclpy.spin_once(wait_node)

        wait_node.destroy_node()
        result = Move.Result()
        return result


# TODO actually convert m to steps
def convert_distance_to_steps(distance, direction):
    steps = distance * 1600 * 16
    if direction == Move.Goal.DIRECTION_BACKWARD:
        steps *= -1
    return int(steps)


def convert_steps_to_distance(steps):
    return steps / 1600 / 16


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
