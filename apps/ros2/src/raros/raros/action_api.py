import uuid

import rclpy
from raros_interfaces.action import Move
from raros_interfaces.action import PlayTone
from raros_interfaces.srv import ActionCompleted
from raros_interfaces.srv import ActionMove as MoveActionSrv
from raros_interfaces.srv import ActionPlayTone as PlayToneActionSrv
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID as UUIDMsg


class ActionApi(Node):
    """
        Node to access action servers. This is a workaround for the lack of ROS2 action client
        support in the roslibjs library (will be fixed with PR #645).
    """

    def __init__(self):
        super().__init__('action_api')
        self.get_logger().info('action_api node started')
        self.pending_goals = set()
        self.play_tone_action_client = ActionClient(self, PlayTone, 'buzzer/play_tone')
        self.move_action_client = ActionClient(self, Move, 'navigation/move')

        self.play_tone_service = self.create_service(PlayToneActionSrv, 'action_api/buzzer/play_tone',
                                                     self.action_play_tone_callback)
        self.move_service = self.create_service(MoveActionSrv, 'action_api/navigation/move',
                                                self.action_move_callback)
        self.action_completed_service = self.create_service(ActionCompleted, 'action_api/action_completed',
                                                            self.action_completed_callback)
        self._play_tone_send_goal_future = None
        self._play_tone_get_result_future = None
        self._move_send_goal_future = None
        self._move_get_result_future = None

    def action_play_tone_callback(self, request, response):
        self.get_logger().info(f'action_play_tone_callback received: "{request}"')

        goal_msg = PlayTone.Goal()
        goal_msg.frequency = request.frequency
        goal_msg.duration = request.duration

        goal_id = self._generate_random_uuid()
        goal_id_hex = self._get_hex_str_from_uuid(goal_id)

        self.pending_goals.add(goal_id_hex)
        self.play_tone_action_client.wait_for_server()
        self._play_tone_send_goal_future = self.play_tone_action_client.send_goal_async(goal_msg, None, goal_id)
        self._play_tone_send_goal_future.add_done_callback(self.play_tone_response_callback)

        self.get_logger().info(f'created goal: {goal_id_hex}')
        response.goal_id = goal_id
        return response

    def play_tone_response_callback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        self._play_tone_get_result_future = goal_handle.get_result_async()
        self._play_tone_get_result_future.add_done_callback(lambda _: self.play_tone_done_callback(goal_handle.goal_id))

    def play_tone_done_callback(self, goal_id):
        goal_id_hex = self._get_hex_str_from_uuid(goal_id)
        self.pending_goals.remove(goal_id_hex)
        self.get_logger().info(f'goal done: {goal_id_hex}')

    # TODO refactor: move duplicated code to a common class
    def action_move_callback(self, request, response):
        self.get_logger().info(f'action_move_callback received: "{request}"')

        goal_msg = Move.Goal()
        goal_msg.speed = request.speed
        goal_msg.distance = request.distance
        goal_msg.direction = request.direction

        goal_id = self._generate_random_uuid()
        goal_id_hex = self._get_hex_str_from_uuid(goal_id)

        self.pending_goals.add(goal_id_hex)
        self.move_action_client.wait_for_server()
        self._move_send_goal_future = self.move_action_client.send_goal_async(goal_msg, None, goal_id)
        self._move_send_goal_future.add_done_callback(self.move_response_callback)

        self.get_logger().info(f'created goal: {goal_id_hex}')
        response.goal_id = goal_id
        return response

    def move_response_callback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        self._move_get_result_future = goal_handle.get_result_async()
        self._move_get_result_future.add_done_callback(lambda _: self.move_done_callback(goal_handle.goal_id))

    def move_done_callback(self, goal_id):
        goal_id_hex = self._get_hex_str_from_uuid(goal_id)
        self.pending_goals.remove(goal_id_hex)
        self.get_logger().info(f'goal done: {goal_id_hex}')

    def action_completed_callback(self, request, response):
        goal_id = request.goal_id.replace('-', '')
        response.completed = goal_id not in self.pending_goals
        return response

    @staticmethod
    def _generate_random_uuid():
        return UUIDMsg(uuid=list(uuid.uuid4().bytes))

    @staticmethod
    def _get_hex_str_from_uuid(uuid_msg: UUIDMsg):
        return ''.join(f'{x:02x}' for x in uuid_msg.uuid)


def main(args=None):
    rclpy.init(args=args)
    node = ActionApi()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
