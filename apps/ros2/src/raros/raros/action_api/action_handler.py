import uuid

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.impl.rcutils_logger import RcutilsLogger
from unique_identifier_msgs.msg import UUID as UUIDMsg


class ActionHandler:
    def __init__(self, action_client: ActionClient, pending_goals: set, logger: RcutilsLogger):
        self.action_client = action_client
        self.pending_goals = pending_goals
        self.logger = logger
        self.goal_future = None
        self.result_future = None

    def send_goal(self, goal_msg):
        goal_id = self._generate_random_uuid()
        goal_id_hex = self._get_hex_str_from_uuid(goal_id)

        self.pending_goals.add(goal_id_hex)
        self.action_client.wait_for_server()
        self.goal_future = self.action_client.send_goal_async(goal_msg, None, goal_id)
        self.goal_future.add_done_callback(self.send_goal_done_callback)

        self.logger.info(f'created goal: {goal_id_hex}')
        return goal_id

    def send_goal_done_callback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(lambda _: self.get_result_done_callback(goal_handle.goal_id))

    def get_result_done_callback(self, goal_id):
        goal_id_hex = self._get_hex_str_from_uuid(goal_id)
        self.pending_goals.remove(goal_id_hex)
        self.logger.info(f'goal done: {goal_id_hex}')

    @staticmethod
    def _generate_random_uuid():
        return UUIDMsg(uuid=list(uuid.uuid4().bytes))

    @staticmethod
    def _get_hex_str_from_uuid(uuid_msg: UUIDMsg):
        return ''.join(f'{x:02x}' for x in uuid_msg.uuid)
