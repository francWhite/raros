import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from raros_interfaces.action import PlayTone
from raros_interfaces.srv import ActionPlayTone
from unique_identifier_msgs.msg import UUID


class ActionApi(Node):
    """
        Node to access action servers. This is a workaround for the lack of ROS2 action client
        support in the roslibjs library (will be fixed with PR #645).
    """

    def __init__(self):
        super().__init__('action_api')
        self.get_logger().info('action_api node started')
        self.play_tone_action_client = ActionClient(self, PlayTone, 'buzzer/play_tone')
        self.play_tone_service = self.create_service(ActionPlayTone, 'action_api/buzzer/play_tone',
                                                     self.action_play_tone_callback)

    def action_play_tone_callback(self, request, response):
        self.get_logger().info(f'action_play_tone_callback received: "{request}"')

        goal_msg = PlayTone.Goal()
        goal_msg.frequency = request.frequency
        goal_msg.duration = request.duration

        goal_id, goal_id_hex = self._generate_random_uuid()
        self.play_tone_action_client.wait_for_server()
        self.play_tone_action_client.send_goal_async(goal_msg, None, goal_id)
        response.goal_id = goal_id

        self.get_logger().info(f'created goal: {goal_id_hex}')
        return response

    @staticmethod
    def _generate_random_uuid():
        new_uuid = uuid.uuid4()
        return UUID(uuid=list(new_uuid.bytes)), new_uuid.hex


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
