import rclpy
from raros.action_api.action_handler import ActionHandler
from raros_interfaces.action import PlayTone, Move, Rotate, Turn
from raros_interfaces.srv import ActionCompleted, ActionPlayTone, ActionMove, ActionRotate, ActionTurn
from rclpy.action import ActionClient
from rclpy.node import Node


class ActionApi(Node):
    """
        Node to access action servers. This is a workaround for the lack of ROS2 action client
        support in the roslibjs library (will be fixed with PR #645).
    """

    def __init__(self):
        super().__init__('action_api')
        self.get_logger().info('action_api node started')
        self.pending_goals = set()

        self.action_completed_service = self.create_service(ActionCompleted, 'action_api/action_completed',
                                                            self.action_completed_callback)

        play_tone_action_client = ActionClient(self, PlayTone, 'buzzer/play_tone')
        self.play_tone_action_handler = ActionHandler(play_tone_action_client, self.pending_goals, self.get_logger())
        self.play_tone_service = self.create_service(ActionPlayTone, 'action_api/buzzer/play_tone',
                                                     self.play_tone_callback)

        move_action_client = ActionClient(self, Move, 'navigation/move')
        self.move_action_handler = ActionHandler(move_action_client, self.pending_goals, self.get_logger())
        self.move_service = self.create_service(ActionMove, 'action_api/navigation/move',
                                                self.move_callback)

        rotate_action_client = ActionClient(self, Rotate, 'navigation/rotate')
        self.rotate_action_handler = ActionHandler(rotate_action_client, self.pending_goals, self.get_logger())
        self.rotate_service = self.create_service(ActionRotate, 'action_api/navigation/rotate',
                                                  self.rotate_callback)

        turn_action_client = ActionClient(self, Turn, 'navigation/turn')
        self.turn_action_handler = ActionHandler(turn_action_client, self.pending_goals, self.get_logger())
        self.turn_service = self.create_service(ActionTurn, 'action_api/navigation/turn',
                                                self.turn_callback)

    def action_completed_callback(self, request, response):
        goal_id = request.goal_id.replace('-', '')
        response.completed = goal_id not in self.pending_goals
        return response

    def play_tone_callback(self, request, response):
        self.get_logger().info(f'play_tone_callback received: "{request}"')
        goal_msg = PlayTone.Goal()
        goal_msg.frequency = request.frequency
        goal_msg.duration = request.duration
        goal_id = self.play_tone_action_handler.send_goal(goal_msg)

        response.goal_id = goal_id
        return response

    def move_callback(self, request, response):
        self.get_logger().info(f'move_callback received: "{request}"')
        goal_msg = Move.Goal()
        goal_msg.speed = request.speed
        goal_msg.distance = request.distance
        goal_msg.direction = request.direction
        goal_id = self.move_action_handler.send_goal(goal_msg)

        response.goal_id = goal_id
        return response

    def rotate_callback(self, request, response):
        self.get_logger().info(f'rotate_callback received: "{request}"')
        goal_msg = Rotate.Goal()
        goal_msg.angle = request.angle
        goal_msg.direction = request.direction
        goal_id = self.rotate_action_handler.send_goal(goal_msg)

        response.goal_id = goal_id
        return response

    def turn_callback(self, request, response):
        self.get_logger().info(f'turn_callback received: "{request}"')
        goal_msg = Turn.Goal()
        goal_msg.angle = request.angle
        goal_msg.radius = request.radius
        goal_msg.direction = request.direction
        goal_id = self.turn_action_handler.send_goal(goal_msg)

        response.goal_id = goal_id
        return response


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
