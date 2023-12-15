import rclpy
from raros_interfaces.msg import Status as StatusMsg, StepperStatus
from raros_interfaces.srv import GetStatus
from rclpy.node import Node
from std_msgs.msg import Bool


class Status(Node):
    def __init__(self):
        super().__init__('status')
        self.get_logger().info('status node started')

        self.status: StatusMsg = StatusMsg()
        self.status.is_available = True
        self.status_service = self.create_service(GetStatus, 'status/get', self.get_status_callback)

        self.moving_subscription = self.create_subscription(StepperStatus, 'arduino_stepper/status',
                                                            self.update_is_moving_callback, 10)
        self.playing_tone_subscription = self.create_subscription(Bool, 'status/playing_tone',
                                                                  self.update_is_playing_tone_callback, 10)
        self.magnet_active_subscription = self.create_subscription(Bool, 'status/magnet_active',
                                                                   self.update_is_magnet_active_callback, 10)
        self.collision_detection_subscription = self.create_subscription(Bool, 'status/collision_detection_active',
                                                                         self.update_is_collision_detection_active_callback,
                                                                         10)

    def get_status_callback(self, request, response):
        response.status = self.status
        return response

    def update_is_moving_callback(self, msg: StepperStatus):
        self.status.is_moving = msg.moving

    def update_is_playing_tone_callback(self, msg: Bool):
        self.status.is_playing_tone = msg.data

    def update_is_magnet_active_callback(self, msg: Bool):
        self.status.is_magnet_active = msg.data

    def update_is_collision_detection_active_callback(self, msg: Bool):
        self.status.is_collision_detection_active = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = Status()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
