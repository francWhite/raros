import rclpy
from raros_interfaces.action import PlayTone
from raros_interfaces.msg import Distance, StepperStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty as EmptySrv


class CollisionDetection(Node):
    def __init__(self):
        super().__init__('collision_detection')
        self.get_logger().info('collision_detection node started')

        self.is_moving = False
        self.distance_subscription = self.create_subscription(Distance, 'range_sensor/distance',
                                                              self.distance_callback, 10)
        self.moving_status_subscription = self.create_subscription(StepperStatus, 'arduino_stepper/status',
                                                                   self.update_moving_status_callback, 10)
        self.stop_service_client = self.create_client(EmptySrv, 'navigation/stop')
        self.play_tone_action_client = ActionClient(self, PlayTone, 'buzzer/play_tone')

        while not self.stop_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('navigation/stop service not ready, waiting...')
        self.get_logger().info('collision_detection node ready')

        update_status_publisher = self.create_publisher(Bool, 'status/collision_detection_active', 10)
        update_status_publisher.publish(Bool(data=True))

    def distance_callback(self, msg: Distance):
        if self.is_moving and (msg.front < 20 or msg.back < 20):
            self.get_logger().info(f'obstacle detected: front={msg.front}, back={msg.back}, stopping...')
            request = EmptySrv.Request()
            self.stop_service_client.call_async(request)
            self.play_beep()

    def play_beep(self):
        goal_msg = PlayTone.Goal()
        goal_msg.frequency = 440
        goal_msg.duration = 1000
        self.play_tone_action_client.wait_for_server()
        self.play_tone_action_client.send_goal_async(goal_msg)

    def update_moving_status_callback(self, msg: StepperStatus):
        self.is_moving = msg.moving
        self.get_logger().info(f'is_moving: {self.is_moving}')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetection()

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise e
