import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AppSubscriberNode(Node):
    def __init__(self):
        super().__init__('app_subscriber_node')
        self.subscription = self.create_subscription(
            String,
            '/polar_move_cmd',
            self.listener_callback,
            10)
        self.get_logger().info('App subscriber node has started. Listening on /app_commands...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from app: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = AppSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()