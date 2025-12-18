import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random  # To generate random numbers
import json    # To create the JSON string

class FeedbackPublisherStringNode(Node):
    """
    This node publishes a JSON string containing random feedback data
    to the /robot_feedback topic every second.
    """
    def __init__(self):
        super().__init__('feedback_publisher_string_node')
        
        # Create a publisher for the standard String message type
        self.publisher_ = self.create_publisher(String, '/robot_feedback', 100)
        
        # Create a timer that will call the timer_callback function once per second
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('String feedback publisher has started. Publishing random data... 🎲')

    def timer_callback(self):
        # 1. Create a Python dictionary with the data
        data_dict = {
            'moved_radius': random.uniform(45.0, 55.0),
            'moved_angle': random.uniform(80.0, 100.0),
            'error_vector': random.uniform(0.0, 1.5)
        }
        
        # 2. Create a String message instance
        msg = String()
        
        # 3. Convert the dictionary to a JSON-formatted string and assign it to the 'data' field
        msg.data = json.dumps(data_dict)
        
        # 4. Publish the message
        self.publisher_.publish(msg)
        
        # 5. Log the published data to the console for debugging
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackPublisherStringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    