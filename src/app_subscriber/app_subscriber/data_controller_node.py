import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random

class DataControllerNode(Node):
    """
    This node handles both single manual commands and full automation files
    sent from the app. It provides REAL-TIME feedback after each step.
    """
    def __init__(self):
        super().__init__('data_controller_node')

        # Subscribes to commands from the app
        self.create_subscription(String, '/polar_move_cmd', self.command_callback, 10)
        
        # Publishes real-time feedback back to the app
        self.feedback_publisher_ = self.create_publisher(String, '/robot_feedback', 10)

        self.get_logger().info('Data Controller Node is ready.')

    def command_callback(self, msg):
        """Receives all messages from the app and decides how to handle them."""
        self.get_logger().info(f'Received message: {msg.data}')
        
        try:
            # First, try to parse it as a full automation file
            data = json.loads(msg.data)
            if 'steps' in data:
                self.get_logger().info('Automation file detected. Starting sequence...')
                self.execute_automation_sequence(data['steps'])
            else:
                # If it's not a file, treat it as a single manual command
                self.get_logger().info('Single manual command detected.')
                self.execute_single_step(data)

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse incoming JSON: {msg.data}")

    def execute_automation_sequence(self, steps):
        """Loops through steps from the automation file."""
        for step in steps:
            self.execute_single_step(step)
            # Handle the delay specified in the step
            delay = float(step.get('delaySeconds', 0))
            if delay > 0:
                self.get_logger().info(f"Waiting for {delay} seconds...")
                time.sleep(delay)
        self.get_logger().info('Automation sequence complete.')

    def execute_single_step(self, command_data):
        """Executes a single movement command and publishes feedback."""
        radius = command_data.get('radius')
        angle = command_data.get('angle')
        
        self.get_logger().info(f"Executing move: Radius={radius}, Angle={angle}")
        
        # --- ECE TEAM: ADD YOUR ROBOT MOVEMENT LOGIC HERE ---
        # This is where the robot would actually be told to move.
        # For now, we just wait for a moment to simulate movement.
        time.sleep(1) 
        # ----------------------------------------------------

        # After the move is complete, generate and publish feedback for THIS step
        self.publish_feedback(command_data)

    def publish_feedback(self, original_command):
        """Generates mock feedback for a single step and publishes it."""
        feedback_dict = {
            'moved_radius': (float(original_command.get('radius', 0.0)) or 0.0) * random.uniform(5, 10),
            'moved_angle': (float(original_command.get('angle', 0.0)) or 0.0) * random.uniform(5, 10),
            'error_vector': random.uniform(0.0, 1.5)
        }
        
        # Create a String message and set its data to the JSON feedback
        feedback_msg = String()
        feedback_msg.data = json.dumps(feedback_dict)
        
        self.feedback_publisher_.publish(feedback_msg)
        self.get_logger().info(f'Published feedback: {feedback_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = DataControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()