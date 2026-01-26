# DataControllerNode

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import deque
from geometry_msgs.msg import Point

class DataControllerNode(Node):
    """
    This node handles commands from the app, publishes goals to the PID node,
    and uses a queue-based state machine to manage sequential automation steps,
    waiting for feedback after each one. It implements error compensation by
    adding accumulated error to the subsequent target command.
    """
    def __init__(self):
        super().__init__('data_controller_node')

       
        # Declare parameter for the Publisher topic name
        self.declare_parameter('target_topic_name', "/target_movement")
        # Declare parameter for the Subscriber topic name
        self.declare_parameter('completion_topic_name', "completed_movement")
       
        # Retrieve parameter values
        target_topic = self.get_parameter('target_topic_name').get_parameter_value().string_value
        completion_topic = self.get_parameter('completion_topic_name').get_parameter_value().string_value

        # --- Publishers ---
        # 1. Publishes the polar goal command to the Distance PID node
        # Topic name is now read from the parameter
        self.target_movement_publisher = self.create_publisher(Point, target_topic, 10)
       
        # 2. Publishes real-time feedback (JSON String) back to the app
        self.feedback_publisher = self.create_publisher(String, '/robot_feedback', 10)

        # --- Subscribers ---
        # 1. Subscribes to commands (JSON String) from the app
        self.create_subscription(String, '/polar_move_cmd', self.command_callback, 10)
       
        # 2. Subscribes to achieved movement feedback (Point) from the PID node
        # Topic name is now read from the parameter
        self.create_subscription(Point, completion_topic, self.completed_movement_callback, 10)
       
        # --- Data Persistence & State Management ---
       
        self.target_distance = 0.0      # Last commanded distance (meters)
        self.target_angle_rad = 0.0     # Last commanded angle (radians)

        # Queue and state flag for sequence management
        self.automation_queue = deque()
        self.is_moving = False          

        self.get_logger().info('Data Controller Node is ready. State: IDLE.')
        self.get_logger().info(f'Target Publisher Topic: {target_topic}')
        self.get_logger().info(f'Completion Subscriber Topic: {completion_topic}')

    def command_callback(self, msg):
        """Receives all messages from the app and decides how to handle them."""
        self.get_logger().info(f'Received message: {msg.data}')
       
        try:
            data = json.loads(msg.data)
            if 'steps' in data:
                self.get_logger().info('Automation file detected. Starting sequence...')
                self.start_automation_sequence(data['steps'])
            else:
                self.get_logger().info('Single manual command detected.')
                self.execute_single_step(data)

        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse incoming JSON: {msg.data}")

    def start_automation_sequence(self, steps):
        """Initializes the queue and starts the first step."""
        if self.is_moving:
            self.get_logger().warn("Robot is busy. Cannot start new sequence.")
            return

        # Load all steps into the queue
        self.automation_queue.extend(steps)
        self.process_next_step() # Start the first command

    def process_next_step(self):
        """
        Sends the next command from the queue.
        CRITICAL: Applies accumulated error to the new target before publishing.
        """
        if not self.automation_queue:
            self.is_moving = False
            self.get_logger().info('Automation sequence complete.')
            return

        next_step = self.automation_queue.popleft()

        # 1. Extract target WITHOUT compensation
        original_radius = float(next_step.get('radius', 0.0))
        original_angle_deg = float(next_step.get('angle', 0.0))

        self.get_logger().info(
        f"Executing step WITHOUT compensation: "
        f"{original_radius:.2f}cm, {original_angle_deg:.2f}deg"
        )

        next_step['radius'] = original_radius
        next_step['angle'] = original_angle_deg


        # 3. Execute with modified step
       
        self.execute_single_step(next_step)
       
    def execute_single_step(self, command_data):
        self.is_moving = True
        """
        Stores the target, publishes the goal message. This function now receives
        compensated values (if part of an automation sequence).
        """
        try:
            # Note: radius and angle are already compensated if coming from process_next_step
            radius = float(command_data.get('radius', 0.0))
            angle_deg = float(command_data.get('angle', 0.0))
        except ValueError as e:
            self.get_logger().error(f"Invalid numeric input for radius or angle: {e}")
            return

        angle_rad = angle_deg * (3.14159 / 180.0)
       
        # Store target values
        self.target_distance = radius/100.0
        self.target_angle_rad = angle_rad
       
        self.get_logger().info(f"Target Stored: Dist={self.target_distance:.2f}m, Angle={self.target_angle_rad:.2f}rad")

        # 1. Create and populate the ROS Point message
        goal_msg = Point()
        goal_msg.x = self.target_distance
        goal_msg.y = self.target_angle_rad
        goal_msg.z = 0.0

        # 2. Publish the goal to the PID node
        self.target_movement_publisher.publish(goal_msg)
        self.get_logger().info(f"Published Goal to PID: {goal_msg.x:.2f}m, {goal_msg.y:.2f}rad")
       
    def completed_movement_callback(self, msg):
        """
        Receives achieved movement from PID, calculates errors, accumulates them,
        publishes feedback, and advances the sequence.
        """
        if not self.is_moving:
            self.get_logger().warn("Received unexpected completion message. Robot was not moving.")
            return

        # 1. Data Extraction and Conversion
        achieved_distance = msg.x
        achieved_angle_rad = msg.y
        achieved_angle_deg = achieved_angle_rad * (180.0 / 3.14159)
       
        # 2. Error Calculation
        distance_error = self.target_distance - achieved_distance
        angle_error_rad = self.target_angle_rad - achieved_angle_rad
        angle_error_deg = angle_error_rad * (180.0 / 3.14159)
       
        # 3. Accumulate Error (CRITICAL STEP for compensation)
        self.get_logger().info(f"Errors (Instantaneous Only): "f"Dist={distance_error:.2f}m, Angle={angle_error_deg:.2f}deg")

        # 4. Publish Feedback to App (uses instantaneous error)
        self.publish_feedback(achieved_angle_deg, achieved_distance, distance_error, angle_error_deg)

        # 5. Advance State Machine
        if self.automation_queue:
            self.get_logger().info("Previous step complete. Dispatching next step from queue.")
            self.process_next_step()
        else:
            self.is_moving = False
            self.get_logger().info("Sequence complete. Robot is idle.")

    def publish_feedback(self, achieved_angle, achieved_distance, distance_error, angle_error):
        """
        Constructs the JSON string with achieved and error values, and publishes it back to the app.
        """
        # Dictionary structure compatible with app (3 keys)
        feedback_dict = {
            'moved_radius': achieved_distance*100,
            'moved_angle': achieved_angle,
            'error_vector': distance_error*100,
        }
       
        feedback_msg = String()
        feedback_msg.data = json.dumps(feedback_dict)
       
        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info(f'Published feedback: {feedback_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DataControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
