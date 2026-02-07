import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
from collections import deque
from geometry_msgs.msg import Point


class DataControllerNode(Node):
    # Path Planner Node implementing Absolute-to-Relative Coordinate Transform.
    # Input:  Absolute polar coordinates (radius, angle) from origin
    # Output: Relative movement commands (turn angle, distance) to PID node
   
    def __init__(self):
        super().__init__('data_controller_node')
        # Delay handling
        self.pending_delay = 0.0
        self.delay_timer = None


        # =======================================================================
        # PARAMETERS
        # =======================================================================
        self.declare_parameter('target_topic_name', "/target_movement")
        self.declare_parameter('completion_topic_name', "completed_movement")
       
        target_topic = self.get_parameter('target_topic_name').get_parameter_value().string_value
        completion_topic = self.get_parameter('completion_topic_name').get_parameter_value().string_value

        # =======================================================================
        # PUBLISHERS
        # =======================================================================
        self.target_movement_publisher = self.create_publisher(Point, target_topic, 10)
        self.feedback_publisher = self.create_publisher(String, '/robot_feedback', 10)

        # =======================================================================
        # SUBSCRIBERS
        # =======================================================================
        self.create_subscription(String, '/polar_move_cmd', self.command_callback, 10)
        self.create_subscription(Point, completion_topic, self.completed_movement_callback, 10)
       
        # =======================================================================
        # STATE TRACKING - Robot's Current Absolute Position
        # =======================================================================
        self.curr_x = 0.0           # meters - current X position from origin
        self.curr_y = 0.0           # meters - current Y position from origin
        self.curr_theta = 0.0       # radians - current absolute heading
       
        # =======================================================================
        # TARGET STORAGE - For error calculation after PID completion
        # =======================================================================
        self.target_x = 0.0             # meters - target X (Cartesian)
        self.target_y = 0.0             # meters - target Y (Cartesian)
        self.target_dist_abs = 0.0      # meters - target radius (Polar)
        self.target_angle_abs = 0.0     # radians - target angle (Polar)

        # =======================================================================
        # QUEUE & STATE MACHINE
        # =======================================================================
        self.automation_queue = deque()
        self.is_moving = False

        # =======================================================================
        # STARTUP LOG
        # =======================================================================
        self.get_logger().info('================================================')
        self.get_logger().info('  DataControllerNode (Path Planner) Ready')
        self.get_logger().info('================================================')
        self.get_logger().info(f'Publish Topic: {target_topic}')
        self.get_logger().info(f'Subscribe Topic: {completion_topic}')
        self.get_logger().info(f'Initial State: ({self.curr_x:.2f}m, {self.curr_y:.2f}m, '
                               f'{math.degrees(self.curr_theta):.1f} deg)')

    # =======================================================================
    # UTILITY: Normalize angle to [-PI, PI]
    # =======================================================================
    def normalize_angle(self, angle_rad):
        # Normalize angle to range [-PI, PI] for shortest turn.
        while angle_rad > math.pi:
            angle_rad -= 2.0 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2.0 * math.pi
        return angle_rad

    # =======================================================================
    # CALLBACK: Receive commands from app
    # =======================================================================
    def command_callback(self, msg):
        # Receives all messages from the app and decides how to handle them.
        self.get_logger().info(f'Received: {msg.data}')
       
        try:
            data = json.loads(msg.data)
           
            if 'steps' in data:
                self.get_logger().info('Automation sequence detected.')
                self.start_automation_sequence(data['steps'])
            else:
                self.get_logger().info('Single command detected.')
                self.execute_single_step(data)

        except json.JSONDecodeError:
            self.get_logger().error(f"JSON parse error: {msg.data}")

    # =======================================================================
    # AUTOMATION: Load queue and start sequence
    # =======================================================================
    def start_automation_sequence(self, steps):
        # Initializes the queue and starts the first step.
        if self.is_moving:
            self.get_logger().warn("Robot is busy. Cannot start new sequence.")
            return

        self.automation_queue.extend(steps)
        self.get_logger().info(f'Loaded {len(steps)} waypoints.')
        self.process_next_step()

    # =======================================================================
    # QUEUE PROCESSOR: Pop next step and execute
    # =======================================================================
    def process_next_step(self):
        # Pops next step from queue and passes to execute_single_step.
        if not self.automation_queue:
            self.is_moving = False
            self.get_logger().info('================================================')
            self.get_logger().info('  SEQUENCE COMPLETE')
            self.get_logger().info('================================================')
            return

        next_step = self.automation_queue.popleft()
        self.get_logger().info(f'Processing waypoint ({len(self.automation_queue)} remaining)')
        self.execute_single_step(next_step)

    # =======================================================================
    # CORE: Absolute-to-Relative Transform & Publish
    # =======================================================================
    def execute_single_step(self, command_data):
        # THE CORE PATH PLANNING FUNCTION
        # Converts absolute polar target to relative movement command.
        # Input: command_data with 'radius' (cm) and 'angle' (deg) - ABSOLUTE from origin
        # Output: Publishes relative (distance, turn) to PID node
       
        self.is_moving = True
       
        # -----------------------------------------------------------------------
        # EXTRACT INPUT - Absolute Polar Target (from app)
        # -----------------------------------------------------------------------
        try:
            radius_cm = float(command_data.get('radius', 0.0))
            angle_deg = float(command_data.get('angle', 0.0))
            delay_sec = float(command_data.get('delaySeconds', 0.0))
            self.pending_delay = max(0.0, delay_sec)

        except ValueError as e:
            self.get_logger().error(f"Invalid input: {e}")
            self.is_moving = False
            return

        # -----------------------------------------------------------------------
        # STEP F: Store absolute target for error calculation later
        # -----------------------------------------------------------------------
        self.target_dist_abs = radius_cm / 100.0          # cm -> meters
        self.target_angle_abs = math.radians(angle_deg)   # deg -> radians
       
        self.get_logger().info(f'------------------------------------------------')
        self.get_logger().info(f'TARGET (Absolute Polar): {radius_cm:.1f}cm @ {angle_deg:.1f} deg')
       
        # -----------------------------------------------------------------------
        # STEP A: Convert Absolute Polar -> Absolute Cartesian
        # -----------------------------------------------------------------------
        self.target_x = self.target_dist_abs * math.cos(self.target_angle_abs)
        self.target_y = self.target_dist_abs * math.sin(self.target_angle_abs)
       
        self.get_logger().info(f'TARGET (Cartesian): ({self.target_x:.3f}m, {self.target_y:.3f}m)')
        self.get_logger().info(f'CURRENT State: ({self.curr_x:.3f}m, {self.curr_y:.3f}m, '
                               f'{math.degrees(self.curr_theta):.1f} deg)')
       
        # -----------------------------------------------------------------------
        # STEP B: Calculate Vector from Current Position to Target
        # -----------------------------------------------------------------------
        dx = self.target_x - self.curr_x
        dy = self.target_y - self.curr_y
       
        # -----------------------------------------------------------------------
        # STEP C: Calculate Relative Command
        # -----------------------------------------------------------------------
        rel_dist = math.sqrt(dx * dx + dy * dy)
       
        # Handle edge case: already at target
        if rel_dist < 0.001:
            target_heading = self.curr_theta  # No movement needed
        else:
            target_heading = math.atan2(dy, dx)
       
        rel_angle = target_heading - self.curr_theta
       
        # -----------------------------------------------------------------------
        # STEP D: Normalize relative angle to [-PI, PI]
        # -----------------------------------------------------------------------
        rel_angle = self.normalize_angle(rel_angle)
       
        self.get_logger().info(f'RELATIVE Command Calculated:')
        self.get_logger().info(f'  dx={dx:.3f}m, dy={dy:.3f}m')
        self.get_logger().info(f'  rel_dist={rel_dist:.3f}m ({rel_dist*100:.1f}cm)')
        self.get_logger().info(f'  target_heading={math.degrees(target_heading):.1f} deg')
        self.get_logger().info(f'  rel_angle={math.degrees(rel_angle):.1f} deg')
       
        # -----------------------------------------------------------------------
        # STEP E: Publish to PID Node
        # Point.x = relative distance (meters)
        # Point.y = relative turn angle (radians)
        # -----------------------------------------------------------------------
        goal_msg = Point()
        goal_msg.x = rel_dist       # meters
        goal_msg.y = rel_angle      # radians
        goal_msg.z = 0.0
       
        self.target_movement_publisher.publish(goal_msg)
        self.get_logger().info(f'PUBLISHED to PID: dist={rel_dist:.3f}m, turn={math.degrees(rel_angle):.1f} deg')

    def delay_timer_callback(self):
    # One-shot timer: cancel after firing
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer = None

        self.pending_delay = 0.0
        self.get_logger().info('Delay complete. Proceeding to next step.')
        self.process_next_step()


    # =======================================================================
    # FEEDBACK: Receive completion, update state, calculate error
    # =======================================================================
    def completed_movement_callback(self, msg):
        # Receives achieved RELATIVE movement from PID node.
        # 1. Updates absolute state based on relative movement
        # 2. Calculates error between absolute target and new position
        # 3. Publishes absolute feedback to app
        # 4. Advances to next waypoint if queue not empty
       
        if not self.is_moving:
            self.get_logger().warn("Unexpected completion message. Ignoring.")
            return

        # -----------------------------------------------------------------------
        # EXTRACT: Achieved relative movement from PID
        # -----------------------------------------------------------------------
        achieved_rel_dist = msg.x       # meters
        achieved_rel_angle = msg.y      # radians
       
        self.get_logger().info(f'------------------------------------------------')
        self.get_logger().info(f'PID FEEDBACK (Relative):')
        self.get_logger().info(f'  achieved_dist={achieved_rel_dist:.3f}m')
        self.get_logger().info(f'  achieved_turn={math.degrees(achieved_rel_angle):.1f} deg')
       
        # -----------------------------------------------------------------------
        # UPDATE STATE: Apply relative movement to absolute state
        # Order: First turn, then move in new direction
        # -----------------------------------------------------------------------
        # Update heading first
        self.curr_theta += achieved_rel_angle
        self.curr_theta = self.normalize_angle(self.curr_theta)
       
        # Then update position (move in direction of new heading)
        self.curr_x += achieved_rel_dist * math.cos(self.curr_theta)
        self.curr_y += achieved_rel_dist * math.sin(self.curr_theta)
       
        self.get_logger().info(f'STATE UPDATED:')
        self.get_logger().info(f'  New Position: ({self.curr_x:.3f}m, {self.curr_y:.3f}m)')
        self.get_logger().info(f'  New Heading: {math.degrees(self.curr_theta):.1f} deg')
       
        # -----------------------------------------------------------------------
        # CALCULATE: New absolute position in polar (for app feedback)
        # -----------------------------------------------------------------------
        moved_radius_abs = math.sqrt(self.curr_x ** 2 + self.curr_y ** 2)
        moved_angle_abs_rad = math.atan2(self.curr_y, self.curr_x)
        moved_angle_abs_deg = math.degrees(moved_angle_abs_rad)
       
        # -----------------------------------------------------------------------
        # ERROR: Calculate between absolute target and actual position
        # -----------------------------------------------------------------------
        error_x = self.target_x - self.curr_x
        error_y = self.target_y - self.curr_y
        error_dist = math.sqrt(error_x ** 2 + error_y ** 2)
       
        self.get_logger().info(f'ERROR CALCULATION:')
        self.get_logger().info(f'  Target: ({self.target_x:.3f}m, {self.target_y:.3f}m)')
        self.get_logger().info(f'  Actual: ({self.curr_x:.3f}m, {self.curr_y:.3f}m)')
        self.get_logger().info(f'  Error Vector: {error_dist*100:.2f}cm')
       
        # -----------------------------------------------------------------------
        # PUBLISH: Absolute feedback to app
        # -----------------------------------------------------------------------
        self.publish_feedback(moved_radius_abs, moved_angle_abs_deg, error_dist)
       
        # -----------------------------------------------------------------------
        # ADVANCE: Process next waypoint or complete
        # -----------------------------------------------------------------------
        if self.automation_queue:
            if self.pending_delay > 0.0:
                self.get_logger().info(f'Waiting {self.pending_delay:.2f}s before next step...')
                self.delay_timer = self.create_timer(self.pending_delay, self.delay_timer_callback)
            else:
                self.process_next_step()
        else:
            self.is_moving = False
            self.get_logger().info('------------------------------------------------')
            self.get_logger().info('Sequence complete. State: IDLE')
            self.get_logger().info(f'Final Position: ({self.curr_x:.3f}m, {self.curr_y:.3f}m)')
            self.get_logger().info(f'Final Heading: {math.degrees(self.curr_theta):.1f} deg')

    # =======================================================================
    # PUBLISH: Absolute feedback to app
    # =======================================================================
    def publish_feedback(self, moved_radius_m, moved_angle_deg, error_dist_m):
       # Robot position angle w.r.t origin
        pos_angle_deg = math.degrees(math.atan2(self.curr_y, self.curr_x))
        if pos_angle_deg < 0:
            pos_angle_deg += 360.0

        feedback_dict = {"moved_radius": round(moved_radius_m * 100, 2),   # cm (distance from origin)
        "moved_angle": round(pos_angle_deg, 2),           # deg (position angle, 0–360)
        "error_vector": round(error_dist_m * 100, 2),     # cm
        }

        feedback_msg = String()
        feedback_msg.data = json.dumps(feedback_dict)

        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info(f'FEEDBACK to App: {feedback_msg.data}')



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