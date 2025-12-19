import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion # Standard tool for Q-to-Yaw conversion

# --- PID Controller Class for Angular/Linear Control ---
class PIDController:
    """A general PID controller class, incorporating anti-windup and deadband."""
    
    # Added min_out (deadband) and logger
    def __init__(self, kp, ki, kd, max_out, min_out, dt, logger): 
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.max_output = max_out
        self.min_output = min_out  # Minimum absolute velocity limit (deadband)
        self.dt = dt 
        self.logger = logger       # Logger instance for debugging
        
        self.integral = 0.0
        self.last_error = 0.0

    def calculate(self, error):
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * self.dt
        
        # Anti-windup check (saturate integral to prevent massive buildup)
        if self.Ki != 0:
            if self.integral * self.Ki > self.max_output:
                self.integral = self.max_output / self.Ki
            elif self.integral * self.Ki < -self.max_output:
                self.integral = -self.max_output / self.Ki
            
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (error - self.last_error) / self.dt if self.dt > 0 else 0.0
        
        # Total output
        output = P + I + D
        
        # --- Saturation and Deadband (Min/Max limits) ---
        
        # 1. Clamp output to Maximum allowed value
        if output > self.max_output:
            output = self.max_output
        elif output < -self.max_output:
            output = -self.max_output
            
        # 2. Apply Deadband (Minimum velocity)
        # If the output is within the deadband, but non-zero, push it to the min_output
        # This overcomes static friction (stiction).
        if abs(output) > 0.0 and abs(output) < self.min_output:
            output = math.copysign(self.min_output, output)
            
        self.last_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0


# --- Distance and Rotation PID Node ---
class DistancePIDController(Node):
    
    # Define the possible states for the sequential movement
    STATE_IDLE = 0
    STATE_ROTATING = 1
    STATE_MOVING = 2
    STATE_FINISHED = 3

    def __init__(self):
        super().__init__('distance_pid_controller')

        # --- Parameter Declaration ---
        self.declare_parameter('control_rate', 50)
        
        # Topic Names
        self.declare_parameter('target_topic', "/target_movement")
        self.declare_parameter('odom_topic', "/odometry/filtered")
        self.declare_parameter('cmd_vel_topic', "/cmd_velocity")
        self.declare_parameter('completion_topic', "completed_movement")

        # Linear PID Gains
        self.declare_parameter('kp_lin', 0.5)
        self.declare_parameter('ki_lin', 0.01)
        self.declare_parameter('kd_lin', 0.01)
        
        # Angular PID Gains
        self.declare_parameter('kp_rot', 0.05)
        self.declare_parameter('ki_rot', 0.0) 
        self.declare_parameter('kd_rot', 0.02) 

        # Velocity Limits
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('min_angular_vel', 0.05)
        self.declare_parameter('min_linear_vel', 0.05)


        # --- Configuration Retrieval ---
        self.rate = self.get_parameter('control_rate').get_parameter_value().integer_value
        self.dt = 1.0 / self.rate
        
        # Retrieve Topic Names
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        completion_topic = self.get_parameter('completion_topic').get_parameter_value().string_value

        # Retrieve PID Gains
        KP_LIN = self.get_parameter('kp_lin').get_parameter_value().double_value
        KI_LIN = self.get_parameter('ki_lin').get_parameter_value().double_value
        KD_LIN = self.get_parameter('kd_lin').get_parameter_value().double_value
        
        KP_ROT = self.get_parameter('kp_rot').get_parameter_value().double_value
        KI_ROT = self.get_parameter('ki_rot').get_parameter_value().double_value
        KD_ROT = self.get_parameter('kd_rot').get_parameter_value().double_value

        # Retrieve Velocity Limits
        MAX_ANGULAR_VEL = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        MAX_LINEAR_VEL = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        MIN_ANGULAR_VEL = self.get_parameter('min_angular_vel').get_parameter_value().double_value
        MIN_LINEAR_VEL = self.get_parameter('min_linear_vel').get_parameter_value().double_value


        # --- Publishers and Subscribers ---
        self.create_subscription(Point, target_topic, self.target_callback, 10)
        
        # Subscribes to filtered odometry 
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Publishes movement completion back to the app_subscriber
        self.completion_publisher = self.create_publisher(Point, completion_topic, 10)
        
        # --- Control State ---
        self.current_state = self.STATE_IDLE
        
        # --- Target and Feedback Storage ---
        self.target_dist = 0.0
        self.target_angle_rad = 0.0
        self.current_yaw = 0.0
        self.current_odom_x = 0.0 
        self.current_odom_y = 0.0
        self.current_dist_travelled = 0.0
        
        # Reference points for relative motion
        self.start_yaw = 0.0
        self.start_pos_x = 0.0
        self.start_pos_y = 0.0
        
        # Direction of movement (1.0 for forward, -1.0 for backward). Used to ensure 
        # the final linear command direction is correct, irrespective of PID output sign.
        self.movement_direction = 1.0 

        # --- Controllers (Initialized with Parameters) ---
        self.pid_rot = PIDController(KP_ROT, KI_ROT, KD_ROT, 
                                     MAX_ANGULAR_VEL, MIN_ANGULAR_VEL, 
                                     self.dt, self.get_logger())
                                     
        self.pid_dist = PIDController(KP_LIN, KI_LIN, KD_LIN, 
                                      MAX_LINEAR_VEL, MIN_LINEAR_VEL, 
                                      self.dt, self.get_logger())

        # --- Main Control Loop Timer ---
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Distance PID Controller Node Initialized with parameters.')

    # --- CALLBACKS ---

    def target_callback(self, msg):
        """Receives the new polar target from the app_subscriber."""
        if self.current_state != self.STATE_IDLE and self.current_state != self.STATE_FINISHED:
            self.get_logger().warn("New target received while busy. Ignoring.")
            return

        # Target distance is read as absolute, and direction is stored separately.
        self.target_dist = abs(msg.x) 
        self.skip_linear = self.target_dist < 0.01
        self.target_angle_rad = msg.y
        #self.target_world_yaw = math.atan2(math.sin(self.target_world_yaw),math.cos(self.target_world_yaw))
        # Determine movement direction based on target distance sign (used in control_loop)
        self.movement_direction = math.copysign(1.0, msg.x)
        
        # Reset position/yaw references to the current filtered state (relative movement)
        self.start_yaw = self.current_yaw
        self.start_pos_x = self.current_odom_x
        self.start_pos_y = self.current_odom_y
        self.current_dist_travelled = 0.0
        
        # Reset PID controllers
        self.pid_rot.reset()
        self.pid_dist.reset()

        # Start the sequence with rotation
        self.current_state = self.STATE_ROTATING
        self.get_logger().info(f"Target set: Turn {self.target_angle_rad:.2f} rad, Move {self.target_dist:.2f} m.")


    def odom_callback(self, msg):
        """Receives odometry feedback and updates current state."""
        # 1. Update position (for distance tracking)
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
        # Calculate distance travelled from the start position
        dx = self.current_odom_x - self.start_pos_x
        dy = self.current_odom_y - self.start_pos_y
        self.current_dist_travelled = math.sqrt(dx*dx + dy*dy)
        
        # 2. Update Yaw (CRITICAL: Quaternion to Yaw Conversion)
        orientation_q = msg.pose.pose.orientation
        
        # euler_from_quaternion returns roll, pitch, yaw
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, 
                                                    orientation_q.z, orientation_q.w])
         
        self.current_yaw = yaw


    # --- CONTROL LOOP ---

    def control_loop(self):
        """Main loop runs at the declared rate (e.g., 50Hz)."""
        twist_msg = Twist()

        if self.current_state == self.STATE_ROTATING:
            # Stage 1: Rotation Control
            
            # Calculate angle needed to reach the target yaw
            angle_error = self.target_angle_rad - (self.current_yaw - self.start_yaw)
            
            # Normalize angle error to [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            # Use PID Controller for turning

            angular_vel = -self.pid_rot.calculate(angle_error)
            #print("angular vel:", angular_vel)
            #self.get_logger().info(f"{angle_error:.3f} = "f"{self.target_angle_rad:.3f} - "f"{self.current_yaw:.3f} - "f"{self.start_yaw:.3f}")

            twist_msg.angular.z = angular_vel
            
            # Check for rotation completion (e.g., error less than 1 degree or 0.017 rad)
            if abs(angle_error) < 0.034:
                self.get_logger().info("Rotation complete. Switching to Linear Move.")
                print("error within threshold")
                twist_msg.angular.z = 0.0 # Stop rotation immediately
                
                # Reset linear PID and switch state
                self.pid_dist.reset()
                if self.skip_linear:
                    self.current_state = self.STATE_FINISHED
                else:
                    self.current_state = self.STATE_MOVING

        elif self.current_state == self.STATE_MOVING:
            # Stage 2: Linear Distance Control
            
            # Distance remaining can now be negative if it overshoots
            distance_remaining = self.target_dist - self.current_dist_travelled
            
            # Use PID Controller for accurate distance tracking
            linear_vel = self.pid_dist.calculate(distance_remaining)
            
            # Apply the initial commanded direction.
            twist_msg.linear.x = linear_vel * self.movement_direction 
            
            # Check for movement completion (e.g., total distance remaining is less than 5cm)
            if abs(distance_remaining) < 0.01:
                self.get_logger().info("Linear move complete.")
                twist_msg.linear.x = 0.0 # Stop movement immediately
                self.current_state = self.STATE_FINISHED
                
            # Optional: Add a safety check for unexpected large yaw drift during straight motion
            # (Requires a secondary P-controller for steering correction, omitted for simplicity)

        elif self.current_state == self.STATE_FINISHED:
            # Movement sequence completed, send completion signal
            
            achieved_msg = Point()
            
            # Achieved Distance: current_dist_travelled
            achieved_msg.x = self.current_dist_travelled
            # Achieved Angle: Current Yaw relative to Start Yaw
            achieved_msg.y = self.current_yaw - self.start_yaw
            
            self.completion_publisher.publish(achieved_msg)
            self.get_logger().info("Published completion signal.")
            
            # Return to idle state, waiting for next command
            self.current_state = self.STATE_IDLE

        elif self.current_state == self.STATE_IDLE:
            # If idle, ensure velocity is zero
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
        # Publish the final Twist command regardless of the state (except IDLE)
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DistancePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()