from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. DEFINE PATHS
    package_name = "mecanum_bot"
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "description", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Config Files
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "controllers.yaml"]
    )
    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "ekf.yaml"]
    )

    # 2. DEFINE NODES

    # A. ROBOT STATE PUBLISHER
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # B. CONTROLLER MANAGER
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            controllers_file,
            # This handles the internal "Switch Controller" timeout
            {'controller_manager.timeout': 10.0} 
        ],
        output="both",
    )

    # C. ROBOT LOCALIZATION (EKF)
    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file],
    )

    # D. ROSBRIDGE
    node_rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{'port': 9090}]
    )

    # E. PID CONTROLLER
    node_pid = Node(
        package=package_name,
        executable="pid",
        name="pid_node",
        output="screen",
        parameters=[{
            'control_rate': 100,
            'target_topic': '/pid_goal',
            'completion_topic': '/pid_result',
            'odom_topic': '/odometry/filtered',
            'cmd_vel_topic': '/mecanum_base_controller/reference_unstamped',

            # --- Angular (Rotation) ---
            'kp_rot': 1.2,           # Increased from 0.15 for snappier response
            'ki_rot': 0.01,          # Tiny amount to overcome friction at the very end
            'kd_rot': 0.05,          # Dampens the approach to prevent overshoot
            'max_angular_vel': 0.8,  # Increased to 1.5 (approx 85 deg/s)
            'min_angular_vel': 0.05,  # Minimum kick to keep motors turning
    
            # --- Linear (Movement) ---
            'kp_lin': 8.0,           # Increased for faster acceleration
            'ki_lin': 0.0,           # Keep at 0 unless it stops short of target
            'kd_lin': 0.1,           # Helps stop smoothly
            'max_linear_vel': 0.4,   # Medium walking speed
            'min_linear_vel': 0.08,  # Minimum power to move the weight
        }]
    )

    # F. DATA CONTROLLER
    node_data_controller = Node(
        package=package_name,
        executable="data_controller",
        name="data_controller_node",
        output="screen",
        parameters=[{
            'target_topic_name': '/pid_goal',
            'completion_topic_name': '/pid_result'
        }]
    )

    # G. SPAWNERS (Corrected Flags)
    
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0" # <--- FIXED FLAG HERE
        ],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_base_controller", 
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0" # <--- FIXED FLAG HERE
        ],
        output="screen",
    )

    # 3. DEFINE LAUNCH ORDER
    delayed_spawners = TimerAction(
        period=15.0,
        actions=[spawn_broadcaster, spawn_controller]
    )

    # 4. LAUNCH THEM
    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        node_ekf,
        node_rosbridge,
        node_pid,
        node_data_controller,
        delayed_spawners, 
    ])