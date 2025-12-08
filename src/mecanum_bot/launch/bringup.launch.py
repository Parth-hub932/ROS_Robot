from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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

    # A. HARDWARE DRIVER (Talks to PSoC via UART)
    # hardware_node = Node(
    #     package=package_name,
    #     executable="hardware_driver",
    #     name="psoc_hardware_driver",
    #     output="screen"
    # )

    # B. ROBOT STATE PUBLISHER (Publishes TF from URDF)
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # C. CONTROLLER MANAGER (The Brain - ros2_control)
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
    )

    # D. ROBOT LOCALIZATION (EKF - Fuses IMU + Odom)
    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file],
    )

    # E. ROSBRIDGE (For Android App WebSocket)
    node_rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{'port': 9090}]
    )

    # F. PID CONTROLLER (Calculates velocity to reach polar goal)
    # --- UPDATED: Passing parameters to map topics correctly ---
    node_pid = Node(
        package=package_name,
        executable="pid",
        name="pid_node",
        output="screen",
        parameters=[{
            # Internal Topic Mapping (Must match Data Controller)
            'target_topic': '/pid_goal',
            'completion_topic': '/pid_result',
            
            # System Topic Mapping (Connecting to Real Robot Topics)
            'odom_topic': '/odometry/filtered',
            'cmd_vel_topic': '/mecanum_base_controller/cmd_vel_unstamped',
            
            # PID Tuning (You can tune these here without editing code!)
            'kp_lin': 1.0,
            'kp_rot': 1.5,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0
        }]
    )

    # G. DATA CONTROLLER (Parses JSON from Android and manages sequence)
    # --- UPDATED: Passing parameters to map topics correctly ---
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

    # H. SPAWNERS (Start the specific controllers inside controller_manager)
    
    # H1. Joint State Broadcaster (Publishes wheel positions to TF)
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # H2. Mecanum Base Controller (Handles kinematic math)
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_base_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 3. DEFINE LAUNCH ORDER
    # We use a TimerAction to give the controller_manager a few seconds to wake up 
    # before trying to spawn the controllers. This prevents "Node not available" errors.
    
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[spawn_broadcaster, spawn_controller]
    )

    # 4. LAUNCH THEM
    return LaunchDescription([
        # hardware_node,
        node_robot_state_publisher,
        node_controller_manager,
        node_ekf,
        node_rosbridge,
        node_pid,
        node_data_controller,
        delayed_spawners, 
    ])