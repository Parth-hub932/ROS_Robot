from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "mecanum_bot"
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(package_name), "description", "robot.urdf.xacro"]),
    ])
    robot_description = {"robot_description": robot_description_content}
    controllers_file = PathJoinSubstitution([FindPackageShare(package_name), "config", "controllers.yaml"])

    # Only essential nodes - NO PID, NO EKF, NO DATA CONTROLLER
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_base_controller", "-c", "/controller_manager"],
        output="screen",
    )

    delayed_spawners = TimerAction(period=5.0, actions=[spawn_broadcaster, spawn_controller])

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        delayed_spawners,
    ])