from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = "mecanum_bot"

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

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "controllers.yaml"]
    )
    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "ekf.yaml"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
            {"controller_manager.timeout": 10.0},
        ],
        output="both",
    )

    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file],
    )

    node_rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
    )

    node_pid = Node(
        package=package_name,
        executable="pid",
        name="pid_node",
        output="screen",
        parameters=[{
            # topics
            "control_rate": 25,
            "target_topic": "/pid_goal",
            "completion_topic": "/pid_result",
            "odom_topic": "/mecanum_base_controller/odometry",
            "cmd_vel_topic": "/mecanum_base_controller/reference_unstamped",
            "imu_topic": "/imu/data",

            # rotation PID (your stable rollback values)
            "kp_rot": 1.2,
            "ki_rot": 0.01,
            "kd_rot": 0.05,
            "max_angular_vel": 0.8,
            "min_angular_vel": 0.02,
            "max_ang_accel": 2.0,

            # rotation slowdown (your stable rollback)
            "rot_slowdown_gain": 1.4,
            "rot_min_apply_err": 0.08,

            # rotation braking cap (new)
            "max_ang_decel": 3.5,

            # gyro damping (your stable rollback value)
            "gyro_damping_gain": 0.35,

            # rotation completion (tuned from your IMU noise floor)
            "rotation_threshold_rad": 0.008,
            "yaw_rate_stop_rad_s": 0.12,
            "rotation_settle_s": 0.12,
            "rot_rate_settle_s": 0.06,
            "rotation_timeout_s": 25.0,

            # linear (your values + braking cap)
            "kp_lin": 1.2,
            "ki_lin": 0.17,
            "kd_lin": 0.25,
            "max_linear_vel": 0.20,
            "min_linear_vel": 0.01,
            "max_accel": 0.5,
            "max_decel": 0.35,
            "approach_gain": 2.30,

            # heading correction (your stable rollback + gyro damping)
            "kp_head": 3.0,
            "ki_head": 0.2,
            "kd_head": 0.25,
            "max_correction_rate": 0.5,
            "min_correction_rate": 0.01,
            "heading_deadband_rad": 0.005,
            "head_gyro_damping_gain": 0.14,

            # completion / timing
            "distance_threshold_m": 0.005,
            "linear_settle_s": 0.25,
            "movement_timeout_s": 30.0,

            # odom settle (new: fixes under-reporting)
            "odom_stable_eps_m": 0.002,
            "odom_stable_time_s": 0.20,
        }],
    )

    node_data_controller = Node(
        package=package_name,
        executable="data_controller",
        name="data_controller_node",
        output="screen",
        parameters=[{
            "init_angle_deg": 0.0,
            "init_radius_cm": 0.0,
            "target_topic_name": "/pid_goal",
            "completion_topic_name": "/pid_result",
        }],
    )

    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0",
        ],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_base_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60.0",
        ],
        output="screen",
    )

    delayed_spawners = TimerAction(
        period=15.0,
        actions=[spawn_broadcaster, spawn_controller],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        node_ekf,
        node_rosbridge,
        node_pid,
        node_data_controller,
        delayed_spawners,
    ])