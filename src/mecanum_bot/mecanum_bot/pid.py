#!/usr/bin/env python3
"""
MERGED PID: stable rollback structure + targeted fixes

From stable rollback (your best):
- rot_slowdown_gain * abs_err for rotation speed cap
- approach_gain * remain for linear speed
- PID with min_output stiction kick (but only when far from target)
- Simple ramp_to for accel limiting

Added fixes:
- Predictive braking cap for rotation: v <= sqrt(2 * ang_decel * remaining)
- Predictive braking cap for linear:   v <= sqrt(2 * decel * remaining)
- Rotation completion requires yaw_rate small (no more finishing while spinning)
- Odom settle before publishing /pid_result (no more under-reporting)
- Direct quat_to_yaw (no tf_transformations dependency)
- Heading gyro damping (reduces wobble)
- Uses wheel odom by default (less lag than EKF)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PIDController:
    def __init__(self, kp, ki, kd, max_out, min_out, dt):
        self.Kp = float(kp)
        self.Ki = float(ki)
        self.Kd = float(kd)
        self.max_output = float(max_out)
        self.min_output = float(min_out)
        self.dt = max(float(dt), 1e-4)
        self.integral = 0.0
        self.last_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def calculate(self, error):
        P = self.Kp * error

        self.integral += error * self.dt
        if self.Ki != 0.0:
            lim = self.max_output / (self.Ki + 1e-12)
            self.integral = clamp(self.integral, -lim, lim)
        I = self.Ki * self.integral

        D = self.Kd * (error - self.last_error) / self.dt
        self.last_error = error

        out = clamp(P + I + D, -self.max_output, self.max_output)

        # Stiction kick — but caller decides when to apply it
        return out


class DistancePIDController(Node):
    IDLE = 0
    ROTATING = 1
    ROT_SETTLE = 2
    MOVING = 3
    LIN_SETTLE = 4
    FINISHED = 5

    def __init__(self):
        super().__init__("pid_node")

        # Topics & rate
        self.declare_parameter("control_rate", 25)
        self.declare_parameter("target_topic", "/pid_goal")
        self.declare_parameter("completion_topic", "/pid_result")
        self.declare_parameter("odom_topic", "/mecanum_base_controller/odometry")
        self.declare_parameter("cmd_vel_topic", "/mecanum_base_controller/reference_unstamped")
        self.declare_parameter("imu_topic", "/imu/data")

        # Rotation PID
        self.declare_parameter("kp_rot", 1.2)
        self.declare_parameter("ki_rot", 0.01)
        self.declare_parameter("kd_rot", 0.05)
        self.declare_parameter("max_angular_vel", 0.8)
        self.declare_parameter("min_angular_vel", 0.02)
        self.declare_parameter("max_ang_accel", 2.0)

        # Rotation slowdown (from your stable rollback)
        self.declare_parameter("rot_slowdown_gain", 1.4)
        self.declare_parameter("rot_min_apply_err", 0.08)

        # Rotation braking cap (new fix: prevents overshoot)
        self.declare_parameter("max_ang_decel", 3.5)

        # Gyro damping
        self.declare_parameter("gyro_damping_gain", 0.35)

        # Rotation completion
        self.declare_parameter("rotation_threshold_rad", 0.008)
        self.declare_parameter("yaw_rate_stop_rad_s", 0.05)
        self.declare_parameter("rotation_settle_s", 0.20)
        self.declare_parameter("rot_rate_settle_s", 0.10)
        self.declare_parameter("rotation_timeout_s", 25.0)

        # Linear PID
        self.declare_parameter("kp_lin", 1.2)
        self.declare_parameter("ki_lin", 0.17)
        self.declare_parameter("kd_lin", 0.25)
        self.declare_parameter("max_linear_vel", 0.20)
        self.declare_parameter("min_linear_vel", 0.01)
        self.declare_parameter("max_accel", 0.5)

        # Linear braking cap (new fix: prevents coast overshoot)
        self.declare_parameter("max_decel", 0.35)

        # Approach slowdown (from your stable rollback)
        self.declare_parameter("approach_gain", 2.5)

        # Heading correction
        self.declare_parameter("kp_head", 3.0)
        self.declare_parameter("ki_head", 0.2)
        self.declare_parameter("kd_head", 0.25)
        self.declare_parameter("max_correction_rate", 0.5)
        self.declare_parameter("min_correction_rate", 0.01)
        self.declare_parameter("heading_deadband_rad", 0.01)
        self.declare_parameter("head_gyro_damping_gain", 0.10)

        # Completion / timing
        self.declare_parameter("distance_threshold_m", 0.005)
        self.declare_parameter("linear_settle_s", 0.25)
        self.declare_parameter("movement_timeout_s", 30.0)

        # Odom settle (new fix: wait for distance to stop changing)
        self.declare_parameter("odom_stable_eps_m", 0.002)
        self.declare_parameter("odom_stable_time_s", 0.20)

        # Read all params
        self.rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, self.rate)

        self.target_topic = self.get_parameter("target_topic").value
        self.completion_topic = self.get_parameter("completion_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value

        self.MAX_ANG = float(self.get_parameter("max_angular_vel").value)
        self.MIN_ANG = float(self.get_parameter("min_angular_vel").value)
        self.MAX_ANG_ACCEL = float(self.get_parameter("max_ang_accel").value)
        self.MAX_ANG_DECEL = float(self.get_parameter("max_ang_decel").value)
        self.ROT_SLOW_K = float(self.get_parameter("rot_slowdown_gain").value)
        self.ROT_MIN_APPLY_ERR = float(self.get_parameter("rot_min_apply_err").value)
        self.GYRO_DAMP = float(self.get_parameter("gyro_damping_gain").value)

        self.ROT_THRESH = float(self.get_parameter("rotation_threshold_rad").value)
        self.YAW_RATE_STOP = float(self.get_parameter("yaw_rate_stop_rad_s").value)
        self.ROT_SETTLE_T = float(self.get_parameter("rotation_settle_s").value)
        self.ROT_RATE_SETTLE = float(self.get_parameter("rot_rate_settle_s").value)
        self.ROT_TIMEOUT = float(self.get_parameter("rotation_timeout_s").value)

        self.MAX_LIN = float(self.get_parameter("max_linear_vel").value)
        self.MIN_LIN = float(self.get_parameter("min_linear_vel").value)
        self.MAX_ACCEL = float(self.get_parameter("max_accel").value)
        self.MAX_DECEL = float(self.get_parameter("max_decel").value)
        self.APPROACH_K = float(self.get_parameter("approach_gain").value)

        self.MAX_CORR = float(self.get_parameter("max_correction_rate").value)
        self.MIN_CORR = float(self.get_parameter("min_correction_rate").value)
        self.HEAD_DB = float(self.get_parameter("heading_deadband_rad").value)
        self.HEAD_GYRO_DAMP = float(self.get_parameter("head_gyro_damping_gain").value)

        self.DIST_THRESH = float(self.get_parameter("distance_threshold_m").value)
        self.LIN_SETTLE_T = float(self.get_parameter("linear_settle_s").value)
        self.MOV_TIMEOUT = float(self.get_parameter("movement_timeout_s").value)

        self.ODOM_STABLE_EPS = float(self.get_parameter("odom_stable_eps_m").value)
        self.ODOM_STABLE_T = float(self.get_parameter("odom_stable_time_s").value)

        # Controllers (same structure as your stable rollback)
        self.pid_rot = PIDController(
            self.get_parameter("kp_rot").value,
            self.get_parameter("ki_rot").value,
            self.get_parameter("kd_rot").value,
            self.MAX_ANG, self.MIN_ANG, self.dt
        )
        self.pid_head = PIDController(
            self.get_parameter("kp_head").value,
            self.get_parameter("ki_head").value,
            self.get_parameter("kd_head").value,
            self.MAX_CORR, self.MIN_CORR, self.dt
        )

        # Pub/Sub
        self.create_subscription(Point, self.target_topic, self.target_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.done_pub = self.create_publisher(Point, self.completion_topic, 10)

        # Runtime state
        self.state = self.IDLE

        self.target_dist = 0.0
        self.target_angle = 0.0
        self.move_dir = 1.0
        self.skip_linear = False

        self.ox = self.oy = 0.0
        self.ox0 = self.oy0 = 0.0
        self.dist_travelled = 0.0

        self.imu_yaw = 0.0
        self.imu_yaw0 = 0.0
        self.yaw_rate_z = 0.0
        self.imu_valid = False

        self.v_lin = 0.0
        self.v_ang = 0.0

        self.t_phase = 0.0
        self.t_settle = 0.0
        self.t_rate_ok = None

        self.last_dist = 0.0
        self.t_dist_stable = None

        self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            f"MERGED PID ready {self.rate:.0f}Hz | "
            f"rollback structure + braking cap + odom settle"
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def ramp_to(self, cur, tgt, accel):
        step = accel * self.dt
        d = tgt - cur
        if abs(d) <= step:
            return tgt
        return cur + math.copysign(step, d)

    def ramp_to_asym(self, cur, tgt, acc_up, acc_down):
        d = tgt - cur
        step = (acc_up if d >= 0.0 else acc_down) * self.dt
        if abs(d) <= step:
            return tgt
        return cur + math.copysign(step, d)

    # Callbacks
    def imu_cb(self, msg: Imu):
        self.yaw_rate_z = float(msg.angular_velocity.z)
        q = msg.orientation
        if abs(q.w) < 1e-8 and abs(q.x) < 1e-8 and abs(q.y) < 1e-8 and abs(q.z) < 1e-8:
            self.imu_valid = False
            return
        self.imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.imu_valid = True

    def odom_cb(self, msg: Odometry):
        self.ox = float(msg.pose.pose.position.x)
        self.oy = float(msg.pose.pose.position.y)
        dx = self.ox - self.ox0
        dy = self.oy - self.oy0
        self.dist_travelled = math.sqrt(dx * dx + dy * dy)

    def target_cb(self, msg: Point):
        if self.state not in (self.IDLE, self.FINISHED):
            self.get_logger().warn("Busy - ignoring new target.")
            return
        if not self.imu_valid:
            self.get_logger().warn("IMU not valid yet - ignoring target.")
            return

        self.target_dist = abs(float(msg.x))
        self.move_dir = math.copysign(1.0, float(msg.x)) if msg.x != 0.0 else 1.0
        self.target_angle = float(msg.y)
        self.skip_linear = self.target_dist < 0.005

        self.imu_yaw0 = self.imu_yaw
        self.ox0 = self.ox
        self.oy0 = self.oy
        self.dist_travelled = 0.0

        self.pid_rot.reset()
        self.pid_head.reset()
        self.v_lin = 0.0
        self.v_ang = 0.0

        self.t_phase = self.now()
        self.t_rate_ok = None
        self.last_dist = 0.0
        self.t_dist_stable = None

        self.state = self.ROTATING

        self.get_logger().info(
            f"Target: turn={math.degrees(self.target_angle):.1f}deg "
            f"dist={self.target_dist*100:.1f}cm"
        )

    # Main loop
    def loop(self):
        tw = Twist()

        # ======================== ROTATING ========================
        if self.state == self.ROTATING:
            if (self.now() - self.t_phase) > self.ROT_TIMEOUT:
                self.get_logger().warn("Rotation timeout")
                self.state = self.LIN_SETTLE
                self.t_settle = self.now()
            else:
                turned = norm_angle(self.imu_yaw - self.imu_yaw0)
                err = norm_angle(self.target_angle - turned)
                abs_err = abs(err)

                # PID output
                cmd = -self.pid_rot.calculate(err)

                # Gyro damping
                cmd = cmd - (self.GYRO_DAMP * self.yaw_rate_z)

                # Speed cap 1: your stable rollback linear cap
                cap_linear = self.ROT_SLOW_K * abs_err

                # Speed cap 2: predictive braking cap (new)
                cap_brake = math.sqrt(max(0.0, 2.0 * self.MAX_ANG_DECEL * abs_err))

                # Use the LOWER of both caps (whichever is more conservative)
                max_now = min(self.MAX_ANG, cap_linear, cap_brake)

                # Stiction: only force min vel when far from target
                if abs_err >= self.ROT_MIN_APPLY_ERR:
                    max_now = max(max_now, self.MIN_ANG)

                cmd = clamp(cmd, -max_now, +max_now)

                # Accel ramp
                self.v_ang = self.ramp_to(self.v_ang, cmd, self.MAX_ANG_ACCEL)

                tw.angular.z = self.v_ang
                tw.linear.x = 0.0

                # FIX: require BOTH angle small AND yaw_rate small
                if abs_err < self.ROT_THRESH and abs(self.yaw_rate_z) < self.YAW_RATE_STOP:
                    self.v_ang = 0.0
                    tw.angular.z = 0.0
                    self.pid_rot.reset()
                    self.t_settle = self.now()
                    self.t_rate_ok = None
                    self.state = self.ROT_SETTLE

        # ======================== ROT_SETTLE ========================
        elif self.state == self.ROT_SETTLE:
            tw.linear.x = 0.0
            tw.angular.z = 0.0

            if abs(self.yaw_rate_z) <= self.YAW_RATE_STOP:
                if self.t_rate_ok is None:
                    self.t_rate_ok = self.now()
            else:
                self.t_rate_ok = None

            min_ok = (self.now() - self.t_settle) >= self.ROT_SETTLE_T
            rate_ok = (self.t_rate_ok is not None) and \
                      ((self.now() - self.t_rate_ok) >= self.ROT_RATE_SETTLE)

            if min_ok and rate_ok:
                turned = norm_angle(self.imu_yaw - self.imu_yaw0)
                err = norm_angle(self.target_angle - turned)

                if abs(err) > self.ROT_THRESH * 3.0:
                    self.pid_rot.reset()
                    self.t_phase = self.now()
                    self.state = self.ROTATING
                else:
                    if self.skip_linear:
                        self.state = self.LIN_SETTLE
                        self.t_settle = self.now()
                        self.last_dist = self.dist_travelled
                        self.t_dist_stable = None
                    else:
                        self.pid_head.reset()
                        self.t_phase = self.now()
                        self.state = self.MOVING

        # ======================== MOVING ========================
        elif self.state == self.MOVING:
            if (self.now() - self.t_phase) > self.MOV_TIMEOUT:
                self.get_logger().warn("Movement timeout")
                self.state = self.LIN_SETTLE
                self.t_settle = self.now()
                self.last_dist = self.dist_travelled
                self.t_dist_stable = None
            else:
                remain = max(0.0, self.target_dist - self.dist_travelled)

                # Speed cap 1: your stable rollback approach gain
                v_approach = self.APPROACH_K * remain

                # Speed cap 2: predictive braking (new)
                v_brake = math.sqrt(max(0.0, 2.0 * self.MAX_DECEL * remain))

                # Use the lower of both
                v_tgt = min(self.MAX_LIN, v_approach, v_brake)

                # Accel/decel ramp (decel can be faster)
                self.v_lin = self.ramp_to_asym(
                    self.v_lin, v_tgt, self.MAX_ACCEL, self.MAX_DECEL
                )

                # Stiction only when far from target
                if remain > 0.05 and 0.0 < abs(self.v_lin) < self.MIN_LIN:
                    self.v_lin = math.copysign(self.MIN_LIN, self.v_lin)

                tw.linear.x = self.v_lin * self.move_dir

                # Heading correction (same as stable rollback + gyro damping)
                turned = norm_angle(self.imu_yaw - self.imu_yaw0)
                h_err = norm_angle(self.target_angle - turned)

                if abs(h_err) < self.HEAD_DB:
                    corr = 0.0
                    self.pid_head.reset()
                else:
                    corr = -self.pid_head.calculate(h_err)

                    # Gyro damping on heading (reduces wobble)
                    corr = corr - (self.HEAD_GYRO_DAMP * self.yaw_rate_z)

                    corr = clamp(corr, -self.MAX_CORR, +self.MAX_CORR)

                    # Stiction for heading only when error is meaningful
                    if abs(h_err) > 0.03 and 0.0 < abs(corr) < self.MIN_CORR:
                        corr = math.copysign(self.MIN_CORR, corr)

                tw.angular.z = corr

                if remain <= self.DIST_THRESH or self.dist_travelled >= self.target_dist:
                    self.v_lin = 0.0
                    tw.linear.x = 0.0
                    tw.angular.z = 0.0
                    self.state = self.LIN_SETTLE
                    self.t_settle = self.now()
                    self.last_dist = self.dist_travelled
                    self.t_dist_stable = None

        # ======================== LIN_SETTLE ========================
        elif self.state == self.LIN_SETTLE:
            tw.linear.x = 0.0
            tw.angular.z = 0.0
            self.v_lin = 0.0
            self.v_ang = 0.0

            # FIX: wait for odom to stop changing before publishing
            d_change = abs(self.dist_travelled - self.last_dist)
            self.last_dist = self.dist_travelled

            if d_change <= self.ODOM_STABLE_EPS:
                if self.t_dist_stable is None:
                    self.t_dist_stable = self.now()
            else:
                self.t_dist_stable = None

            time_ok = (self.now() - self.t_settle) >= self.LIN_SETTLE_T
            odom_ok = (self.t_dist_stable is not None) and \
                      ((self.now() - self.t_dist_stable) >= self.ODOM_STABLE_T)

            if time_ok and odom_ok:
                self.state = self.FINISHED

        # ======================== FINISHED ========================
        elif self.state == self.FINISHED:
            tw.linear.x = 0.0
            tw.angular.z = 0.0

            total_turn = norm_angle(self.imu_yaw - self.imu_yaw0)

            done = Point()
            done.x = float(self.dist_travelled)
            done.y = float(total_turn)
            done.z = 0.0
            self.done_pub.publish(done)

            self.state = self.IDLE

        # ======================== IDLE ========================
        else:
            tw.linear.x = 0.0
            tw.angular.z = 0.0

        self.cmd_pub.publish(tw)


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


if __name__ == "__main__":
    main()