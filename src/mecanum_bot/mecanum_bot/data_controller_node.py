#!/usr/bin/env python3
"""
Path planner: absolute polar coordinates to relative movement commands.
No calibration bias - encoder compensation is handled in hardware.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
from collections import deque
from geometry_msgs.msg import Point


class DataControllerNode(Node):
    def __init__(self):
        super().__init__('data_controller_node')

        self.declare_parameter('target_topic_name', '/target_movement')
        self.declare_parameter('completion_topic_name', 'completed_movement')
        self.declare_parameter('init_radius_cm', 0.0)
        self.declare_parameter('init_angle_deg', 0.0)

        target_topic = self.get_parameter('target_topic_name').get_parameter_value().string_value
        completion_topic = self.get_parameter('completion_topic_name').get_parameter_value().string_value

        init_r_cm = self.get_parameter('init_radius_cm').get_parameter_value().double_value
        init_a_deg = self.get_parameter('init_angle_deg').get_parameter_value().double_value
        init_r_m = init_r_cm / 100.0
        init_a_rad = math.radians(init_a_deg)

        self.target_pub = self.create_publisher(Point, target_topic, 10)
        self.feedback_pub = self.create_publisher(String, '/robot_feedback', 10)

        self.create_subscription(String, '/polar_move_cmd', self.command_cb, 10)
        self.create_subscription(Point, completion_topic, self.done_cb, 10)
        self.create_subscription(String, '/sys_command', self.sys_cb, 10)

        self.curr_x = init_r_m * math.cos(init_a_rad)
        self.curr_y = init_r_m * math.sin(init_a_rad)
        self.curr_theta = init_a_rad

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_dist_abs = 0.0
        self.target_angle_abs = 0.0

        self.queue = deque()
        self.is_moving = False
        self.pending_delay = 0.0
        self.delay_timer = None

        self.get_logger().info(
            'DataController ready. pos=(%.1fcm,%.1fcm) heading=%.1fdeg' % (
                self.curr_x * 100, self.curr_y * 100, math.degrees(self.curr_theta)))

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def sys_cb(self, msg):
        if 'SHUTDOWN' in msg.data.upper():
            self.get_logger().fatal('###_INITIATE_HARDWARE_POWEROFF_###')

    def command_cb(self, msg):
        self.get_logger().info('Received: %s' % msg.data)
        try:
            data = json.loads(msg.data)
            if 'steps' in data:
                self.start_sequence(data['steps'])
            else:
                self.execute_step(data)
        except json.JSONDecodeError:
            self.get_logger().error('JSON parse error: %s' % msg.data)

    def start_sequence(self, steps):
        if self.is_moving:
            self.get_logger().warn('Busy - ignoring.')
            return
        self.queue.extend(steps)
        self.get_logger().info('Loaded %d waypoints.' % len(steps))
        self.next_step()

    def next_step(self):
        if not self.queue:
            self.is_moving = False
            self.get_logger().info('=== SEQUENCE COMPLETE ===')
            return
        self.execute_step(self.queue.popleft())

    def execute_step(self, cmd):
        self.is_moving = True
        try:
            radius_cm = float(cmd.get('radius', 0.0))
            angle_deg = float(cmd.get('angle', 0.0))
            delay_sec = float(cmd.get('delaySeconds', 0.0))
            self.pending_delay = max(0.0, delay_sec)
        except ValueError as e:
            self.get_logger().error('Invalid input: %s' % str(e))
            self.is_moving = False
            return

        radius_m = radius_cm / 100.0
        angle_rad = math.radians(angle_deg)

        self.target_dist_abs = radius_m
        self.target_angle_abs = angle_rad

        self.target_x = radius_m * math.cos(angle_rad)
        self.target_y = radius_m * math.sin(angle_rad)

        self.get_logger().info('------------------------------------------------')
        self.get_logger().info(
            'TARGET: %.1fcm @ %.1fdeg -> (%.3fm, %.3fm)' % (
                radius_cm, angle_deg, self.target_x, self.target_y))
        self.get_logger().info(
            'CURRENT: (%.3fm, %.3fm, %.1fdeg)' % (
                self.curr_x, self.curr_y, math.degrees(self.curr_theta)))

        dx = self.target_x - self.curr_x
        dy = self.target_y - self.curr_y
        rel_dist = math.sqrt(dx * dx + dy * dy)

        if rel_dist < 0.001:
            target_heading = self.curr_theta
        else:
            target_heading = math.atan2(dy, dx)

        rel_angle = self.normalize_angle(target_heading - self.curr_theta)

        self.get_logger().info(
            'COMMAND: dist=%.1fcm turn=%.1fdeg' % (
                rel_dist * 100, math.degrees(rel_angle)))

        goal = Point()
        goal.x = rel_dist
        goal.y = rel_angle
        goal.z = 0.0
        self.target_pub.publish(goal)

    def delay_cb(self):
        if self.delay_timer:
            self.delay_timer.cancel()
            self.delay_timer = None
        self.pending_delay = 0.0
        self.next_step()

    def done_cb(self, msg):
        if not self.is_moving:
            self.get_logger().warn('Unexpected completion. Ignoring.')
            return

        rel_dist = float(msg.x)
        rel_angle = float(msg.y)

        self.curr_theta += rel_angle
        self.curr_theta = self.normalize_angle(self.curr_theta)
        self.curr_x += rel_dist * math.cos(self.curr_theta)
        self.curr_y += rel_dist * math.sin(self.curr_theta)

        err_x = self.target_x - self.curr_x
        err_y = self.target_y - self.curr_y
        err_dist = math.sqrt(err_x * err_x + err_y * err_y)

        self.get_logger().info('------------------------------------------------')
        self.get_logger().info(
            'FEEDBACK: dist=%.1fcm turn=%.1fdeg' % (
                rel_dist * 100, math.degrees(rel_angle)))
        self.get_logger().info(
            'POSITION: (%.3fm, %.3fm, %.1fdeg) err=%.1fcm' % (
                self.curr_x, self.curr_y,
                math.degrees(self.curr_theta), err_dist * 100))

        # Feedback to app
        r_abs = math.sqrt(self.curr_x ** 2 + self.curr_y ** 2)
        a_abs = math.degrees(math.atan2(self.curr_y, self.curr_x))
        if a_abs < 0:
            a_abs += 360.0

        fb = String()
        fb.data = json.dumps({
            'moved_radius': round(r_abs * 100, 2),
            'moved_angle': round(a_abs, 2),
            'error_vector': round(err_dist * 100, 2),
        })
        self.feedback_pub.publish(fb)

        if self.queue:
            if self.pending_delay > 0.0:
                self.delay_timer = self.create_timer(self.pending_delay, self.delay_cb)
            else:
                self.next_step()
        else:
            self.is_moving = False
            self.get_logger().info(
                'IDLE. Final: (%.3fm, %.3fm, %.1fdeg)' % (
                    self.curr_x, self.curr_y, math.degrees(self.curr_theta)))


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