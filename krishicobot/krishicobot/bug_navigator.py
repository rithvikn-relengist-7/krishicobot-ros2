#!/usr/bin/env python3
"""LiDAR/odometry waypoint navigation with obstacle-triggered wall following.

Reconstructed and cleaned from the author's e-YRC Krishi CoBot source screenshots.
Tune waypoints and thresholds for the actual robot and environment before use.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class BugNavigator(Node):
    def __init__(self):
        super().__init__("ebot_navigator")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Replace these example coordinates with the map's waypoint sequence.
        self.waypoints = [
            (-1.53, -1.95, 1.57),
            (0.13, 1.24, 0.00),
            (0.38, -3.32, -1.57),
        ]
        self.current_waypoint_idx = 0

        self.pos_tolerance = 0.30
        self.ang_tolerance = math.radians(10)
        self.linear_vel_max = 0.50
        self.angular_vel_max = 1.00
        self.kp_linear = 1.5
        self.kp_angular = 2.0
        self.min_obstacle_distance = 0.575
        self.kp_wall_follow = 0.7

        self.robot_x = -1.5339
        self.robot_y = -6.6156
        self.robot_yaw = 1.57
        self.lidar_data = None
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.following_wall = False

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("eBot waypoint navigator initialized")

    def scan_callback(self, msg: LaserScan):
        self.lidar_data = msg
        self.detect_obstacles()

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(sin_yaw, cos_yaw)

    def detect_obstacles(self):
        if self.lidar_data is None:
            return

        ranges = self.lidar_data.ranges
        angles = np.linspace(
            self.lidar_data.angle_min,
            self.lidar_data.angle_max,
            len(ranges),
        )
        valid = [
            (float(r), float(a))
            for r, a in zip(ranges, angles)
            if math.isfinite(r) and r > 0.0
        ]
        close = [(r, a) for r, a in valid if r < self.min_obstacle_distance]
        self.obstacle_detected = bool(close)
        self.obstacle_direction = min(close, key=lambda item: item[0])[1] if close else None

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def is_waypoint_reached(self):
        target = self.waypoints[self.current_waypoint_idx]
        distance = math.hypot(target[0] - self.robot_x, target[1] - self.robot_y)
        yaw_error = self.normalize_angle(target[2] - self.robot_yaw)
        return distance <= self.pos_tolerance and abs(yaw_error) <= self.ang_tolerance

    def can_reach_target_directly(self, target):
        # The original task logic resumes direct travel once the immediate
        # obstacle condition clears; this is intentionally a simple heuristic.
        return not self.obstacle_detected

    def wall_follow_control(self):
        cmd = Twist()
        if self.obstacle_direction is None:
            return cmd

        relative = self.normalize_angle(self.obstacle_direction - self.robot_yaw)
        desired_yaw = (
            self.robot_yaw - math.pi / 6
            if relative > 0
            else self.robot_yaw + math.pi / 6
        )
        yaw_error = self.normalize_angle(desired_yaw - self.robot_yaw)
        cmd.linear.x = 0.30
        cmd.angular.z = float(np.clip(
            self.kp_wall_follow * yaw_error,
            -self.angular_vel_max,
            self.angular_vel_max,
        ))
        return cmd

    def direct_navigation_control(self, distance, target_direction, target_yaw):
        cmd = Twist()
        if distance > self.pos_tolerance:
            cmd.linear.x = min(self.kp_linear * distance, self.linear_vel_max)
            heading_error = self.normalize_angle(target_direction - self.robot_yaw)
            cmd.angular.z = float(np.clip(
                self.kp_angular * heading_error,
                -self.angular_vel_max,
                self.angular_vel_max,
            ))
        else:
            yaw_error = self.normalize_angle(target_yaw - self.robot_yaw)
            cmd.angular.z = float(np.clip(
                self.kp_angular * yaw_error,
                -self.angular_vel_max,
                self.angular_vel_max,
            ))
        return cmd

    def navigate_to_waypoint(self):
        target = self.waypoints[self.current_waypoint_idx]
        dx = target[0] - self.robot_x
        dy = target[1] - self.robot_y
        distance = math.hypot(dx, dy)
        direction = math.atan2(dy, dx)

        if self.obstacle_detected:
            self.following_wall = True
        elif self.following_wall and self.can_reach_target_directly(target):
            self.following_wall = False
            self.get_logger().info("Obstacle cleared; resuming waypoint navigation")

        if self.following_wall:
            return self.wall_follow_control()
        return self.direct_navigation_control(distance, direction, target[2])

    def control_loop(self):
        if self.lidar_data is None:
            return

        if self.is_waypoint_reached():
            self.get_logger().info(
                f"Reached waypoint {self.current_waypoint_idx + 1}"
            )
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("All waypoints reached")
                return

        cmd = self.navigate_to_waypoint()
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        self.cmd_vel_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BugNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
