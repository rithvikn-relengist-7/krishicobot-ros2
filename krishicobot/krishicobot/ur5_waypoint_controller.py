#!/usr/bin/env python3
"""Waypoint controller for a UR5 end-effector using TF pose feedback.

Motion output is disabled by default. Verify the robot model, workspace,
waypoints, speed limits and emergency-stop procedure before enabling motion.
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener


class UR5WaypointNode(Node):
    def __init__(self):
        super().__init__("ur5_waypoint_node")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "ee_link")
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.10)

        self.enable_motion = self.get_parameter("enable_motion").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)

        self.twist_pub = self.create_publisher(Twist, "/delta_twist_cmds", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Example waypoints transcribed from the supplied task-code screenshots.
        # Confirm these poses in the target robot's coordinate frame before use.
        self.waypoints = [
            ("P1", np.array([-0.214, -0.532, 0.557]),
             np.array([0.707, 0.028, 0.034, 0.707])),
            ("P2", np.array([0.159, 0.201, 0.815]),
             np.array([0.029, 0.997, 0.045, 0.033])),
            ("P3", np.array([-0.159, 0.501, 0.415]),
             np.array([0.029, 0.997, 0.045, 0.033])),
            ("P4", np.array([-0.806, 0.010, 0.182]),
             np.array([-0.684, 0.726, 0.050, 0.008])),
        ]
        self.index = 0
        self.state = "moving"
        self.pause_started = None
        self.position_tolerance = 0.15
        self.orientation_tolerance = 0.15
        self.stop_duration = 1.0
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"UR5 waypoint node ready; motion enabled={self.enable_motion}"
        )

    @staticmethod
    def quaternion_error(current, target):
        current_rot = Rotation.from_quat(current)
        target_rot = Rotation.from_quat(target)
        return (target_rot * current_rot.inv()).as_rotvec()

    def current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            t, q = tf.transform.translation, tf.transform.rotation
            return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])
        except Exception as exc:
            self.get_logger().warning(f"TF lookup failed: {exc}")
            return None, None

    def publish_command(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = map(float, linear)
        msg.angular.x, msg.angular.y, msg.angular.z = map(float, angular)
        if self.enable_motion:
            self.twist_pub.publish(msg)
        else:
            self.twist_pub.publish(Twist())
            self.get_logger().debug("Dry-run: computed command suppressed")

    def control_loop(self):
        if self.index >= len(self.waypoints):
            self.twist_pub.publish(Twist())
            return

        position, orientation = self.current_pose()
        if position is None:
            self.twist_pub.publish(Twist())
            return

        name, target_pos, target_ori = self.waypoints[self.index]
        pos_error = target_pos - position
        ori_error = self.quaternion_error(orientation, target_ori)
        pos_norm = float(np.linalg.norm(pos_error))
        ori_norm = float(np.linalg.norm(ori_error))

        if self.state == "pausing":
            self.twist_pub.publish(Twist())
            if time.monotonic() - self.pause_started >= self.stop_duration:
                self.index += 1
                self.state = "moving"
            return

        if pos_norm <= self.position_tolerance and ori_norm <= self.orientation_tolerance:
            self.get_logger().info(f"Reached {name}; pausing")
            self.twist_pub.publish(Twist())
            self.state = "pausing"
            self.pause_started = time.monotonic()
            return

        linear = np.zeros(3)
        angular = np.zeros(3)
        if pos_norm > 0.01:
            linear = pos_error / pos_norm * min(self.linear_speed, pos_norm)
        if ori_norm > 0.01:
            angular = ori_error / ori_norm * min(self.angular_speed, ori_norm)
        self.publish_command(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = UR5WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
