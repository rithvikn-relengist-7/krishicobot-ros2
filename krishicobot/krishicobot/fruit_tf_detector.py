#!/usr/bin/env python3
"""OpenCV fruit-colour detection + depth-based 3-D TF publication.

This is a cleaned reconstruction of the FruitsTF node shown in the supplied
e-YRC source PDF. Camera calibration, HSV thresholds, frames and marker size
must be verified against the actual camera before running on hardware.
"""
import math

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image


class FruitsTF(Node):
    def __init__(self):
        super().__init__("fruits_tf")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("color_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("show_image", False)
        self.declare_parameter("team_id", 3614)

        self.target_frame = self.get_parameter("target_frame").value
        self.show_image = self.get_parameter("show_image").value
        self.team_id = self.get_parameter("team_id").value
        self.source_frame = "camera_link"
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        # Values visible in the source screenshots; tune for the target fruit
        # and lighting conditions. These are not universal fruit thresholds.
        self.hsv_lower = np.array([0, 0, 50], dtype=np.uint8)
        self.hsv_upper = np.array([180, 50, 155], dtype=np.uint8)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            Image, self.get_parameter("color_topic").value,
            self.color_callback, 10
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self.depth_callback, 10
        )
        self.create_timer(0.2, self.process_image)
        self.get_logger().info("FruitsTF node started")

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.source_frame = msg.header.frame_id or self.source_frame
        except Exception as exc:
            self.get_logger().error(f"Color CvBridge error: {exc}")

    def depth_callback(self, msg):
        try:
            self.depth_image = np.asarray(
                self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            )
            self.source_frame = msg.header.frame_id or self.source_frame
        except Exception as exc:
            self.get_logger().error(f"Depth CvBridge error: {exc}")

    def detect_bad_fruit(self, image):
        height, width = image.shape[:2]
        spatial_mask = np.zeros((height, width), dtype=np.uint8)
        roi_width = int(width * 0.40)
        cv2.rectangle(spatial_mask, (0, 0), (roi_width, height), 255, -1)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.bitwise_and(color_mask, color_mask, mask=spatial_mask)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        centers = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h < 80:
                continue
            cx, cy = x + w // 2, y + h // 2
            centers.append((cx, cy))
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(
                image, "Candidate", (x, max(0, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
            )
        return centers, image, mask

    @staticmethod
    def depth_roi_mean_meters(depth, cx, cy, roi_size=15):
        h, w = depth.shape[:2]
        half = roi_size // 2
        y0, y1 = max(0, cy - half), min(h, cy + half + 1)
        x0, x1 = max(0, cx - half), min(w, cx + half + 1)
        roi = depth[y0:y1, x0:x1]
        if roi.size == 0:
            return None

        if np.issubdtype(roi.dtype, np.floating):
            valid = roi[np.isfinite(roi) & (roi > 0.0)]
            return float(np.mean(valid)) if valid.size else None

        valid = roi[roi > 0]
        return float(np.mean(valid)) / 1000.0 if valid.size else None

    def publish_detection(self, detection_id, cx, cy, depth_m, transform):
        # Pinhole projection using the intrinsics shown in the supplied source.
        fx, fy = 915.3003540039062, 914.0320434570312
        cx0, cy0 = 642.724365234375, 361.9780578613281
        x_cam = depth_m * (cx - cx0) / fx
        y_cam = depth_m * (cy - cy0) / fy

        point = PointStamped()
        point.header.frame_id = self.source_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = depth_m
        point.point.y = -x_cam
        point.point.z = -y_cam

        point_base = tf2_geometry_msgs.do_transform_point(point, transform)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.target_frame
        tf_msg.child_frame_id = f"{self.team_id}_bad_fruit_{detection_id}"
        tf_msg.transform.translation.x = point_base.point.x
        tf_msg.transform.translation.y = point_base.point.y
        tf_msg.transform.translation.z = point_base.point.z
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

    def process_image(self):
        if self.color_image is None or self.depth_image is None:
            return

        image = self.color_image.copy()
        depth = self.depth_image.copy()
        detections, visual, mask = self.detect_bad_fruit(image)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as exc:
            self.get_logger().warning(f"TF lookup unavailable: {exc}")
            return

        for detection_id, (cx, cy) in enumerate(detections, start=1):
            if cy >= depth.shape[0] or cx >= depth.shape[1]:
                continue
            depth_m = self.depth_roi_mean_meters(depth, cx, cy)
            if depth_m is None or depth_m <= 0.05:
                continue
            try:
                self.publish_detection(detection_id, cx, cy, depth_m, transform)
                cv2.circle(visual, (cx, cy), 6, (255, 0, 0), -1)
            except Exception as exc:
                self.get_logger().warning(f"Could not publish detection: {exc}")

        if self.show_image:
            cv2.imshow("fruit_candidates", visual)
            cv2.imshow("fruit_mask", mask)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_image:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FruitsTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
