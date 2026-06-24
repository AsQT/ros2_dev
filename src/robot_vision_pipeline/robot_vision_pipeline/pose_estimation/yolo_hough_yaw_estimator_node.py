#!/usr/bin/env python3
"""Estimate object direction from YOLO bbox ROI using Canny + HoughLinesP."""

from __future__ import annotations

import json
import math
import threading
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


def normalize_yaw_0_90(angle: float) -> float:
    angle = float(angle) % 180.0
    if angle > 90.0:
        angle = 180.0 - angle
    return float(angle)


class YoloHoughYawEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_hough_yaw_estimator_node")

        self.declare_parameter("target_class", "wood")
        self.declare_parameter("yolo_json_topic", "/vision/yolo/detections_json")
        self.declare_parameter("detections_json_topic", "/vision/yolo/detections_json")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_json_topic", "/vision/yolo/hough_yaw_json")
        self.declare_parameter(
            "debug_image_topic", "/vision/debug_hough_yaw_image"
        )
        self.declare_parameter("debug_edges_topic", "/vision/debug_hough_edges")
        self.declare_parameter("bbox_padding", 25)
        self.declare_parameter("canny_low", 30)
        self.declare_parameter("canny_high", 100)
        self.declare_parameter("hough_threshold", 10)
        self.declare_parameter("min_line_length", 10)
        self.declare_parameter("max_line_gap", 8)
        self.declare_parameter("arrow_length", 60)
        self.declare_parameter("publish_debug_image", True)

        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.yolo_json_topic = str(self.get_parameter("yolo_json_topic").value)
        detections_json_topic = str(self.get_parameter("detections_json_topic").value)
        if detections_json_topic:
            self.yolo_json_topic = detections_json_topic
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_json_topic = str(self.get_parameter("output_json_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.debug_edges_topic = str(self.get_parameter("debug_edges_topic").value)
        self.bbox_padding = int(self.get_parameter("bbox_padding").value)
        self.canny_low = int(self.get_parameter("canny_low").value)
        self.canny_high = int(self.get_parameter("canny_high").value)
        self.hough_threshold = int(self.get_parameter("hough_threshold").value)
        self.min_line_length = int(self.get_parameter("min_line_length").value)
        self.max_line_gap = int(self.get_parameter("max_line_gap").value)
        self.arrow_length = int(self.get_parameter("arrow_length").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.bridge = CvBridge()
        self._image_lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_header: Any = None
        self._warned_no_image = False

        json_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.yolo_json_topic,
            self._on_yolo_json,
            json_qos,
        )
        self.output_pub = self.create_publisher(String, self.output_json_topic, json_qos)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.debug_edges_pub = self.create_publisher(Image, self.debug_edges_topic, 10)

        self.get_logger().info(
            "YOLO Hough yaw estimator started | "
            f"target_class={self.target_class}, json={self.yolo_json_topic}, "
            f"image={self.image_topic}, output={self.output_json_topic}"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"Image cv_bridge error: {exc}")
            return

        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            self.get_logger().warn("Received empty color image; ignoring it.")
            return

        with self._image_lock:
            self._latest_image = frame.copy()
            self._latest_image_header = msg.header
            self._warned_no_image = False

    def _on_yolo_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid YOLO JSON: {exc}")
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            self.get_logger().warn("YOLO JSON field 'detections' is not a list.")
            return

        with self._image_lock:
            frame = None if self._latest_image is None else self._latest_image.copy()
            image_header = self._latest_image_header

        if frame is None:
            if not self._warned_no_image:
                self.get_logger().warn(
                    "No color image received yet; cannot estimate Hough yaw."
                )
                self._warned_no_image = True
            return

        debug_image = frame.copy()
        debug_edges = np.zeros(frame.shape[:2], dtype=np.uint8)
        output_detections = []
        target_count = 0

        for det in detections:
            class_name = str(det.get("class_name", "")).strip().lower()
            if class_name != self.target_class:
                continue

            target_count += 1
            result = self._process_detection(frame, det, debug_image, debug_edges)
            if result is not None:
                output_detections.append(result)

        if target_count == 0:
            self.get_logger().info(
                f"No detection with target_class='{self.target_class}' in YOLO JSON."
            )

        frame_id = self._frame_id_from_payload(payload, image_header)
        stamp = payload.get("stamp", {})
        out_payload = {
            "header": {
                "stamp": {
                    "sec": int(
                        stamp.get(
                            "sec",
                            getattr(getattr(image_header, "stamp", None), "sec", 0),
                        )
                    ),
                    "nanosec": int(
                        stamp.get(
                            "nanosec",
                            getattr(
                                getattr(image_header, "stamp", None),
                                "nanosec",
                                0,
                            ),
                        )
                    ),
                },
                "frame_id": frame_id,
            },
            "detections": output_detections,
        }

        out_msg = String()
        out_msg.data = json.dumps(out_payload, ensure_ascii=False)
        self.output_pub.publish(out_msg)

        if self.publish_debug_image:
            self._publish_debug_image(debug_image, image_header)
            self._publish_edges_image(debug_edges, image_header)

    def _process_detection(
        self,
        frame: np.ndarray,
        det: dict[str, Any],
        debug_image: np.ndarray,
        debug_edges: np.ndarray,
    ) -> Optional[dict[str, Any]]:
        h_img, w_img = frame.shape[:2]
        bbox = self._extract_bbox(det)
        if bbox is None:
            self.get_logger().warn(f"Skipping malformed bbox in detection: {det}")
            return None

        bx1, by1, bx2, by2 = self._clamp_bbox(*bbox, w_img=w_img, h_img=h_img)
        if bx2 <= bx1 or by2 <= by1:
            self.get_logger().warn(f"Skipping invalid bbox after clamp: {bbox}")
            return None

        rx1, ry1, rx2, ry2 = self._padded_bbox(bx1, by1, bx2, by2, w_img, h_img)
        roi = frame[ry1:ry2, rx1:rx2]
        if roi.size == 0:
            self.get_logger().warn(
                f"Skipping empty ROI for bbox: {[bx1, by1, bx2, by2]}"
            )
            return None

        (
            line_global,
            yaw_deg,
            yaw_raw_deg,
            best_length,
            edges,
            edge_pixels,
            arrow_angle_deg,
        ) = self._estimate_hough_direction(roi, rx1, ry1)
        debug_edges[ry1:ry2, rx1:rx2] = edges
        if line_global is None:
            self.get_logger().warn(
                "No Hough line found. "
                f"roi_size=({rx2 - rx1},{ry2 - ry1}), edge_pixels={edge_pixels}"
            )
        else:
            self.get_logger().info(
                f"Hough yaw raw={yaw_raw_deg:.2f}, "
                f"yaw_0_90={yaw_deg:.2f}, length={best_length:.1f}"
            )

        cx = int(round((bx1 + bx2) * 0.5))
        cy = int(round((by1 + by2) * 0.5))
        confidence = float(det.get("confidence", 0.0))
        class_name = str(det.get("class_name", self.target_class))

        self._draw_debug(
            debug_image,
            class_name,
            bx1,
            by1,
            bx2,
            by2,
            cx,
            cy,
            yaw_deg,
            line_global,
            arrow_angle_deg,
        )

        return {
            "class_name": class_name,
            "confidence": confidence,
            "bbox": [bx1, by1, bx2, by2],
            "center": [cx, cy],
            "yaw_deg": None if yaw_deg is None else round(float(yaw_deg), 3),
            "line": None if line_global is None else [int(v) for v in line_global],
            "method": "HoughLinesP",
        }

    def _estimate_hough_direction(
        self,
        roi: np.ndarray,
        offset_x: int,
        offset_y: int,
    ) -> tuple[
        Optional[tuple[int, int, int, int]],
        Optional[float],
        Optional[float],
        float,
        np.ndarray,
        int,
        Optional[float],
    ]:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)
        edge_pixels = int(np.count_nonzero(edges))
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        if lines is None or len(lines) == 0:
            return None, None, None, 0.0, edges, edge_pixels, None

        best_line = None
        best_length = -1.0
        for line in lines:
            x1, y1, x2, y2 = [int(v) for v in line.reshape(4)]
            length = math.hypot(float(x2 - x1), float(y2 - y1))
            if length > best_length:
                best_length = length
                best_line = (x1, y1, x2, y2)

        if best_line is None:
            return None, None, None, 0.0, edges, edge_pixels, None

        lx1, ly1, lx2, ly2 = best_line
        yaw_raw_deg = math.degrees(math.atan2(float(ly2 - ly1), float(lx2 - lx1)))
        yaw_deg = normalize_yaw_0_90(yaw_raw_deg)
        arrow_angle_deg = self._stable_arrow_angle_deg(lx1, ly1, lx2, ly2)
        line_global = (
            int(lx1 + offset_x),
            int(ly1 + offset_y),
            int(lx2 + offset_x),
            int(ly2 + offset_y),
        )
        return (
            line_global,
            yaw_deg,
            yaw_raw_deg,
            best_length,
            edges,
            edge_pixels,
            arrow_angle_deg,
        )

    @staticmethod
    def _stable_arrow_angle_deg(x1: int, y1: int, x2: int, y2: int) -> float:
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        if dx < 0.0 or (abs(dx) < 1e-6 and dy < 0.0):
            dx = -dx
            dy = -dy
        return math.degrees(math.atan2(dy, dx))

    def _extract_bbox(
        self,
        det: dict[str, Any],
    ) -> Optional[tuple[float, float, float, float]]:
        bbox = det.get("bbox_xyxy", det.get("bbox"))
        if isinstance(bbox, dict):
            try:
                return (
                    float(bbox["x1"]),
                    float(bbox["y1"]),
                    float(bbox["x2"]),
                    float(bbox["y2"]),
                )
            except (KeyError, TypeError, ValueError):
                return None

        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            try:
                return (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
            except (TypeError, ValueError):
                return None

        return None

    @staticmethod
    def _clamp_bbox(
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        w_img: int,
        h_img: int,
    ) -> tuple[int, int, int, int]:
        ix1 = max(0, min(w_img - 1, int(round(x1))))
        iy1 = max(0, min(h_img - 1, int(round(y1))))
        ix2 = max(0, min(w_img, int(round(x2))))
        iy2 = max(0, min(h_img, int(round(y2))))
        return ix1, iy1, ix2, iy2

    def _padded_bbox(
        self,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        w_img: int,
        h_img: int,
    ) -> tuple[int, int, int, int]:
        pad = max(0, int(self.bbox_padding))
        return (
            max(0, x1 - pad),
            max(0, y1 - pad),
            min(w_img, x2 + pad),
            min(h_img, y2 + pad),
        )

    @staticmethod
    def _frame_id_from_payload(payload: dict[str, Any], image_header: Any) -> str:
        frame_id = str(payload.get("frame_id", ""))
        if frame_id:
            return frame_id
        header = payload.get("header", {})
        if isinstance(header, dict) and header.get("frame_id"):
            return str(header.get("frame_id"))
        if image_header is not None and getattr(image_header, "frame_id", ""):
            return str(image_header.frame_id)
        return "camera_color_optical_frame"

    def _draw_debug(
        self,
        image: np.ndarray,
        class_name: str,
        bx1: int,
        by1: int,
        bx2: int,
        by2: int,
        cx: int,
        cy: int,
        yaw_deg: Optional[float],
        line: Optional[tuple[int, int, int, int]],
        arrow_angle_deg: Optional[float],
    ) -> None:
        cv2.rectangle(image, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 4, (0, 255, 255), -1)

        if line is not None:
            lx1, ly1, lx2, ly2 = line
            cv2.line(image, (lx1, ly1), (lx2, ly2), (0, 0, 255), 2)

        if arrow_angle_deg is not None:
            yaw_rad = math.radians(arrow_angle_deg)
            arrow_x2 = int(cx + self.arrow_length * math.cos(yaw_rad))
            arrow_y2 = int(cy + self.arrow_length * math.sin(yaw_rad))
            cv2.arrowedLine(
                image,
                (int(cx), int(cy)),
                (arrow_x2, arrow_y2),
                (0, 0, 255),
                2,
                tipLength=0.25,
            )

        yaw_text = "null" if yaw_deg is None else f"{yaw_deg:.1f}"
        text = f"{class_name} yaw={yaw_text} deg"
        text_x = bx1
        text_y = max(16, by1 - 8)
        cv2.putText(
            image,
            text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    def _publish_debug_image(self, image: np.ndarray, image_header: Any) -> None:
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            if image_header is not None:
                msg.header = image_header
            self.debug_image_pub.publish(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Debug image publish error: {exc}")

    def _publish_edges_image(self, image: np.ndarray, image_header: Any) -> None:
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
            if image_header is not None:
                msg.header = image_header
            self.debug_edges_pub.publish(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Debug edges publish error: {exc}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloHoughYawEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
