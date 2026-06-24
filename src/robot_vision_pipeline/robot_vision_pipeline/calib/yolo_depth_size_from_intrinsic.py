"""YOLO + aligned depth size debug node using intrinsics from YAML.

Run directly with python after sourcing ROS and the workspace. This file does not
subscribe to a camera info topic and does not require a ros2 run entry point.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


def _default_model_path() -> str:
    try:
        share_dir = Path(get_package_share_directory("robot_vision_pipeline"))
        candidate = share_dir / "models" / "yolov8.pt"
        if candidate.is_file():
            return str(candidate)
    except PackageNotFoundError:
        pass

    source_candidate = Path(__file__).resolve().parents[2] / "models" / "yolov8.pt"
    if source_candidate.is_file():
        return str(source_candidate)

    return ""


class YoloDepthSizeFromIntrinsicNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_depth_size_from_intrinsic")

        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("intrinsic_yaml_path", "")
        self.declare_parameter("debug_image_topic", "/calib/yolo_depth_size_intrinsic_debug_image")
        self.declare_parameter("model_path", _default_model_path())
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("max_det", 20)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("depth_kernel_half_size", 5)
        self.declare_parameter("table_depth_raw_mm", 566.0)
        self.declare_parameter("min_valid_height_mm", 1.0)
        self.declare_parameter("max_valid_height_mm", 300.0)
        self.declare_parameter("box_width_scale", 1.0)
        self.declare_parameter("box_length_scale", 1.0)
        self.declare_parameter("default_size_z_m", 0.04)

        self.bridge = CvBridge()
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_encoding = ""
        self.warned_no_depth = False

        self.color_image_topic = str(self.get_parameter("color_image_topic").value)
        self.depth_image_topic = str(self.get_parameter("depth_image_topic").value)
        self.intrinsic_yaml_path = str(self.get_parameter("intrinsic_yaml_path").value).strip()
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.device = str(self.get_parameter("device").value)
        self.depth_kernel_half_size = int(self.get_parameter("depth_kernel_half_size").value)
        self.table_depth_raw_mm = float(self.get_parameter("table_depth_raw_mm").value)
        self.min_valid_height_mm = float(self.get_parameter("min_valid_height_mm").value)
        self.max_valid_height_mm = float(self.get_parameter("max_valid_height_mm").value)
        self.box_width_scale = float(self.get_parameter("box_width_scale").value)
        self.box_length_scale = float(self.get_parameter("box_length_scale").value)

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self._load_intrinsics(self.intrinsic_yaml_path)

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path or not os.path.isfile(model_path):
            self.get_logger().error(
                "Không tìm thấy YOLO model. Hãy truyền -p model_path:=/duong/dan/toi/best.pt"
            )
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.names = self._model_names(self.model)
        self.get_logger().info(f"YOLO model loaded. Classes: {self.names}")

        self.create_subscription(
            Image,
            self.color_image_topic,
            self._on_color_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_image_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
        )
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.get_logger().info("========================================")
        self.get_logger().info("YoloDepthSizeFromIntrinsicNode started")
        self.get_logger().info(f"Color image topic   : {self.color_image_topic}")
        self.get_logger().info(f"Depth image topic   : {self.depth_image_topic}")
        self.get_logger().info(f"Intrinsic YAML path : {self.intrinsic_yaml_path}")
        self.get_logger().info(f"Debug image topic   : {self.debug_image_topic}")
        self.get_logger().info(f"fx fy cx cy         : {self.fx}, {self.fy}, {self.cx}, {self.cy}")
        self.get_logger().info(
            f"Depth kernel        : {2 * self.depth_kernel_half_size + 1}x"
            f"{2 * self.depth_kernel_half_size + 1}"
        )
        self.get_logger().info(
            f"Size model          : table_depth_raw={self.table_depth_raw_mm:.1f} mm, "
            f"height_range=[{self.min_valid_height_mm:.1f}, {self.max_valid_height_mm:.1f}] mm"
        )
        self.get_logger().info("Output frame        : camera color optical frame")
        self.get_logger().info("========================================")

    @staticmethod
    def _model_names(model: Any) -> dict[int, str]:
        names = getattr(model, "names", {})
        if isinstance(names, dict):
            return {int(k): str(v) for k, v in names.items()}
        if isinstance(names, (list, tuple)):
            return {idx: str(name) for idx, name in enumerate(names)}
        return {}

    def _class_allowed(self, class_id: int) -> bool:
        if len(self.names) <= 1:
            return True
        class_name = self.names.get(class_id, str(class_id)).strip().lower()
        return class_name == "wood"

    def _load_intrinsics(self, yaml_path: str) -> None:
        if not yaml_path:
            self.get_logger().error(
                "intrinsic_yaml_path đang rỗng. Hãy truyền "
                "-p intrinsic_yaml_path:=/duong/dan/toi/Intrinsic.yaml"
            )
            return

        path = Path(yaml_path)
        if not path.is_file():
            self.get_logger().error(f"Không tìm thấy Intrinsic.yaml: {yaml_path}")
            return

        try:
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().error(f"Lỗi đọc Intrinsic.yaml: {exc}")
            return

        matrix = data.get("camera_matrix")
        matrix_data = matrix.get("data") if isinstance(matrix, dict) else None
        if matrix_data is None or len(matrix_data) < 9:
            self.get_logger().error(
                "Intrinsic.yaml thiếu camera_matrix.data hoặc data không đủ 9 phần tử."
            )
            return

        try:
            self.fx = float(matrix_data[0])
            self.fy = float(matrix_data[4])
            self.cx = float(matrix_data[2])
            self.cy = float(matrix_data[5])
        except (TypeError, ValueError) as exc:
            self.fx = self.fy = self.cx = self.cy = None
            self.get_logger().error(f"camera_matrix.data không hợp lệ: {exc}")
            return

        if not self._intrinsics_valid():
            self.get_logger().error("fx/fy/cx/cy trong Intrinsic.yaml không hợp lệ.")

    def _intrinsics_valid(self) -> bool:
        values = (self.fx, self.fy, self.cx, self.cy)
        if any(value is None for value in values):
            return False
        return bool(
            self.fx != 0.0
            and self.fy != 0.0
            and all(math.isfinite(float(value)) for value in values)
        )

    def _on_depth_image(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Không convert được depth image: {exc}")
            return

        self.latest_depth = np.asarray(depth)
        self.latest_depth_encoding = msg.encoding

    def _median_depth_kernel(self, u: int, v: int) -> Optional[float]:
        depth = self.latest_depth
        if depth is None:
            return None

        height, width = depth.shape[:2]
        if u < 0 or v < 0 or u >= width or v >= height:
            return None

        half_size = max(0, self.depth_kernel_half_size)
        x0 = max(0, u - half_size)
        x1 = min(width, u + half_size + 1)
        y0 = max(0, v - half_size)
        y1 = min(height, v + half_size + 1)
        values = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64).reshape(-1)
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size == 0:
            return None

        if depth.dtype == np.uint16 or self.latest_depth_encoding in ("16UC1", "mono16"):
            valid = valid / 1000.0

        valid = valid[np.isfinite(valid) & (valid > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _pixel_to_camera(self, u: int, v: int, z: float) -> Optional[tuple[float, float, float]]:
        if not self._intrinsics_valid():
            return None

        x = (float(u) - float(self.cx)) * z / float(self.fx)
        y = (float(v) - float(self.cy)) * z / float(self.fy)
        return x, y, z

    def _bbox_to_real_size(
        self, bbox_w_px: float, bbox_h_px: float, z: float
    ) -> Optional[tuple[float, float, float]]:
        if not self._intrinsics_valid():
            return None

        # Ported from gp7_vision_pipeline/pixel_to_base_mapper_node.py:
        # width_mm  = bbox_width_px  * depth_m * 1000 / fx
        # length_mm = bbox_height_px * depth_m * 1000 / fy
        # height_mm = table_depth_raw_mm - depth_m * 1000
        depth_mm = z * 1000.0
        height_mm = self.table_depth_raw_mm - depth_mm
        if not (self.min_valid_height_mm <= height_mm <= self.max_valid_height_mm):
            self.get_logger().warning(
                f"object height {height_mm:.1f}mm outside valid range "
                f"[{self.min_valid_height_mm:.1f}, {self.max_valid_height_mm:.1f}]mm, bỏ qua."
            )
            return None

        width_mm = float(bbox_w_px) * depth_mm / float(self.fx)
        length_mm = float(bbox_h_px) * depth_mm / float(self.fy)
        width_mm *= self.box_width_scale
        length_mm *= self.box_length_scale

        size_x_m = width_mm / 1000.0
        size_y_m = length_mm / 1000.0
        size_z_m = height_mm / 1000.0
        return size_x_m, size_y_m, size_z_m

    def _on_color_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Không convert được color image: {exc}")
            return

        debug = frame.copy()

        if not self._intrinsics_valid():
            self.get_logger().warning("Chưa có intrinsic hợp lệ, chưa tính tọa độ/kích thước.")
            self._publish_debug(debug, msg)
            return

        if self.latest_depth is None:
            if not self.warned_no_depth:
                self.warned_no_depth = True
                self.get_logger().warning("Chưa có aligned depth image, chưa tính tọa độ/kích thước.")
            self._publish_debug(debug, msg)
            return

        try:
            results = self.model.predict(
                source=frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                max_det=self.max_det,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO inference lỗi: {exc}")
            self._publish_debug(debug, msg)
            return

        object_id = 0
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                class_id = int(box.cls[0].item()) if box.cls is not None else -1
                class_name = self.names.get(class_id, str(class_id))
                if not self._class_allowed(class_id):
                    continue

                confidence = float(box.conf[0].item()) if box.conf is not None else 0.0
                x1, y1, x2, y2 = [float(x) for x in box.xyxy[0].tolist()]
                u = int(round((x1 + x2) * 0.5))
                v = int(round((y1 + y2) * 0.5))

                depth = self.latest_depth
                if depth is None:
                    continue
                depth_h, depth_w = depth.shape[:2]
                if u < 0 or v < 0 or u >= depth_w or v >= depth_h:
                    self.get_logger().warning(
                        f"object_id={object_id} tâm bbox ({u},{v}) ngoài ảnh depth "
                        f"{depth_w}x{depth_h}, bỏ qua."
                    )
                    continue

                z = self._median_depth_kernel(u, v)
                if z is None:
                    self.get_logger().warning(
                        f"object_id={object_id} depth kernel tại ({u},{v}) không hợp lệ, bỏ qua."
                    )
                    continue

                camera_xyz = self._pixel_to_camera(u, v, z)
                if camera_xyz is None:
                    continue
                x, y, z = camera_xyz

                bbox_w_px = x2 - x1
                bbox_h_px = y2 - y1
                real_size = self._bbox_to_real_size(bbox_w_px, bbox_h_px, z)
                if real_size is None:
                    continue
                size_x, size_y, size_z = real_size

                object_id += 1
                self.get_logger().info(
                    "object_id=%d\n"
                    "class_name=%s\n"
                    "confidence=%.3f\n"
                    "u=%d v=%d\n"
                    "depth_m=%.4f\n"
                    "X=%.4f Y=%.4f Z=%.4f\n"
                    "size_x=%.4f\n"
                    "size_y=%.4f\n"
                    "size_z=%.4f"
                    % (
                        object_id,
                        class_name,
                        confidence,
                        u,
                        v,
                        z,
                        x,
                        y,
                        z,
                        size_x,
                        size_y,
                        size_z,
                    )
                )

                self._draw_detection(
                    debug,
                    x1,
                    y1,
                    x2,
                    y2,
                    u,
                    v,
                    class_name,
                    confidence,
                    x,
                    y,
                    z,
                    size_x,
                    size_y,
                    size_z,
                )

        self._publish_debug(debug, msg)

    def _draw_detection(
        self,
        image: np.ndarray,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        u: int,
        v: int,
        class_name: str,
        confidence: float,
        x: float,
        y: float,
        z: float,
        size_x: float,
        size_y: float,
        size_z: float,
    ) -> None:
        p1 = (int(round(x1)), int(round(y1)))
        p2 = (int(round(x2)), int(round(y2)))
        cv2.rectangle(image, p1, p2, (0, 255, 0), 2)
        cv2.circle(image, (u, v), 4, (0, 0, 255), -1)

        label = f"{class_name} conf={confidence:.2f}"
        coords = f"X={x:.3f} Y={y:.3f} Z={z:.3f} m"
        size_text = f"size={size_x:.3f} x {size_y:.3f} x {size_z:.3f} m"
        cv2.putText(
            image,
            label,
            (p1[0], max(20, p1[1] - 48)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            coords,
            (p1[0], max(42, p1[1] - 28)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            size_text,
            (p1[0], max(64, p1[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    def _publish_debug(self, debug: np.ndarray, source_msg: Image) -> None:
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Không tạo được debug image: {exc}")
            return

        debug_msg.header = source_msg.header
        self.debug_pub.publish(debug_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: Optional[YoloDepthSizeFromIntrinsicNode] = None
    try:
        node = YoloDepthSizeFromIntrinsicNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
