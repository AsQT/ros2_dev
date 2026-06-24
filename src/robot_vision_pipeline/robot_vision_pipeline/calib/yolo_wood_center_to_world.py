#!/usr/bin/env python3

from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO

from robot_vision_pipeline_msgs.msg import Wood, WoodArray


class YoloWoodCenterToWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_wood_center_to_world")

        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_image_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter(
            "model_path",
            "/home/asus/ros_vision/src/robot_vision_pipeline/models/yolov8.pt",
        )
        self.declare_parameter(
            "extrinsic_yaml_path",
            "/home/asus/ros_vision/src/robot_vision_pipeline/config/Extrinsic_camera_to_world.yaml",
        )
        self.declare_parameter("target_class_name", "wood")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("depth_kernel_size", 5)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("output_topic", "/vision/wood_world_objects")
        self.declare_parameter(
            "debug_image_topic",
            "/vision/yolo_wood_world_debug_image",
        )
        self.declare_parameter("publish_debug_image", True)

        self.color_image_topic = str(self.get_parameter("color_image_topic").value)
        self.depth_image_topic = str(self.get_parameter("depth_image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.extrinsic_yaml_path = str(self.get_parameter("extrinsic_yaml_path").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.depth_kernel_size = int(self.get_parameter("depth_kernel_size").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_encoding = ""
        self.T_camera_to_world: Optional[np.ndarray] = None
        self.names: dict[int, str] = {}
        self.last_camera_info_warn = 0.0
        self.last_depth_warn = 0.0
        self.last_invalid_depth_warn = 0.0

        self.T_camera_to_world = self.load_extrinsic_yaml(self.extrinsic_yaml_path)
        self.model = self.load_yolo_model(self.model_path)
        self.names = self.model_names(self.model)

        self.pub_woods = self.create_publisher(WoodArray, self.output_topic, 10)
        self.pub_debug = None
        if self.publish_debug_image:
            self.pub_debug = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_subscription(
            Image,
            self.color_image_topic,
            self.color_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_image_topic,
            self.depth_image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("yolo_wood_center_to_world started")
        self.get_logger().info(f"Color image topic : {self.color_image_topic}")
        self.get_logger().info(f"Depth image topic : {self.depth_image_topic}")
        self.get_logger().info(f"CameraInfo topic  : {self.camera_info_topic}")
        self.get_logger().info(f"Output topic      : {self.output_topic}")
        self.get_logger().info(f"Debug image topic : {self.debug_image_topic}")
        self.get_logger().info(f"World frame       : {self.world_frame}")
        self.get_logger().info(f"YOLO classes      : {self.names}")

    def load_extrinsic_yaml(self, yaml_path: str) -> np.ndarray:
        path = Path(yaml_path).expanduser()
        if not path.is_file():
            self.get_logger().error(
                f"Không đọc được Extrinsic_camera_to_world.yaml: {path}"
            )
            raise FileNotFoundError(f"Extrinsic YAML not found: {path}")

        try:
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().error(f"Lỗi đọc Extrinsic YAML {path}: {exc}")
            raise

        if "T_camera_to_world" not in data:
            self.get_logger().error(
                f"YAML thiếu key T_camera_to_world: {path}"
            )
            raise KeyError("T_camera_to_world")

        matrix = np.array(data["T_camera_to_world"], dtype=np.float64)
        if matrix.shape != (4, 4):
            self.get_logger().error(
                f"T_camera_to_world không phải ma trận 4x4 trong: {path}"
            )
            raise ValueError("T_camera_to_world must be 4x4")

        self.get_logger().info(f"Loaded T_camera_to_world from: {path}")
        return matrix

    def load_yolo_model(self, model_path: str) -> YOLO:
        path = Path(model_path).expanduser()
        if not path.is_file():
            self.get_logger().error(f"YOLO model path sai hoặc không tồn tại: {path}")
            raise FileNotFoundError(f"YOLO model not found: {path}")

        try:
            self.get_logger().info(f"Loading YOLO model: {path}")
            return YOLO(str(path))
        except Exception as exc:
            self.get_logger().error(f"Không load được YOLO model {path}: {exc}")
            raise

    @staticmethod
    def model_names(model: Any) -> dict[int, str]:
        names = getattr(model, "names", {})
        if isinstance(names, dict):
            return {int(key): str(value) for key, value in names.items()}
        if isinstance(names, (list, tuple)):
            return {index: str(value) for index, value in enumerate(names)}
        return {}

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # Lấy K và D trực tiếp từ CameraInfo, không đọc Intrinsic.yaml.
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1)

    def depth_image_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().warning(f"Không convert được aligned depth image: {exc}")
            return

        self.latest_depth = np.asarray(depth)
        self.latest_depth_encoding = msg.encoding

    def color_image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Không convert được color image: {exc}")
            return

        header = msg.header
        annotated = frame.copy()
        output = self.make_empty_output(header)

        if self.camera_matrix is None:
            self.warn_throttled("camera_info", "Waiting for CameraInfo...", 2.0)
            self.publish_outputs(output, annotated, header)
            return

        if self.latest_depth is None:
            self.warn_throttled(
                "depth",
                "Waiting for aligned depth image...",
                2.0,
            )
            self.publish_outputs(output, annotated, header)
            return

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            verbose=False,
        )

        wood_id = 0
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                class_id = int(box.cls[0].item())
                class_name = self.names.get(class_id, str(class_id))
                confidence = float(box.conf[0].item())

                if not self.class_allowed(class_id, class_name):
                    continue

                xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
                x1, y1, x2, y2 = xyxy.tolist()
                u = int(round((x1 + x2) * 0.5))
                v = int(round((y1 + y2) * 0.5))

                depth_m = self.median_depth_m(u, v)
                if depth_m is None:
                    self.warn_throttled(
                        "invalid_depth",
                        f"Depth invalid tại object wood tâm pixel ({u}, {v}), bỏ qua object này.",
                        1.0,
                    )
                    self.draw_detection(
                        annotated,
                        (x1, y1, x2, y2),
                        u,
                        v,
                        class_name,
                        confidence,
                        None,
                    )
                    continue

                camera_point = self.pixel_depth_to_camera(u, v, depth_m)
                world_point = self.camera_to_world(camera_point)

                wood_msg = Wood()
                wood_msg.header.stamp = header.stamp
                wood_msg.header.frame_id = self.world_frame
                wood_msg.wood_id = wood_id
                wood_msg.class_name = self.target_class_name
                wood_msg.confidence = float(confidence)
                wood_msg.pose.position.x = float(world_point[0])
                wood_msg.pose.position.y = float(world_point[1])
                wood_msg.pose.position.z = float(world_point[2])
                wood_msg.pose.orientation.x = 0.0
                wood_msg.pose.orientation.y = 0.0
                wood_msg.pose.orientation.z = 0.0
                wood_msg.pose.orientation.w = 1.0

                output.woods.append(wood_msg)
                wood_id += 1

                self.draw_detection(
                    annotated,
                    (x1, y1, x2, y2),
                    u,
                    v,
                    class_name,
                    confidence,
                    world_point,
                )

        self.publish_outputs(output, annotated, header)

    def make_empty_output(self, header) -> WoodArray:
        output = WoodArray()
        output.header.stamp = header.stamp
        output.header.frame_id = self.world_frame
        output.woods = []
        return output

    def class_allowed(self, class_id: int, class_name: str) -> bool:
        if self.target_class_id >= 0:
            return class_id == self.target_class_id
        return class_name.strip().lower() == self.target_class_name.strip().lower()

    def median_depth_m(self, u: int, v: int) -> Optional[float]:
        # Thử kernel mặc định, rồi mở rộng 9x9, 11x11 nếu cần.
        for kernel_size in self.depth_kernel_candidates():
            depth = self.depth_in_kernel_m(u, v, kernel_size)
            if depth is not None:
                return depth
        return None

    def depth_kernel_candidates(self) -> list[int]:
        base = max(1, int(self.depth_kernel_size))
        if base % 2 == 0:
            base += 1

        candidates = [base, 9, 11]
        unique = []
        for value in candidates:
            if value not in unique:
                unique.append(value)
        return unique

    def depth_in_kernel_m(self, u: int, v: int, kernel_size: int) -> Optional[float]:
        depth = self.latest_depth
        if depth is None:
            return None

        height, width = depth.shape[:2]
        if u < 0 or v < 0 or u >= width or v >= height:
            return None

        half = kernel_size // 2
        x0 = max(0, u - half)
        x1 = min(width, u + half + 1)
        y0 = max(0, v - half)
        y1 = min(height, v + half + 1)

        values = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64).reshape(-1)
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size == 0:
            return None

        if self.latest_depth_encoding == "16UC1" or depth.dtype == np.uint16:
            valid = valid / 1000.0

        valid = valid[np.isfinite(valid) & (valid > 0.0)]
        if valid.size == 0:
            return None

        return float(np.median(valid))

    def pixel_depth_to_camera(self, u: int, v: int, zc: float) -> np.ndarray:
        K = self.camera_matrix
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        xc = (float(u) - cx) * zc / fx
        yc = (float(v) - cy) * zc / fy
        return np.array([xc, yc, zc, 1.0], dtype=np.float64)

    def camera_to_world(self, camera_point: np.ndarray) -> np.ndarray:
        world_point_h = self.T_camera_to_world @ camera_point
        return world_point_h[:3]

    def draw_detection(
        self,
        image: np.ndarray,
        bbox: tuple[float, float, float, float],
        u: int,
        v: int,
        class_name: str,
        confidence: float,
        world_point: Optional[np.ndarray],
    ) -> None:
        if not self.publish_debug_image:
            return

        x1, y1, x2, y2 = [int(round(value)) for value in bbox]
        color = (0, 220, 0) if world_point is not None else (0, 0, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.circle(image, (u, v), 5, (255, 0, 255), -1)

        label = f"{class_name} {confidence:.2f}"
        cv2.putText(
            image,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

        if world_point is None:
            world_text = "W(depth invalid)"
        else:
            world_text = (
                f"W({world_point[0]:.3f}, "
                f"{world_point[1]:.3f}, {world_point[2]:.3f})"
            )

        cv2.putText(
            image,
            world_text,
            (x1, min(image.shape[0] - 10, y2 + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )

    def publish_outputs(self, output: WoodArray, annotated: np.ndarray, header) -> None:
        self.pub_woods.publish(output)

        if self.publish_debug_image and self.pub_debug is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                debug_msg.header = header
                self.pub_debug.publish(debug_msg)
            except CvBridgeError as exc:
                self.get_logger().error(f"Không publish được debug image: {exc}")

    def warn_throttled(self, key: str, text: str, period_sec: float) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        attr = f"last_{key}_warn"
        last = float(getattr(self, attr, 0.0))
        if now - last >= period_sec:
            self.get_logger().warning(text)
            setattr(self, attr, now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloWoodCenterToWorldNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
