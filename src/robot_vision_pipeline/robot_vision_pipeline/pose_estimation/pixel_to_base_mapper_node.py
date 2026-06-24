#!/usr/bin/env python3
"""
Map YOLO BoxDetection (wood / box) from pixel + aligned depth + camera intrinsics
to 3D poses in camera_color_optical_frame.

Scope:
  - Subscribe:
      /vision/wood_detection
      /vision/box_detection
      /camera/camera/color/camera_info
      /camera/camera/aligned_depth_to_color/image_raw
      /camera/camera/color/image_raw

  - Publish:
      /vision/wood_objects
      /vision/box_objects
      /vision/debug_image_camera

Notes:
  - This node publishes poses in camera_color_optical_frame.
  - Camera -> base_link transform is NOT implemented here.
  - Homography is NOT used.
  - Wood yaw is read from /vision/yolo/hough_yaw_json and stored in pose.orientation.
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from robot_vision_pipeline.depth_utils import robust_center_depth
from robot_vision_pipeline_msgs.msg import Box, BoxArray, BoxDetection, Wood, WoodArray


def pixel_to_camera_xyz(
    u: float,
    v: float,
    z_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """
    Convert pixel coordinate + depth to 3D camera optical frame.

    Xc = (u - cx) * Z / fx
    Yc = (v - cy) * Z / fy
    Zc = Z
    """
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"Invalid camera intrinsics: fx={fx}, fy={fy}")

    x_c = (float(u) - float(cx)) * float(z_m) / float(fx)
    y_c = (float(v) - float(cy)) * float(z_m) / float(fy)
    z_c = float(z_m)
    return float(x_c), float(y_c), float(z_c)


def fallback_depth_from_detection(det: BoxDetection) -> Optional[float]:
    """
    Fallback depth from BoxDetection fields.

    Priority:
      1. distance_m
      2. roi_median_raw_depth in mm
      3. center_raw_depth in mm
    """
    if float(det.distance_m) > 0.0:
        return float(det.distance_m)

    if int(det.roi_median_raw_depth) > 0:
        return float(det.roi_median_raw_depth) * 0.001

    if int(det.center_raw_depth) > 0:
        return float(det.center_raw_depth) * 0.001

    return None


def compute_box_size_from_bbox(
    bbox_w_px: float,
    bbox_h_px: float,
    depth_m: float,
    fx: float,
    fy: float,
    default_x_m: float,
    default_y_m: float,
    default_z_m: float,
) -> Tuple[float, float, float]:
    """
    Estimate box size from bbox pixels and depth.

    size.x = bbox_w_px * depth / fx
    size.y = bbox_h_px * depth / fy
    size.z = default_z_m
    """
    if depth_m <= 0.0 or fx <= 0.0 or fy <= 0.0:
        return default_x_m, default_y_m, default_z_m

    size_x = float(bbox_w_px) * float(depth_m) / float(fx)
    size_y = float(bbox_h_px) * float(depth_m) / float(fy)

    if size_x <= 0.0 or not math.isfinite(size_x):
        size_x = default_x_m
    if size_y <= 0.0 or not math.isfinite(size_y):
        size_y = default_y_m

    return float(size_x), float(size_y), float(default_z_m)


def normalize_axis_deg(angle_deg: float) -> float:
    return float((float(angle_deg) + 90.0) % 180.0 - 90.0)


def yaw_to_quaternion_z(yaw_deg: float) -> Tuple[float, float, float, float]:
    yaw_rad = math.radians(float(yaw_deg))
    return (
        0.0,
        0.0,
        math.sin(yaw_rad * 0.5),
        math.cos(yaw_rad * 0.5),
    )


@dataclass
class HoughYawData:
    object_id: int
    class_name: str
    confidence: float
    center_u: float
    center_v: float
    yaw_deg: float
    yaw_valid: bool
    method: str
    arrow_valid: bool
    arrow_start_u: float
    arrow_start_v: float
    arrow_end_u: float
    arrow_end_v: float
    stamp_sec: float


@dataclass
class WoodData:
    wood_id: int
    class_name: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    center_x: int
    center_y: int
    x_c_m: float
    y_c_m: float
    z_c_m: float
    x_out_m: float
    y_out_m: float
    z_out_m: float
    yaw_deg: float
    yaw_valid: bool
    yaw_method: str
    arrow_valid: bool
    arrow_start_u: float
    arrow_start_v: float
    arrow_end_u: float
    arrow_end_v: float
    stamp_sec: float


@dataclass
class BoxData:
    box_id: int
    class_name: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int
    center_x: int
    center_y: int
    x_c_m: float
    y_c_m: float
    z_c_m: float
    x_out_m: float
    y_out_m: float
    z_out_m: float
    size_x_m: float
    size_y_m: float
    size_z_m: float
    area_m2: float
    area_cm2: float
    safe_area_m2: float
    safe_area_cm2: float
    stamp_sec: float


class PixelToBaseMapperNode(Node):
    """
    Convert wood / box pixel detections to camera-frame 3D objects.

    Output frame:
      camera_color_optical_frame by default.
    """

    def __init__(self) -> None:
        super().__init__("pixel_to_base_mapper_node")

        self._bridge = CvBridge()

        self._color_lock = threading.Lock()
        self._latest_color_bgr: Optional[np.ndarray] = None

        self._depth_lock = threading.Lock()
        self._latest_depth_msg: Optional[Image] = None
        self._latest_depth_encoding: str = ""

        self._camera_info_lock = threading.Lock()
        self._camera_intrinsics: Optional[Tuple[float, float, float, float]] = None

        self._data_lock = threading.Lock()
        self._latest_woods: Dict[int, WoodData] = {}
        self._latest_boxes: Dict[int, BoxData] = {}

        self._yaw_lock = threading.Lock()
        self._latest_hough_yaws: Dict[int, HoughYawData] = {}
        self._debug_center_by_wood_id: Dict[int, Tuple[float, float]] = {}

        self._warned_no_intrinsics_wood = False
        self._warned_no_intrinsics_box = False
        self._warned_no_depth = False

        # ----------------------------- parameters
        self.declare_parameter("output_frame_id", "camera_color_optical_frame")
        self.declare_parameter("use_world_transform", False)
        self.declare_parameter("world_frame_id", "aruco_world")
        self.declare_parameter("extrinsic_yaml_path", "")

        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw")

        self.declare_parameter("wood_detection_topic", "/vision/wood_detection")
        self.declare_parameter("box_detection_topic", "/vision/box_detection")
        self.declare_parameter("hough_yaw_topic", "/vision/yolo/hough_yaw_json")
        self.declare_parameter("hough_yaw_json_topic", "/vision/yolo/hough_yaw_json")

        self.declare_parameter("wood_objects_topic", "/vision/wood_objects")
        self.declare_parameter("box_objects_topic", "/vision/box_objects")
        self.declare_parameter("debug_image_topic", "/vision/debug_image_camera")

        self.declare_parameter("depth_kernel_radius", 2)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("depth_outlier_threshold_m", 0.02)
        self.declare_parameter("min_valid_depth_samples", 3)

        self.declare_parameter("default_box_size_x_m", 0.08)
        self.declare_parameter("default_box_size_y_m", 0.08)
        self.declare_parameter("default_box_size_z_m", 0.05)
        self.declare_parameter("box_obstacle_margin_m", 0.02)

        self.declare_parameter("overlay_timeout_sec", 1.0)
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("publish_period_sec", 0.5)

        self.declare_parameter("use_hough_yaw", True)
        self.declare_parameter("use_hough_yaw_for_wood", True)
        self.declare_parameter("yaw_match_max_dist_px", 40.0)
        self.declare_parameter("yaw_match_max_center_dist_px", 40.0)
        self.declare_parameter("yaw_max_age_sec", 0.5)
        self.declare_parameter("yaw_stale_timeout_sec", 0.5)
        self.declare_parameter("draw_wood_yaw_arrow", True)
        self.declare_parameter("yaw_arrow_length_px", 55)
        self.declare_parameter("use_yaw_arrow_from_hough_json", True)
        self.declare_parameter("debug_center_alpha", 0.2)
        self.declare_parameter("draw_yolo_roi", True)
        self.declare_parameter("roi_x", 125)
        self.declare_parameter("roi_y", 150)
        self.declare_parameter("roi_width", 360)
        self.declare_parameter("roi_height", 260)

        self._output_frame_id = str(self.get_parameter("output_frame_id").value)
        self._use_world_transform = bool(self.get_parameter("use_world_transform").value)
        self._world_frame_id = str(self.get_parameter("world_frame_id").value)
        self._extrinsic_yaml_path = str(self.get_parameter("extrinsic_yaml_path").value)
        self._T_world_camera: Optional[np.ndarray] = None
        if self._use_world_transform:
            self._T_world_camera = self._load_t_world_camera(self._extrinsic_yaml_path)
            if self._T_world_camera is None:
                self.get_logger().warn(
                    "use_world_transform=true but T_world_camera was not loaded; "
                    "falling back to camera coordinates."
                )

        self._camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._color_image_topic = str(self.get_parameter("color_image_topic").value)
        self._depth_image_topic = str(self.get_parameter("depth_image_topic").value)

        self._wood_detection_topic = str(self.get_parameter("wood_detection_topic").value)
        self._box_detection_topic = str(self.get_parameter("box_detection_topic").value)
        self._hough_yaw_json_topic = str(self.get_parameter("hough_yaw_topic").value)
        legacy_hough_yaw_topic = str(
            self.get_parameter("hough_yaw_json_topic").value
        )
        if legacy_hough_yaw_topic:
            self._hough_yaw_json_topic = legacy_hough_yaw_topic

        self._wood_objects_topic = str(self.get_parameter("wood_objects_topic").value)
        self._box_objects_topic = str(self.get_parameter("box_objects_topic").value)
        self._debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self._depth_kernel_radius = max(0, int(self.get_parameter("depth_kernel_radius").value))
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._depth_outlier_threshold_m = float(
            self.get_parameter("depth_outlier_threshold_m").value
        )
        self._min_valid_depth_samples = max(
            1, int(self.get_parameter("min_valid_depth_samples").value)
        )

        self._default_box_x = float(self.get_parameter("default_box_size_x_m").value)
        self._default_box_y = float(self.get_parameter("default_box_size_y_m").value)
        self._default_box_z = float(self.get_parameter("default_box_size_z_m").value)
        self._box_obstacle_margin_m = max(
            0.0,
            float(self.get_parameter("box_obstacle_margin_m").value),
        )

        self._overlay_timeout_sec = float(self.get_parameter("overlay_timeout_sec").value)
        self._stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self._publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        self._use_hough_yaw_for_wood = bool(
            self.get_parameter("use_hough_yaw").value
        ) and bool(self.get_parameter("use_hough_yaw_for_wood").value)
        self._yaw_match_max_center_dist_px = float(
            self.get_parameter("yaw_match_max_dist_px").value
        )
        legacy_match_dist = float(
            self.get_parameter("yaw_match_max_center_dist_px").value
        )
        if legacy_match_dist > 0.0:
            self._yaw_match_max_center_dist_px = legacy_match_dist
        self._yaw_stale_timeout_sec = float(self.get_parameter("yaw_max_age_sec").value)
        legacy_stale_timeout = float(self.get_parameter("yaw_stale_timeout_sec").value)
        if legacy_stale_timeout > 0.0:
            self._yaw_stale_timeout_sec = legacy_stale_timeout
        self._draw_wood_yaw_arrow = bool(self.get_parameter("draw_wood_yaw_arrow").value)
        self._yaw_arrow_length_px = int(self.get_parameter("yaw_arrow_length_px").value)
        self._use_yaw_arrow_from_hough_json = bool(
            self.get_parameter("use_yaw_arrow_from_hough_json").value
        )
        self._debug_center_alpha = float(self.get_parameter("debug_center_alpha").value)
        self._draw_yolo_roi = bool(self.get_parameter("draw_yolo_roi").value)
        self._roi_x = int(self.get_parameter("roi_x").value)
        self._roi_y = int(self.get_parameter("roi_y").value)
        self._roi_width = int(self.get_parameter("roi_width").value)
        self._roi_height = int(self.get_parameter("roi_height").value)

        # ----------------------------- QoS
        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----------------------------- publishers
        self._pub_wood_objects = self.create_publisher(
            WoodArray,
            self._wood_objects_topic,
            det_qos,
        )
        self._pub_box_objects = self.create_publisher(
            BoxArray,
            self._box_objects_topic,
            det_qos,
        )
        self._pub_debug = self.create_publisher(
            Image,
            self._debug_image_topic,
            debug_qos,
        )

        # ----------------------------- subscriptions
        self.create_subscription(
            BoxDetection,
            self._wood_detection_topic,
            self._on_wood_detection,
            det_qos,
        )
        self.create_subscription(
            BoxDetection,
            self._box_detection_topic,
            self._on_box_detection,
            det_qos,
        )
        self.create_subscription(
            String,
            self._hough_yaw_json_topic,
            self._on_hough_yaw_json,
            det_qos,
        )
        self.create_subscription(
            Image,
            self._color_image_topic,
            self._on_color_image,
            img_qos,
        )
        self.create_subscription(
            Image,
            self._depth_image_topic,
            self._on_depth_image,
            img_qos,
        )
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._on_camera_info,
            QoSProfile(depth=1),
        )

        self._publish_timer = self.create_timer(
            self._publish_period_sec,
            self._publish_objects,
        )

        self.get_logger().info(
            "pixel_to_base_mapper_node started | "
            f"output_frame={self._effective_output_frame_id()}, "
            f"use_world_transform={self._world_transform_active()}, "
            f"camera_info={self._camera_info_topic}, "
            f"color={self._color_image_topic}, "
            f"depth={self._depth_image_topic}, "
            f"wood_in={self._wood_detection_topic}, "
            f"box_in={self._box_detection_topic}, "
            f"hough_yaw={self._hough_yaw_json_topic}, "
            f"wood_out={self._wood_objects_topic}, "
            f"box_out={self._box_objects_topic}, "
            f"debug={self._debug_image_topic}"
        )

    # ------------------------------------------------------------------ common
    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _world_transform_active(self) -> bool:
        return self._use_world_transform and self._T_world_camera is not None

    def _effective_output_frame_id(self) -> str:
        if self._world_transform_active():
            return self._world_frame_id or "aruco_world"
        return self._output_frame_id

    def _load_t_world_camera(self, yaml_path: str) -> Optional[np.ndarray]:
        path = str(yaml_path).strip()
        if not path:
            self.get_logger().warn("extrinsic_yaml_path is empty.")
            return None

        try:
            with open(path, "r", encoding="utf-8") as file:
                data = yaml.safe_load(file) or {}
        except Exception as exc:
            self.get_logger().warn(f"Failed to read extrinsic YAML '{path}': {exc}")
            return None

        matrix = None
        extrinsic_matrix = data.get("extrinsic_matrix", {})
        if isinstance(extrinsic_matrix, dict):
            matrix = extrinsic_matrix.get("T_world_camera")
        if matrix is None:
            matrix = data.get("T_world_camera")
        if matrix is None:
            self.get_logger().warn(f"No T_world_camera found in '{path}'.")
            return None

        transform = np.asarray(matrix, dtype=np.float64)
        if transform.shape != (4, 4):
            self.get_logger().warn(
                f"T_world_camera in '{path}' has shape {transform.shape}, expected (4, 4)."
            )
            return None

        world_frame = str(data.get("world_frame", "")).strip()
        if world_frame:
            self._world_frame_id = world_frame

        self.get_logger().info(
            f"Loaded T_world_camera from {path}; world_frame={self._world_frame_id}"
        )
        return transform

    def transform_camera_to_world(
        self,
        x_c_m: float,
        y_c_m: float,
        z_c_m: float,
    ) -> Tuple[float, float, float]:
        if not self._world_transform_active():
            return float(x_c_m), float(y_c_m), float(z_c_m)

        p_camera = np.array(
            [float(x_c_m), float(y_c_m), float(z_c_m), 1.0],
            dtype=np.float64,
        )
        p_world = self._T_world_camera @ p_camera
        return float(p_world[0]), float(p_world[1]), float(p_world[2])

    @staticmethod
    def _object_label(
        class_name: str,
        confidence: float,
        x_m: float,
        y_m: float,
        z_m: float,
    ) -> str:
        return (
            f"{class_name} {float(confidence):.2f} "
            f"({float(x_m):.3f},{float(y_m):.3f},{float(z_m):.3f})"
        )

    def _on_hough_yaw_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid hough yaw json: {exc}")
            return

        objects = payload.get("detections", payload.get("objects", []))
        if not isinstance(objects, list):
            return

        now = self._now_seconds()
        parsed: Dict[int, HoughYawData] = {}

        for obj in objects:
            try:
                class_name = str(obj.get("class_name", "")).strip().lower()
                if class_name != "wood":
                    continue

                yaw_value = obj.get("yaw_deg", None)
                if yaw_value is None:
                    continue
                yaw_deg = float(yaw_value)
                if not math.isfinite(yaw_deg):
                    continue

                yaw_valid = bool(obj.get("yaw_valid", True))
                if not yaw_valid:
                    continue

                center_u, center_v = self._center_from_hough_object(obj)
                arrow_valid, arrow_start_u, arrow_start_v, arrow_end_u, arrow_end_v = (
                    self._arrow_from_hough_object(obj, center_u, center_v)
                )

                object_id = int(obj.get("id", len(parsed)))
                confidence = float(obj.get("confidence", 0.0))
                method = str(obj.get("method", ""))

                parsed[object_id] = HoughYawData(
                    object_id=object_id,
                    class_name=class_name,
                    confidence=confidence,
                    center_u=center_u,
                    center_v=center_v,
                    yaw_deg=float(yaw_deg),
                    yaw_valid=True,
                    method=method,
                    arrow_valid=arrow_valid,
                    arrow_start_u=arrow_start_u,
                    arrow_start_v=arrow_start_v,
                    arrow_end_u=arrow_end_u,
                    arrow_end_v=arrow_end_v,
                    stamp_sec=now,
                )
            except Exception as exc:
                self.get_logger().warn(f"Skip malformed hough yaw object: {exc}")
                continue

        with self._yaw_lock:
            self._latest_hough_yaws = parsed

    @staticmethod
    def _center_from_hough_object(obj: dict) -> Tuple[float, float]:
        center = obj.get("center", None)
        if isinstance(center, (list, tuple)) and len(center) >= 2:
            return float(center[0]), float(center[1])

        refined_center = obj.get("refined_center", {})
        if isinstance(refined_center, dict):
            return (
                float(refined_center.get("u", 0.0)),
                float(refined_center.get("v", 0.0)),
            )

        bbox = obj.get("bbox", None)
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
            return (
                0.5 * (float(bbox[0]) + float(bbox[2])),
                0.5 * (float(bbox[1]) + float(bbox[3])),
            )

        bbox_xyxy = obj.get("bbox_xyxy", {})
        if isinstance(bbox_xyxy, dict):
            return (
                0.5 * (float(bbox_xyxy.get("x1", 0.0)) + float(bbox_xyxy.get("x2", 0.0))),
                0.5 * (float(bbox_xyxy.get("y1", 0.0)) + float(bbox_xyxy.get("y2", 0.0))),
            )

        return 0.0, 0.0

    @staticmethod
    def _arrow_from_hough_object(
        obj: dict,
        center_u: float,
        center_v: float,
    ) -> Tuple[bool, float, float, float, float]:
        yaw_arrow = obj.get("yaw_arrow", {})
        if isinstance(yaw_arrow, dict) and bool(yaw_arrow.get("valid", False)):
            return (
                True,
                float(yaw_arrow.get("start_u", center_u)),
                float(yaw_arrow.get("start_v", center_v)),
                float(yaw_arrow.get("end_u", center_u)),
                float(yaw_arrow.get("end_v", center_v)),
            )

        line = obj.get("line", None)
        if isinstance(line, (list, tuple)) and len(line) >= 4:
            x1 = float(line[0])
            y1 = float(line[1])
            x2 = float(line[2])
            y2 = float(line[3])
            dx = x2 - x1
            dy = y2 - y1
            if dx < 0.0 or (abs(dx) < 1e-6 and dy < 0.0):
                dx = -dx
                dy = -dy
            length = math.hypot(dx, dy)
            if length > 1e-6:
                scale = 55.0 / length
                return (
                    True,
                    float(center_u),
                    float(center_v),
                    float(center_u + dx * scale),
                    float(center_v + dy * scale),
                )

        return False, float(center_u), float(center_v), float(center_u), float(center_v)

    def _find_matching_hough_yaw(
        self,
        center_u: float,
        center_v: float,
    ) -> Optional[HoughYawData]:
        if not self._use_hough_yaw_for_wood:
            return None

        now = self._now_seconds()
        with self._yaw_lock:
            yaws = list(self._latest_hough_yaws.values())

        best = None
        best_dist = float("inf")
        for item in yaws:
            if not item.yaw_valid:
                continue
            if (now - item.stamp_sec) > self._yaw_stale_timeout_sec:
                continue

            du = float(center_u) - float(item.center_u)
            dv = float(center_v) - float(item.center_v)
            dist = math.hypot(du, dv)
            if dist < best_dist:
                best_dist = dist
                best = item

        if best is None:
            return None
        if best_dist > self._yaw_match_max_center_dist_px:
            return None
        return best

    # ------------------------------------------------------------------ camera info
    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warn(
                f"Invalid CameraInfo intrinsics: fx={fx:.3f}, fy={fy:.3f}"
            )
            return

        with self._camera_info_lock:
            first_load = self._camera_intrinsics is None
            self._camera_intrinsics = (fx, fy, cx, cy)

        if first_load:
            self.get_logger().info(
                f"Camera intrinsics loaded: fx={fx:.3f}, fy={fy:.3f}, "
                f"cx={cx:.3f}, cy={cy:.3f}"
            )

    def _get_intrinsics(self) -> Optional[Tuple[float, float, float, float]]:
        with self._camera_info_lock:
            return self._camera_intrinsics

    # ------------------------------------------------------------------ depth
    def _on_depth_image(self, msg: Image) -> None:
        with self._depth_lock:
            self._latest_depth_msg = msg
            self._latest_depth_encoding = msg.encoding

    def _depth_at_center(self, u: int, v: int) -> Optional[float]:
        with self._depth_lock:
            depth_msg = self._latest_depth_msg
            depth_encoding = self._latest_depth_encoding

        if depth_msg is None:
            if not self._warned_no_depth:
                self.get_logger().warn(
                    f"No depth image yet. Waiting for {self._depth_image_topic}"
                )
                self._warned_no_depth = True
            return None

        try:
            depth_arr = self._bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f"Depth cv_bridge error: {exc}")
            return None

        if (
            depth_arr is None
            or not isinstance(depth_arr, np.ndarray)
            or depth_arr.ndim != 2
        ):
            self.get_logger().warn("Invalid depth image array.")
            return None

        h, w = depth_arr.shape[:2]
        u_clamped = int(max(0, min(int(u), w - 1)))
        v_clamped = int(max(0, min(int(v), h - 1)))

        depth_m, _, _, _, _ = robust_center_depth(
            depth_arr,
            u_clamped,
            v_clamped,
            radius=self._depth_kernel_radius,
            encoding=depth_encoding,
            min_depth_m=self._min_depth_m,
            max_depth_m=self._max_depth_m,
            outlier_threshold_m=self._depth_outlier_threshold_m,
            min_valid_samples=self._min_valid_depth_samples,
        )

        if depth_m is None:
            return None

        depth_m = float(depth_m)
        if not math.isfinite(depth_m) or depth_m <= 0.0:
            return None

        return depth_m

    def _resolve_depth(self, det: BoxDetection, center_u: int, center_v: int) -> Optional[float]:
        """
        Depth priority:
          1. aligned depth image around bbox center
          2. depth fields already available in BoxDetection
        """
        depth_from_image = self._depth_at_center(center_u, center_v)
        if depth_from_image is not None and depth_from_image > 0.0:
            return depth_from_image

        return fallback_depth_from_detection(det)

    # ------------------------------------------------------------------ image debug
    def _on_color_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"Color cv_bridge error: {exc}")
            return

        with self._color_lock:
            self._latest_color_bgr = bgr.copy()

        now = self._now_seconds()
        with self._data_lock:
            woods = [
                w for w in self._latest_woods.values()
                if (now - w.stamp_sec) <= self._overlay_timeout_sec
            ]
            boxes = [
                b for b in self._latest_boxes.values()
                if (now - b.stamp_sec) <= self._overlay_timeout_sec
            ]

        debug = bgr.copy()
        h_img, w_img = debug.shape[:2]
        self._draw_yolo_roi_overlay(debug, w_img, h_img)

        for wood in woods:
            wood_label = self._object_label(
                wood.class_name,
                wood.confidence,
                wood.x_out_m,
                wood.y_out_m,
                wood.z_out_m,
            )

            self._draw_object_debug(
                debug,
                x_min=wood.x_min,
                y_min=wood.y_min,
                x_max=wood.x_max,
                y_max=wood.y_max,
                center_x=wood.center_x,
                center_y=wood.center_y,
                label=wood_label,
                color=(0, 200, 0),
                image_w=w_img,
                image_h=h_img,
            )
            if self._draw_wood_yaw_arrow and wood.yaw_valid:
                if self._use_yaw_arrow_from_hough_json and wood.arrow_valid:
                    self._draw_yaw_arrow_from_points(
                        debug,
                        start_u=wood.arrow_start_u,
                        start_v=wood.arrow_start_v,
                        end_u=wood.arrow_end_u,
                        end_v=wood.arrow_end_v,
                        image_w=w_img,
                        image_h=h_img,
                    )
                else:
                    arrow_cx, arrow_cy = self._smooth_debug_center(
                        wood.wood_id,
                        float(wood.center_x),
                        float(wood.center_y),
                    )
                    self._draw_yaw_arrow_from_yaw(
                        debug,
                        center_x=arrow_cx,
                        center_y=arrow_cy,
                        yaw_deg=wood.yaw_deg,
                        length_px=self._yaw_arrow_length_px,
                        image_w=w_img,
                        image_h=h_img,
                    )

        for box in boxes:
            box_label = (
                f"{box.class_name} {box.confidence:.2f} "
                f"L={box.size_y_m * 100.0:.1f}cm "
                f"W={box.size_x_m * 100.0:.1f}cm"
            )

            self._draw_object_debug(
                debug,
                x_min=box.x_min,
                y_min=box.y_min,
                x_max=box.x_max,
                y_max=box.y_max,
                center_x=box.center_x,
                center_y=box.center_y,
                label=box_label,
                color=(0, 255, 255),
                image_w=w_img,
                image_h=h_img,
            )

        try:
            out_msg = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            out_msg.header = msg.header
            self._pub_debug.publish(out_msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Debug cv_bridge error: {exc}")

    def _draw_yolo_roi_overlay(self, image: np.ndarray, image_w: int, image_h: int) -> None:
        if not self._draw_yolo_roi:
            return
        x1 = max(0, min(int(self._roi_x), image_w - 1))
        y1 = max(0, min(int(self._roi_y), image_h - 1))
        x2 = max(0, min(int(self._roi_x + self._roi_width), image_w - 1))
        y2 = max(0, min(int(self._roi_y + self._roi_height), image_h - 1))
        if x2 <= x1 or y2 <= y1:
            return
        color = (255, 0, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image,
            "YOLO ROI",
            (x1 + 5, min(image_h - 5, y1 + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    def _smooth_debug_center(
        self,
        object_id: int,
        center_x: float,
        center_y: float,
    ) -> Tuple[float, float]:
        alpha = max(0.0, min(1.0, float(self._debug_center_alpha)))
        key = int(object_id)
        prev = self._debug_center_by_wood_id.get(key)
        if prev is None:
            smoothed = (float(center_x), float(center_y))
        else:
            smoothed = (
                (1.0 - alpha) * float(prev[0]) + alpha * float(center_x),
                (1.0 - alpha) * float(prev[1]) + alpha * float(center_y),
            )
        self._debug_center_by_wood_id[key] = smoothed
        return smoothed

    @staticmethod
    def _draw_object_debug(
        image: np.ndarray,
        x_min: int,
        y_min: int,
        x_max: int,
        y_max: int,
        center_x: int,
        center_y: int,
        label: str,
        color: Tuple[int, int, int],
        image_w: int,
        image_h: int,
    ) -> None:
        x1 = max(0, min(int(x_min), image_w - 1))
        y1 = max(0, min(int(y_min), image_h - 1))
        x2 = max(0, min(int(x_max), image_w - 1))
        y2 = max(0, min(int(y_max), image_h - 1))

        cx = max(0, min(int(center_x), image_w - 1))
        cy = max(0, min(int(center_y), image_h - 1))

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.circle(image, (cx, cy), 5, color, -1)
        cv2.circle(image, (cx, cy), 6, (255, 255, 255), 1)

        text_x = x1 + 4
        text_y = max(18, y1 - 8)
        if label:
            lines = str(label).splitlines()
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.45
            thickness = 1
            line_step = 18
            line_sizes = [
                cv2.getTextSize(line, font, font_scale, thickness)[0]
                for line in lines
            ]
            max_text_w = max((size[0] for size in line_sizes), default=0)
            max_text_h = max((size[1] for size in line_sizes), default=12)
            text_x = max(2, min(text_x, image_w - max_text_w - 2))
            text_y = max(max_text_h + 2, text_y)
            bottom_y = text_y + (len(lines) - 1) * line_step + 4
            if bottom_y >= image_h:
                text_y = max(
                    max_text_h + 2,
                    image_h - 4 - (len(lines) - 1) * line_step,
                )

            for index, line in enumerate(lines):
                cv2.putText(
                    image,
                    line,
                    (text_x, text_y + index * line_step),
                    font,
                    font_scale,
                    color,
                    thickness,
                    cv2.LINE_AA,
                )

    @staticmethod
    def _draw_yaw_arrow_from_points(
        image: np.ndarray,
        start_u: float,
        start_v: float,
        end_u: float,
        end_v: float,
        image_w: int,
        image_h: int,
    ) -> None:
        sx = max(0, min(int(round(start_u)), image_w - 1))
        sy = max(0, min(int(round(start_v)), image_h - 1))
        ex = max(0, min(int(round(end_u)), image_w - 1))
        ey = max(0, min(int(round(end_v)), image_h - 1))

        cv2.arrowedLine(
            image,
            (sx, sy),
            (ex, ey),
            (0, 0, 255),
            3,
            tipLength=0.25,
        )
        cv2.circle(image, (sx, sy), 4, (0, 0, 255), -1)

    @staticmethod
    def _draw_yaw_arrow_from_yaw(
        image: np.ndarray,
        center_x: int,
        center_y: int,
        yaw_deg: float,
        length_px: int,
        image_w: int,
        image_h: int,
    ) -> None:
        cx = max(0, min(int(center_x), image_w - 1))
        cy = max(0, min(int(center_y), image_h - 1))

        yaw = normalize_axis_deg(float(yaw_deg))
        theta = math.radians(yaw)

        dx = math.cos(theta)
        dy = math.sin(theta)
        if dx < 0.0:
            dx = -dx
            dy = -dy

        length = max(20, int(length_px))
        ex = int(round(cx + length * dx))
        ey = int(round(cy + length * dy))

        ex = max(0, min(ex, image_w - 1))
        ey = max(0, min(ey, image_h - 1))

        cv2.arrowedLine(
            image,
            (cx, cy),
            (ex, ey),
            (0, 0, 255),
            3,
            tipLength=0.25,
        )
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)

    # ------------------------------------------------------------------ detections
    def _on_wood_detection(self, msg: BoxDetection) -> None:
        intrinsics = self._get_intrinsics()
        if intrinsics is None:
            if not self._warned_no_intrinsics_wood:
                self.get_logger().warn(
                    f"No camera intrinsics yet; skipping wood. "
                    f"Waiting for {self._camera_info_topic}"
                )
                self._warned_no_intrinsics_wood = True
            return

        fx, fy, cx_i, cy_i = intrinsics

        center_u = int(msg.center_x)
        center_v = int(msg.center_y)

        depth_m = self._resolve_depth(msg, center_u, center_v)
        if depth_m is None or depth_m <= 0.0:
            self.get_logger().warn(
                f"Wood depth invalid; skip id={int(msg.object_id)} "
                f"center=({center_u},{center_v})"
            )
            return

        try:
            x_c, y_c, z_c = pixel_to_camera_xyz(
                center_u,
                center_v,
                depth_m,
                fx,
                fy,
                cx_i,
                cy_i,
            )
        except ValueError as exc:
            self.get_logger().warn(f"pixel_to_camera_xyz failed for wood: {exc}")
            return

        now = self._now_seconds()
        class_name = str(msg.class_name) if str(msg.class_name) else "wood"
        x_out, y_out, z_out = self.transform_camera_to_world(x_c, y_c, z_c)

        yaw_data = self._find_matching_hough_yaw(center_u, center_v)
        if yaw_data is not None:
            yaw_deg = float(yaw_data.yaw_deg)
            yaw_valid = True
            yaw_method = yaw_data.method
            arrow_valid = bool(yaw_data.arrow_valid)
            arrow_start_u = float(yaw_data.arrow_start_u)
            arrow_start_v = float(yaw_data.arrow_start_v)
            arrow_end_u = float(yaw_data.arrow_end_u)
            arrow_end_v = float(yaw_data.arrow_end_v)
        else:
            self.get_logger().warn(
                "No Hough yaw matched for wood object, use identity orientation."
            )
            yaw_deg = 0.0
            yaw_valid = False
            yaw_method = "none"
            arrow_valid = False
            arrow_start_u = float(center_u)
            arrow_start_v = float(center_v)
            arrow_end_u = float(center_u)
            arrow_end_v = float(center_v)

        with self._data_lock:
            self._latest_woods[int(msg.object_id)] = WoodData(
                wood_id=int(msg.object_id),
                class_name=class_name,
                confidence=float(msg.confidence),
                x_min=int(msg.x_min),
                y_min=int(msg.y_min),
                x_max=int(msg.x_max),
                y_max=int(msg.y_max),
                center_x=center_u,
                center_y=center_v,
                x_c_m=x_c,
                y_c_m=y_c,
                z_c_m=z_c,
                x_out_m=x_out,
                y_out_m=y_out,
                z_out_m=z_out,
                yaw_deg=yaw_deg,
                yaw_valid=yaw_valid,
                yaw_method=yaw_method,
                arrow_valid=arrow_valid,
                arrow_start_u=arrow_start_u,
                arrow_start_v=arrow_start_v,
                arrow_end_u=arrow_end_u,
                arrow_end_v=arrow_end_v,
                stamp_sec=now,
            )

    def _on_box_detection(self, msg: BoxDetection) -> None:
        intrinsics = self._get_intrinsics()
        if intrinsics is None:
            if not self._warned_no_intrinsics_box:
                self.get_logger().warn(
                    f"No camera intrinsics yet; skipping box. "
                    f"Waiting for {self._camera_info_topic}"
                )
                self._warned_no_intrinsics_box = True
            return

        fx, fy, cx_i, cy_i = intrinsics

        center_u = int(msg.center_x)
        center_v = int(msg.center_y)

        depth_m = self._resolve_depth(msg, center_u, center_v)
        if depth_m is None or depth_m <= 0.0:
            self.get_logger().warn(
                f"Box depth invalid; skip id={int(msg.object_id)} "
                f"center=({center_u},{center_v})"
            )
            return

        try:
            x_c, y_c, z_c = pixel_to_camera_xyz(
                center_u,
                center_v,
                depth_m,
                fx,
                fy,
                cx_i,
                cy_i,
            )
        except ValueError as exc:
            self.get_logger().warn(f"pixel_to_camera_xyz failed for box: {exc}")
            return

        bbox_w_px = float(msg.width_px) if float(msg.width_px) > 0.0 else float(msg.x_max - msg.x_min)
        bbox_h_px = float(msg.height_px) if float(msg.height_px) > 0.0 else float(msg.y_max - msg.y_min)

        width_m, length_m, height_m = compute_box_size_from_bbox(
            bbox_w_px=bbox_w_px,
            bbox_h_px=bbox_h_px,
            depth_m=depth_m,
            fx=fx,
            fy=fy,
            default_x_m=self._default_box_x,
            default_y_m=self._default_box_y,
            default_z_m=self._default_box_z,
        )
        area_m2 = width_m * length_m
        area_cm2 = area_m2 * 10000.0

        width_m_safe = width_m + 2.0 * self._box_obstacle_margin_m
        length_m_safe = length_m + 2.0 * self._box_obstacle_margin_m
        safe_area_m2 = width_m_safe * length_m_safe
        safe_area_cm2 = safe_area_m2 * 10000.0

        now = self._now_seconds()
        class_name = str(msg.class_name) if str(msg.class_name) else "box"
        x_out, y_out, z_out = self.transform_camera_to_world(x_c, y_c, z_c)

        with self._data_lock:
            self._latest_boxes[int(msg.object_id)] = BoxData(
                box_id=int(msg.object_id),
                class_name=class_name,
                confidence=float(msg.confidence),
                x_min=int(msg.x_min),
                y_min=int(msg.y_min),
                x_max=int(msg.x_max),
                y_max=int(msg.y_max),
                center_x=center_u,
                center_y=center_v,
                x_c_m=x_c,
                y_c_m=y_c,
                z_c_m=z_c,
                x_out_m=x_out,
                y_out_m=y_out,
                z_out_m=z_out,
                size_x_m=width_m,
                size_y_m=length_m,
                size_z_m=height_m,
                area_m2=area_m2,
                area_cm2=area_cm2,
                safe_area_m2=safe_area_m2,
                safe_area_cm2=safe_area_cm2,
                stamp_sec=now,
            )

        self.get_logger().debug(
            f"box id={int(msg.object_id)} depth={depth_m:.3f} m, "
            f"size={width_m:.3f}x{length_m:.3f} m "
            f"area={area_cm2:.1f} cm2, "
            f"safe_size={width_m_safe:.3f}x{length_m_safe:.3f} m "
            f"safe_area={safe_area_cm2:.1f} cm2"
        )

    # ------------------------------------------------------------------ publish objects
    def _publish_objects(self) -> None:
        now = self._now_seconds()
        now_msg = self.get_clock().now().to_msg()
        frame_id = self._effective_output_frame_id()

        with self._data_lock:
            self._latest_woods = {
                wood_id: wood
                for wood_id, wood in self._latest_woods.items()
                if (now - wood.stamp_sec) <= self._stale_timeout_sec
            }
            self._latest_boxes = {
                box_id: box
                for box_id, box in self._latest_boxes.items()
                if (now - box.stamp_sec) <= self._stale_timeout_sec
            }

            woods_snapshot = list(self._latest_woods.values())
            boxes_snapshot = list(self._latest_boxes.values())

        wood_arr_msg = WoodArray()
        wood_arr_msg.header.stamp = now_msg
        wood_arr_msg.header.frame_id = frame_id

        wood_list: List[Wood] = []
        for wood_data in sorted(woods_snapshot, key=lambda item: item.wood_id):
            wood_msg = Wood()
            wood_msg.header.stamp = now_msg
            wood_msg.header.frame_id = frame_id
            wood_msg.wood_id = wood_data.wood_id
            wood_msg.class_name = wood_data.class_name
            wood_msg.confidence = wood_data.confidence

            wood_msg.pose.position.x = wood_data.x_out_m
            wood_msg.pose.position.y = wood_data.y_out_m
            wood_msg.pose.position.z = wood_data.z_out_m

            if wood_data.yaw_valid:
                qx, qy, qz, qw = yaw_to_quaternion_z(wood_data.yaw_deg)
            else:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

            wood_msg.pose.orientation.x = qx
            wood_msg.pose.orientation.y = qy
            wood_msg.pose.orientation.z = qz
            wood_msg.pose.orientation.w = qw

            wood_list.append(wood_msg)

        wood_arr_msg.woods = wood_list

        box_arr_msg = BoxArray()
        box_arr_msg.header.stamp = now_msg
        box_arr_msg.header.frame_id = frame_id

        box_list: List[Box] = []
        for box_data in sorted(boxes_snapshot, key=lambda item: item.box_id):
            box_msg = Box()
            box_msg.header.stamp = now_msg
            box_msg.header.frame_id = frame_id
            box_msg.box_id = box_data.box_id
            box_msg.class_name = box_data.class_name
            box_msg.confidence = box_data.confidence

            box_msg.pose.position.x = box_data.x_out_m
            box_msg.pose.position.y = box_data.y_out_m
            box_msg.pose.position.z = box_data.z_out_m

            box_msg.pose.orientation.x = 0.0
            box_msg.pose.orientation.y = 0.0
            box_msg.pose.orientation.z = 0.0
            box_msg.pose.orientation.w = 1.0

            box_msg.size.x = box_data.size_x_m
            box_msg.size.y = box_data.size_y_m
            box_msg.size.z = box_data.size_z_m

            box_list.append(box_msg)

        box_arr_msg.boxes = box_list

        self._pub_wood_objects.publish(wood_arr_msg)
        self._pub_box_objects.publish(box_arr_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixelToBaseMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
