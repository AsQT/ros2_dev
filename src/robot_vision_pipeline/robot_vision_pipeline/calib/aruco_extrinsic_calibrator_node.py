#!/usr/bin/env python3

import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class FlowList(list):
    pass


def flow_list_representer(dumper, data):
    return dumper.represent_sequence(
        "tag:yaml.org,2002:seq",
        data,
        flow_style=True,
    )


yaml.SafeDumper.add_representer(FlowList, flow_list_representer)


class ArucoExtrinsicCalibratorNode(Node):
    def __init__(self):
        super().__init__("aruco_extrinsic_calibrator_node")

        default_layout = self._default_package_path("config/aruco_board_layout.yaml")
        default_result = self._default_result_yaml_path()

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("debug_image_topic", "/vision/aruco_calib/debug_image")
        self.declare_parameter("marker_size", 0.021)
        self.declare_parameter("square_size", 0.029)
        self.declare_parameter("squares_x", 8)
        self.declare_parameter("squares_y", 6)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("board_layout_file", default_layout)
        self.declare_parameter("world_frame", "aruco_world")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_bad_tf_for_debug", False)
        self.declare_parameter("save_result", True)
        self.declare_parameter("result_yaml_path", default_result)
        self.declare_parameter("min_detected_markers", 2)
        self.declare_parameter("min_charuco_corners", 6)
        self.declare_parameter("show_debug_axes", True)
        self.declare_parameter("axis_length", 0.08)
        self.declare_parameter("use_ransac", True)
        self.declare_parameter("ransac_reprojection_error", 3.0)
        self.declare_parameter("ransac_confidence", 0.99)
        self.declare_parameter("max_allowed_reprojection_error", 5.0)
        self.declare_parameter("corner_order_mode", "opencv")
        self.declare_parameter("print_marker_debug", True)
        self.declare_parameter("use_board_roi", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.square_size = float(self.get_parameter("square_size").value)
        self.squares_x = int(self.get_parameter("squares_x").value)
        self.squares_y = int(self.get_parameter("squares_y").value)
        self.aruco_dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        self.board_layout_file = self._resolve_path(
            self.get_parameter("board_layout_file").value
        )
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_bad_tf_for_debug = bool(
            self.get_parameter("publish_bad_tf_for_debug").value
        )
        self.save_result = bool(self.get_parameter("save_result").value)
        result_yaml_param = self.get_parameter("result_yaml_path").value
        self.result_yaml_path = self._resolve_path(result_yaml_param)
        if str(result_yaml_param) == str(default_result):
            self.result_yaml_path = self._result_path_from_board_layout_or_default(
                self.result_yaml_path
            )
        self.min_detected_markers = int(self.get_parameter("min_detected_markers").value)
        self.min_charuco_corners = int(self.get_parameter("min_charuco_corners").value)
        self.show_debug_axes = bool(self.get_parameter("show_debug_axes").value)
        self.axis_length = float(self.get_parameter("axis_length").value)
        self.use_ransac = bool(self.get_parameter("use_ransac").value)
        self.ransac_reprojection_error = float(
            self.get_parameter("ransac_reprojection_error").value
        )
        self.ransac_confidence = float(self.get_parameter("ransac_confidence").value)
        self.max_allowed_reprojection_error = float(
            self.get_parameter("max_allowed_reprojection_error").value
        )
        self.corner_order_mode = str(self.get_parameter("corner_order_mode").value)
        self.print_marker_debug = bool(self.get_parameter("print_marker_debug").value)
        self.use_board_roi = bool(self.get_parameter("use_board_roi").value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_frame = ""
        self._logged_camera_info = False
        self._saved_result = False
        self._last_result_log = 0.0
        self._last_detection_log = 0.0
        self._last_marker_debug_log = 0.0
        self._warn_times = {}

        self.marker_layout = self._load_board_layout()
        self.board_width = self.squares_x * self.square_size
        self.board_height = self.squares_y * self.square_size
        self.aruco_dict = self._make_aruco_dictionary(self.aruco_dictionary_name)
        self.aruco_params = self._make_detector_parameters()
        self.charuco_board = self._create_charuco_board()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_image = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.pub_debug = self.create_publisher(Image, self.debug_image_topic, 10)

        self._log_startup()

    def _default_package_path(self, relative_path):
        try:
            return str(Path(get_package_share_directory("robot_vision_pipeline")) / relative_path)
        except Exception:
            return str(Path(__file__).resolve().parents[2] / relative_path)

    def _default_result_yaml_path(self):
        result_name = "aruco_extrinsic_result.yaml"

        cwd = Path.cwd().resolve()
        cwd_candidates = [
            cwd / "src" / "robot_vision_pipeline" / "config",
            cwd / "robot_vision_pipeline" / "config",
        ]
        for config_dir in cwd_candidates:
            if config_dir.exists():
                return str(config_dir / result_name)

        file_path = Path(__file__).resolve()
        for parent in file_path.parents:
            if parent.name == "robot_vision_pipeline" and (parent / "config").exists():
                return str(parent / "config" / result_name)

        try:
            share_config = Path(get_package_share_directory("robot_vision_pipeline")) / "config"
            self.get_logger().warn(
                "Could not infer source package config directory for aruco_extrinsic_result.yaml. "
                f"Falling back to install/share path: {share_config / result_name}"
            )
            return str(share_config / result_name)
        except Exception:
            fallback = cwd / "aruco_extrinsic_result.yaml"
            self.get_logger().warn(
                "Could not infer source or install package path for aruco_extrinsic_result.yaml. "
                f"Falling back to current directory: {fallback}"
            )
            return str(fallback)

    def _result_path_from_board_layout_or_default(self, current_path):
        layout_config_dir = self.board_layout_file.parent
        is_source_config = (
            layout_config_dir.name == "config"
            and layout_config_dir.parent.name == "robot_vision_pipeline"
            and "install" not in layout_config_dir.parts
        )
        if is_source_config:
            return layout_config_dir / "aruco_extrinsic_result.yaml"
        if "install" in Path(current_path).parts:
            self.get_logger().warn(
                "Default result_yaml_path is inside install/share. "
                "Use -p result_yaml_path:=<source_package>/config/aruco_extrinsic_result.yaml "
                "if you want to force a source config location."
            )
        return current_path

    @staticmethod
    def _resolve_path(path_value):
        path = Path(str(path_value)).expanduser()
        if path.is_absolute():
            return path
        package_root = Path(__file__).resolve().parents[2]
        candidate = package_root / path
        if candidate.exists() or package_root.name == "robot_vision_pipeline":
            return candidate
        return Path.cwd() / path

    def _load_board_layout(self):
        try:
            with self.board_layout_file.open("r", encoding="utf-8") as file:
                data = yaml.safe_load(file) or {}
        except Exception as exc:
            self.get_logger().error(f"Cannot read board layout YAML: {exc}")
            return {}

        self.marker_size = float(data.get("marker_size", self.marker_size))
        self.square_size = float(data.get("square_size", self.square_size))
        self.squares_x = int(data.get("squares_x", self.squares_x))
        self.squares_y = int(data.get("squares_y", self.squares_y))
        self.aruco_dictionary_name = str(
            data.get("aruco_dictionary", self.aruco_dictionary_name)
        )
        self.world_frame = str(data.get("world_frame", self.world_frame))
        self.camera_frame = str(data.get("camera_frame", self.camera_frame))
        self.use_board_roi = bool(data.get("use_board_roi", self.use_board_roi))

        raw_layout = data.get("marker_layout", {})
        layout = {}
        for marker_id_raw, cell in raw_layout.items():
            try:
                marker_id = int(marker_id_raw)
                layout[marker_id] = {
                    "row": int(cell["row"]),
                    "col": int(cell["col"]),
                }
            except Exception as exc:
                self.get_logger().warn(
                    f"Skipping invalid marker layout entry {marker_id_raw}: {exc}"
                )
        self.get_logger().info(f"Loaded {len(layout)} fallback marker layout entries")
        return layout

    def _log_startup(self):
        self.get_logger().info("aruco_extrinsic_calibrator_node started")
        self.get_logger().info(f"Image topic          : {self.image_topic}")
        self.get_logger().info(f"CameraInfo topic     : {self.camera_info_topic}")
        self.get_logger().info(f"Debug image topic    : {self.debug_image_topic}")
        self.get_logger().info(f"Board layout file    : {self.board_layout_file}")
        self.get_logger().info(f"Result YAML path     : {self.result_yaml_path}")
        self.get_logger().info(
            "\n"
            "========== ChArUco board ==========\n"
            f"squares_x: {self.squares_x}\n"
            f"squares_y: {self.squares_y}\n"
            f"square_size: {self.square_size:.6f}\n"
            f"marker_size: {self.marker_size:.6f}\n"
            f"board_width: {self.board_width:.6f}\n"
            f"board_height: {self.board_height:.6f}\n"
            f"aruco_dictionary: {self.aruco_dictionary_name}\n"
            "==================================="
        )
        self.get_logger().info(
            "\n"
            "========== World frame ==========\n"
            "origin: center\n"
            "X+: right\n"
            "Y+: up\n"
            "Z+: out_of_board\n"
            f"TF: {self.world_frame} -> {self.camera_frame}\n"
            "================================="
        )
        if self.use_board_roi:
            self.get_logger().warn(
                "use_board_roi is true but ROI cropping is disabled for ChArUco calibration. "
                "Detecting on the full image."
            )

    def _camera_info_to_intrinsics(self, msg):
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1)
        if dist_coeffs.size == 0:
            dist_coeffs = np.zeros((5,), dtype=np.float64)
        return camera_matrix, dist_coeffs

    def _create_charuco_board(self):
        try:
            return cv2.aruco.CharucoBoard_create(
                self.squares_x,
                self.squares_y,
                self.square_size,
                self.marker_size,
                self.aruco_dict,
            )
        except Exception:
            return cv2.aruco.CharucoBoard(
                (self.squares_x, self.squares_y),
                self.square_size,
                self.marker_size,
                self.aruco_dict,
            )

    def _detect_aruco_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )
            return corners, ids, rejected
        except Exception as exc:
            self.get_logger().error(f"ArUco detection failed: {exc}")
            return [], None, []

    def _detect_charuco_board(self, frame):
        corners, ids, _ = self._detect_aruco_markers(frame)
        detected_aruco_ids = [] if ids is None else [int(x) for x in ids.flatten()]
        if ids is None or len(ids) == 0:
            return corners, ids, detected_aruco_ids, None, None, "No ArUco markers detected"

        charuco_marker_corners, charuco_marker_ids, duplicate_ids = (
            self._filter_duplicate_aruco_markers_for_charuco(corners, ids)
        )
        if duplicate_ids:
            self.warn_throttled(
                "duplicate_charuco_markers",
                "Duplicate ArUco IDs detected before ChArUco interpolation. "
                f"Ignoring duplicated IDs for ChArUco: {duplicate_ids}",
                2.0,
            )
        if charuco_marker_ids is None or len(charuco_marker_ids) == 0:
            return (
                corners,
                ids,
                detected_aruco_ids,
                None,
                None,
                "All detected ArUco IDs were duplicates; no markers left for ChArUco interpolation",
            )

        try:
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                charuco_marker_corners,
                charuco_marker_ids,
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                self.charuco_board,
            )
        except Exception as exc:
            return corners, ids, detected_aruco_ids, None, None, f"interpolateCornersCharuco failed: {exc}"

        if charuco_ids is None or charuco_corners is None:
            return corners, ids, detected_aruco_ids, None, None, "No ChArUco corners interpolated"

        if int(count) < self.min_charuco_corners:
            return (
                corners,
                ids,
                detected_aruco_ids,
                charuco_corners,
                charuco_ids,
                f"Not enough valid ChArUco corners for solvePnP. Need at least {self.min_charuco_corners}, got {int(count)}.",
            )

        return corners, ids, detected_aruco_ids, charuco_corners, charuco_ids, ""

    @staticmethod
    def _filter_duplicate_aruco_markers_for_charuco(corners, ids):
        flat_ids = [int(x) for x in ids.flatten()]
        counts = {}
        for marker_id in flat_ids:
            counts[marker_id] = counts.get(marker_id, 0) + 1
        duplicate_ids = sorted(marker_id for marker_id, count in counts.items() if count > 1)
        filtered_corners = []
        filtered_ids = []
        for marker_corners, marker_id in zip(corners, flat_ids):
            if counts[marker_id] == 1:
                filtered_corners.append(marker_corners)
                filtered_ids.append([marker_id])
        if not filtered_ids:
            return [], None, duplicate_ids
        return filtered_corners, np.array(filtered_ids, dtype=np.int32), duplicate_ids

    def _charuco_id_to_row_col(self, charuco_id):
        corners_x = self.squares_x - 1
        row_from_top = int(charuco_id) // corners_x
        col = int(charuco_id) % corners_x
        return row_from_top, col

    def _create_charuco_object_point_center_origin(self, charuco_id):
        row_from_top, col = self._charuco_id_to_row_col(charuco_id)
        x = (col + 1) * self.square_size - self.board_width / 2.0
        y = self.board_height / 2.0 - (row_from_top + 1) * self.square_size
        return np.array([x, y, 0.0], dtype=np.float64)

    def _create_marker_object_points_center_origin(self, marker_id):
        cell = self.marker_layout[marker_id]
        row = float(cell["row"])
        col = float(cell["col"])
        x0 = (col + 0.5) * self.square_size - self.board_width / 2.0
        y0 = (row + 0.5) * self.square_size - self.board_height / 2.0
        half = self.marker_size * 0.5
        return np.array(
            [
                [x0 - half, y0 + half, 0.0],
                [x0 + half, y0 + half, 0.0],
                [x0 + half, y0 - half, 0.0],
                [x0 - half, y0 - half, 0.0],
            ],
            dtype=np.float64,
        )

    def _collect_charuco_2d_3d_correspondences(self, charuco_corners, charuco_ids):
        object_points = []
        image_points = []
        debug_rows = []
        for corner, corner_id_raw in zip(charuco_corners.reshape(-1, 2), charuco_ids.reshape(-1)):
            corner_id = int(corner_id_raw)
            row_from_top, col = self._charuco_id_to_row_col(corner_id)
            object_point = self._create_charuco_object_point_center_origin(corner_id)
            object_points.append(object_point)
            image_points.append(corner.astype(np.float64))
            debug_rows.append(
                {
                    "id": corner_id,
                    "row_from_top": row_from_top,
                    "col": col,
                    "object": object_point,
                    "image": corner.astype(np.float64),
                }
            )
        return (
            np.array(object_points, dtype=np.float64).reshape(-1, 3),
            np.array(image_points, dtype=np.float64).reshape(-1, 2),
            debug_rows,
        )

    def _apply_corner_order_mode(self, marker_corners):
        pts = marker_corners.reshape(4, 2).astype(np.float64)
        orders = {
            "opencv": [0, 1, 2, 3],
            "flip_vertical": [3, 2, 1, 0],
            "flip_horizontal": [1, 0, 3, 2],
            "reverse": [0, 3, 2, 1],
        }
        order = orders.get(self.corner_order_mode, orders["opencv"])
        if self.corner_order_mode not in orders:
            self.warn_throttled(
                "corner_order_mode",
                f"Unknown corner_order_mode '{self.corner_order_mode}', using opencv",
                2.0,
            )
        return pts[order]

    def _collect_aruco_fallback_2d_3d_correspondences(self, corners, ids):
        object_points = []
        image_points = []
        used_ids = []
        missing_ids = []
        duplicate_ids = []
        marker_debug = []
        seen_marker_ids = set()

        if ids is None:
            return (
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 2), dtype=np.float64),
                used_ids,
                missing_ids,
                duplicate_ids,
                marker_debug,
            )

        for index, marker_id_raw in enumerate(ids.flatten()):
            marker_id = int(marker_id_raw)
            if marker_id in seen_marker_ids:
                duplicate_ids.append(marker_id)
                continue
            seen_marker_ids.add(marker_id)

            if marker_id not in self.marker_layout:
                missing_ids.append(marker_id)
                continue

            obj_corners = self._create_marker_object_points_center_origin(marker_id)
            img_corners = self._apply_corner_order_mode(corners[index])
            object_points.extend(obj_corners.tolist())
            image_points.extend(img_corners.tolist())
            used_ids.append(marker_id)
            cell = self.marker_layout[marker_id]
            row = int(cell["row"])
            col = int(cell["col"])
            center_world = np.array(
                [
                    (float(col) + 0.5) * self.square_size - self.board_width / 2.0,
                    (float(row) + 0.5) * self.square_size - self.board_height / 2.0,
                    0.0,
                ],
                dtype=np.float64,
            )
            marker_debug.append(
                {
                    "marker_id": marker_id,
                    "row": row,
                    "col": col,
                    "center_world": center_world,
                    "object_points": obj_corners.copy(),
                    "image_points": img_corners.copy(),
                }
            )

        return (
            np.array(object_points, dtype=np.float64).reshape(-1, 3),
            np.array(image_points, dtype=np.float64).reshape(-1, 2),
            sorted(used_ids),
            sorted(set(missing_ids)),
            sorted(set(duplicate_ids)),
            marker_debug,
        )

    def _validate_pnp_inputs(self, object_points, image_points):
        if object_points is None or image_points is None:
            return False, "object_points or image_points is None"
        if len(object_points) != len(image_points):
            return False, f"2D/3D point count mismatch: object={len(object_points)}, image={len(image_points)}"
        if len(object_points) < 4:
            return False, f"Not enough valid points for solvePnP. Need at least 4 points, got {len(object_points)}."
        if object_points.shape[1] != 3 or image_points.shape[1] != 2:
            return False, f"Invalid point shapes: object_points={object_points.shape}, image_points={image_points.shape}"
        if not np.isfinite(object_points).all() or not np.isfinite(image_points).all():
            return False, "object_points or image_points contains NaN/Inf"
        return True, ""

    def _filter_planar_homography_inliers(self, object_points, image_points):
        valid, reason = self._validate_pnp_inputs(object_points, image_points)
        if not valid:
            return object_points, image_points, None, reason
        if len(object_points) < 6:
            return object_points, image_points, None, ""

        plane_points = object_points[:, :2].astype(np.float64)
        threshold = max(float(self.ransac_reprojection_error), 5.0)
        homography, mask = cv2.findHomography(
            plane_points,
            image_points.astype(np.float64),
            cv2.RANSAC,
            threshold,
        )
        if homography is None or mask is None:
            return object_points, image_points, None, "findHomography failed"

        inlier_mask = mask.reshape(-1).astype(bool)
        inlier_count = int(np.count_nonzero(inlier_mask))
        if inlier_count < max(4, self.min_charuco_corners):
            return (
                object_points,
                image_points,
                inlier_mask,
                f"Not enough homography inliers. Need at least {max(4, self.min_charuco_corners)}, got {inlier_count}.",
            )

        return object_points[inlier_mask], image_points[inlier_mask], inlier_mask, ""

    def _solve_pnp(self, object_points, image_points):
        valid, reason = self._validate_pnp_inputs(object_points, image_points)
        if not valid:
            return False, None, None, None, reason

        try:
            if self.use_ransac:
                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                    reprojectionError=self.ransac_reprojection_error,
                    confidence=self.ransac_confidence,
                )
                if success:
                    return True, rvec, tvec, inliers, ""

                # Planar ChArUco inlier sets can be small after filtering. If
                # RANSAC cannot initialize, still try iterative PnP and let the
                # reprojection-error gate decide whether the pose is usable.
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                reason = "" if success else "cv2.solvePnPRansac and cv2.solvePnP returned success=False"
                return bool(success), rvec, tvec, inliers, reason

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            reason = "" if success else "cv2.solvePnP returned success=False"
            return bool(success), rvec, tvec, None, reason
        except Exception as exc:
            return False, None, None, None, f"solvePnP exception: {exc}"

    @staticmethod
    def _invert_transform(rvec, tvec):
        r_cw, _ = cv2.Rodrigues(rvec)
        t_cw = tvec.reshape(3)
        r_wc = r_cw.T
        t_wc = -r_wc @ t_cw
        return r_cw, t_cw, r_wc, t_wc

    @staticmethod
    def _rotation_matrix_to_quaternion(rotation_matrix):
        r = rotation_matrix
        trace = float(np.trace(r))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r[2, 1] - r[1, 2]) / s
            qy = (r[0, 2] - r[2, 0]) / s
            qz = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
        quat = np.array([qx, qy, qz, qw], dtype=np.float64)
        norm = np.linalg.norm(quat)
        if norm > 0.0:
            quat /= norm
        return quat

    @staticmethod
    def _rotation_matrix_to_rpy(rotation_matrix):
        r = rotation_matrix
        sy = math.sqrt(float(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0]))
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
            pitch = math.atan2(float(-r[2, 0]), sy)
            yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
        else:
            roll = math.atan2(float(-r[1, 2]), float(r[1, 1]))
            pitch = math.atan2(float(-r[2, 0]), sy)
            yaw = 0.0
        return np.array([roll, pitch, yaw], dtype=np.float64)

    def _compute_reprojection_error(self, object_points, image_points, rvec, tvec):
        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        projected_points = projected_points.reshape(-1, 2)
        errors = np.linalg.norm(projected_points - image_points, axis=1)
        return float(np.mean(errors)), float(np.max(errors)), projected_points

    def _draw_center_origin_xy_axes(self, annotated, rvec, tvec):
        axis_points = np.array(
            [
                [0.0, 0.0, 0.0],
                [self.axis_length, 0.0, 0.0],
                [0.0, self.axis_length, 0.0],
            ],
            dtype=np.float64,
        )
        projected, _ = cv2.projectPoints(
            axis_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        pts = projected.reshape(-1, 2)
        origin = tuple(np.round(pts[0]).astype(int))
        x_axis = tuple(np.round(pts[1]).astype(int))
        y_axis = tuple(np.round(pts[2]).astype(int))
        cv2.circle(annotated, origin, 7, (0, 255, 255), -1)
        cv2.arrowedLine(annotated, origin, x_axis, (0, 0, 255), 3, tipLength=0.18)
        cv2.arrowedLine(annotated, origin, y_axis, (0, 255, 0), 3, tipLength=0.18)
        self._put_text(annotated, "O_world", origin[0] + 8, origin[1] - 8, (0, 255, 255), 0.55)
        self._put_text(annotated, "X+", x_axis[0] + 6, x_axis[1] - 6, (0, 0, 255), 0.55)
        self._put_text(annotated, "Y+", y_axis[0] + 6, y_axis[1] - 6, (0, 255, 0), 0.55)

    def _draw_debug_image(
        self,
        annotated,
        aruco_corners,
        aruco_ids,
        charuco_corners,
        charuco_ids,
        projected_points,
        rvec,
        tvec,
        mean_error,
        max_error,
        status_text,
        reason,
        ok,
    ):
        if aruco_ids is not None and len(aruco_ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, aruco_corners, aruco_ids)
        if charuco_corners is not None and charuco_ids is not None:
            try:
                cv2.aruco.drawDetectedCornersCharuco(
                    annotated,
                    charuco_corners,
                    charuco_ids,
                    (0, 255, 0),
                )
            except Exception:
                for point in charuco_corners.reshape(-1, 2):
                    cv2.circle(annotated, tuple(np.round(point).astype(int)), 4, (0, 255, 0), -1)
        if projected_points is not None:
            for point in projected_points.reshape(-1, 2):
                cv2.drawMarker(
                    annotated,
                    tuple(np.round(point).astype(int)),
                    (255, 0, 255),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=10,
                    thickness=2,
                )
        if ok and rvec is not None and tvec is not None and self.show_debug_axes:
            self._draw_center_origin_xy_axes(annotated, rvec, tvec)

        color = (0, 220, 0) if ok else (0, 0, 255)
        self._put_text(annotated, f"Status: {status_text}", 20, 34, color, 0.78)
        if reason:
            self._put_text(annotated, f"Reason: {reason[:95]}", 20, 64, (0, 165, 255), 0.5)
        y = 92 if reason else 64
        detected = [] if aruco_ids is None else [int(x) for x in aruco_ids.flatten()]
        num_charuco = 0 if charuco_ids is None else len(charuco_ids)
        self._put_text(annotated, f"Detected ArUco IDs: {detected}", 20, y, (255, 255, 255), 0.5)
        self._put_text(annotated, f"Detected ChArUco corners: {num_charuco}", 20, y + 26, (255, 255, 255), 0.5)
        if mean_error > 0.0 or max_error > 0.0:
            self._put_text(
                annotated,
                f"reproj mean/max: {mean_error:.3f}/{max_error:.3f} px",
                20,
                y + 52,
                (255, 255, 255) if ok else (0, 165, 255),
                0.5,
            )
        return annotated

    @staticmethod
    def _put_text(image, text, x, y, color=(255, 255, 255), scale=0.55):
        cv2.putText(image, str(text), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, str(text), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)

    def _publish_tf(self, stamp, t_wc, quat_wc):
        if not self.publish_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = float(t_wc[0])
        transform.transform.translation.y = float(t_wc[1])
        transform.transform.translation.z = float(t_wc[2])
        transform.transform.rotation.x = float(quat_wc[0])
        transform.transform.rotation.y = float(quat_wc[1])
        transform.transform.rotation.z = float(quat_wc[2])
        transform.transform.rotation.w = float(quat_wc[3])
        self.tf_broadcaster.sendTransform(transform)

    def _save_result_yaml(self, result):
        if not self.save_result:
            return
        t = result["t_wc"]
        rpy = result["rpy_wc"]
        q = result["quat_wc"]
        t_world_camera = result["t_world_camera_matrix"]
        t_camera_world = result["t_camera_world_matrix"]
        data = {
            "world_frame": self.world_frame,
            "camera_frame": self.camera_frame,
            "extrinsic_matrix": {
                "T_world_camera": self._matrix_to_list(t_world_camera),
                "T_camera_world": self._matrix_to_list(t_camera_world),
            },
            "camera_pose_in_world": {
                "x": float(t[0]),
                "y": float(t[1]),
                "z": float(t[2]),
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "yaw": float(rpy[2]),
            },
            "camera_pose_quaternion": {
                "x": float(q[0]),
                "y": float(q[1]),
                "z": float(q[2]),
                "w": float(q[3]),
            },
            "reprojection_error": {
                "mean_px": float(result["mean_error"]),
                "max_px": float(result["max_error"]),
            },
            "camera_matrix": self._matrix_to_list(self.camera_matrix),
            "distortion_coefficients": self._float_list(self.dist_coeffs),
            "board": {
                "marker_size": float(self.marker_size),
                "square_size": float(self.square_size),
                "squares_x": int(self.squares_x),
                "squares_y": int(self.squares_y),
                "board_width": float(self.board_width),
                "board_height": float(self.board_height),
                "aruco_dictionary": self.aruco_dictionary_name,
                "board_layout_file": str(self.board_layout_file),
            },
            "world_convention": {
                "origin": "center",
                "x_axis": "right",
                "y_axis": "up",
                "z_axis": "out_of_board",
            },
            "debug": {
                "points_source": result["points_source"],
                "used_points_count": int(result["point_count"]),
                "homography_total_count": int(result.get("homography_total_count", 0)),
                "homography_inlier_count": int(result.get("homography_inlier_count", 0)),
                "ransac_inliers_count": int(result.get("inliers_count", 0)),
                "result_yaml_path": str(self.result_yaml_path),
            },
            "translation": {
                "x": float(t[0]),
                "y": float(t[1]),
                "z": float(t[2]),
            },
            "rotation_rpy": {
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "yaw": float(rpy[2]),
            },
            "rotation_quaternion": {
                "x": float(q[0]),
                "y": float(q[1]),
                "z": float(q[2]),
                "w": float(q[3]),
            },
            "T_world_camera": self._matrix_to_list(t_world_camera),
            "T_camera_world": self._matrix_to_list(t_camera_world),
        }
        try:
            self.result_yaml_path.parent.mkdir(parents=True, exist_ok=True)
            with self.result_yaml_path.open("w", encoding="utf-8") as file:
                yaml.safe_dump(data, file, sort_keys=False, default_flow_style=False, allow_unicode=True)
            if not self._saved_result:
                self.get_logger().info(f"Saved extrinsic result to:\n{self.result_yaml_path}")
            self._saved_result = True
        except Exception as exc:
            self.get_logger().error(f"Cannot save result YAML {self.result_yaml_path}: {exc}")

    def camera_info_callback(self, msg):
        self.camera_matrix, self.dist_coeffs = self._camera_info_to_intrinsics(msg)
        self.camera_info_frame = msg.header.frame_id or self.camera_info_frame
        if not self._logged_camera_info:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            self.get_logger().info(
                "CameraInfo received: "
                f"frame={self.camera_info_frame}, "
                f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}, "
                f"D={self.dist_coeffs.tolist()}"
            )
            self._logged_camera_info = True

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge error: {exc}")
            return

        annotated = frame.copy()
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.warn_throttled("camera_info", "Waiting for camera_info...", 2.0)
            self._draw_debug_image(
                annotated,
                [],
                None,
                None,
                None,
                None,
                None,
                None,
                0.0,
                0.0,
                "Waiting for camera_info",
                "Waiting for camera_info",
                False,
            )
            self._publish_debug(msg, annotated)
            return

        aruco_corners, aruco_ids, detected_aruco_ids, charuco_corners, charuco_ids, charuco_reason = self._detect_charuco_board(frame)
        object_points = None
        image_points = None
        points_source = "charuco"
        marker_debug = []
        used_ids = []
        missing_ids = []
        duplicate_ids = []

        if charuco_ids is not None and len(charuco_ids) >= self.min_charuco_corners:
            object_points, image_points, charuco_debug = self._collect_charuco_2d_3d_correspondences(
                charuco_corners,
                charuco_ids,
            )
            self._print_charuco_debug_log(charuco_debug)
            pnp_reason = ""
        else:
            points_source = "aruco_fallback"
            object_points, image_points, used_ids, missing_ids, duplicate_ids, marker_debug = (
                self._collect_aruco_fallback_2d_3d_correspondences(aruco_corners, aruco_ids)
            )
            self._print_marker_debug_log(marker_debug)
            pnp_reason = charuco_reason or "Not enough valid ChArUco corners"
            if missing_ids:
                self.warn_throttled(
                    "missing_ids",
                    f"Detected marker ID not found in fallback layout: {missing_ids}",
                    2.0,
                )
            if duplicate_ids:
                self.warn_throttled(
                    "duplicate_ids",
                    f"Duplicate marker ID in one frame, skip: {duplicate_ids}",
                    2.0,
                )
            if len(used_ids) < self.min_detected_markers:
                pnp_reason = (
                    f"{pnp_reason}; not enough fallback markers. "
                    f"Need {self.min_detected_markers}, got {len(used_ids)}."
                )

        self._log_detection_state(
            detected_aruco_ids,
            charuco_ids,
            object_points,
            image_points,
            points_source,
        )

        homography_reason = ""
        homography_inlier_count = 0
        homography_total_count = 0 if object_points is None else len(object_points)
        if points_source == "charuco":
            object_points, image_points, homography_mask, homography_reason = (
                self._filter_planar_homography_inliers(object_points, image_points)
            )
            homography_inlier_count = (
                len(object_points)
                if homography_mask is None
                else int(np.count_nonzero(homography_mask))
            )
            self._log_homography_filter(
                homography_total_count,
                homography_inlier_count,
                homography_reason,
            )
            if homography_reason:
                self.warn_throttled("homography_filter", homography_reason, 2.0)
                self._draw_debug_image(
                    annotated,
                    aruco_corners,
                    aruco_ids,
                    charuco_corners,
                    charuco_ids,
                    None,
                    None,
                    None,
                    0.0,
                    0.0,
                    "solvePnP failed",
                    homography_reason,
                    False,
                )
                self._publish_debug(msg, annotated)
                return

        valid, validation_reason = self._validate_pnp_inputs(object_points, image_points)
        if not valid or (points_source == "aruco_fallback" and len(used_ids) < self.min_detected_markers):
            reason = validation_reason or homography_reason or pnp_reason
            self.warn_throttled("not_enough", reason, 2.0)
            self._draw_debug_image(
                annotated,
                aruco_corners,
                aruco_ids,
                charuco_corners,
                charuco_ids,
                None,
                None,
                None,
                0.0,
                0.0,
                "solvePnP failed",
                reason,
                False,
            )
            self._publish_debug(msg, annotated)
            return

        success, rvec, tvec, inliers, solve_reason = self._solve_pnp(object_points, image_points)
        if not success:
            reason = solve_reason or "solvePnP failed"
            self.warn_throttled("solve_pnp", reason, 2.0)
            self._draw_debug_image(
                annotated,
                aruco_corners,
                aruco_ids,
                charuco_corners,
                charuco_ids,
                None,
                None,
                None,
                0.0,
                0.0,
                "solvePnP failed",
                reason,
                False,
            )
            self._publish_debug(msg, annotated)
            return

        r_cw, t_cw, r_wc, t_wc = self._invert_transform(rvec, tvec)
        quat_wc = self._rotation_matrix_to_quaternion(r_wc)
        rpy_wc = self._rotation_matrix_to_rpy(r_wc)
        mean_error, max_error, projected_points = self._compute_reprojection_error(
            object_points,
            image_points,
            rvec,
            tvec,
        )
        pose_accepted = mean_error <= self.max_allowed_reprojection_error
        inliers_count = 0 if inliers is None else int(len(inliers))

        t_camera_world_matrix = np.eye(4, dtype=np.float64)
        t_camera_world_matrix[:3, :3] = r_cw
        t_camera_world_matrix[:3, 3] = t_cw
        t_world_camera_matrix = np.eye(4, dtype=np.float64)
        t_world_camera_matrix[:3, :3] = r_wc
        t_world_camera_matrix[:3, 3] = t_wc

        result = {
            "point_count": object_points.shape[0],
            "points_source": points_source,
            "homography_total_count": homography_total_count,
            "homography_inlier_count": homography_inlier_count,
            "t_wc": t_wc,
            "quat_wc": quat_wc,
            "rpy_wc": rpy_wc,
            "mean_error": mean_error,
            "max_error": max_error,
            "pose_accepted": pose_accepted,
            "inliers_count": inliers_count,
            "t_world_camera_matrix": t_world_camera_matrix,
            "t_camera_world_matrix": t_camera_world_matrix,
        }
        self._log_result(result)

        if pose_accepted:
            self._publish_tf(msg.header.stamp, t_wc, quat_wc)
            self._save_result_yaml(result)
            status_text = "ACCEPTED"
            reason = ""
        else:
            reason = (
                "Reject pose: reprojection error too large. "
                "Check origin, axes, marker layout, marker size, square size, camera_info, corner order."
            )
            self.warn_throttled("reject_pose", reason, 1.0)
            if self.publish_bad_tf_for_debug:
                self._publish_tf(msg.header.stamp, t_wc, quat_wc)
            status_text = "REJECTED"

        self._draw_debug_image(
            annotated,
            aruco_corners,
            aruco_ids,
            charuco_corners,
            charuco_ids,
            projected_points,
            rvec,
            tvec,
            mean_error,
            max_error,
            status_text,
            reason,
            pose_accepted,
        )
        self._publish_debug(msg, annotated)

    def _log_detection_state(self, detected_aruco_ids, charuco_ids, object_points, image_points, points_source):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_detection_log < 2.0:
            return
        self._last_detection_log = now
        num_charuco = 0 if charuco_ids is None else len(charuco_ids)
        obj_shape = None if object_points is None else object_points.shape
        img_shape = None if image_points is None else image_points.shape
        self.get_logger().info(
            "\n"
            "========== Detection ==========\n"
            f"detected_aruco_ids: {detected_aruco_ids}\n"
            f"num_detected_aruco_markers: {len(detected_aruco_ids)}\n"
            f"num_interpolated_charuco_corners: {num_charuco}\n"
            f"points_source: {points_source}\n"
            f"object_points.shape: {obj_shape}\n"
            f"image_points.shape: {img_shape}\n"
            f"camera_matrix K:\n{self.camera_matrix}\n"
            f"distortion D: {self.dist_coeffs}\n"
            f"squares_x: {self.squares_x}, squares_y: {self.squares_y}\n"
            f"square_size: {self.square_size}, marker_size: {self.marker_size}\n"
            f"board_width: {self.board_width}, board_height: {self.board_height}\n"
            "==============================="
        )

    def _log_homography_filter(self, total_count, inlier_count, reason):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_detection_log > 0.25:
            return
        self.get_logger().info(
            "\n"
            "========== ChArUco planar filter ==========\n"
            f"total_points: {total_count}\n"
            f"homography_inliers: {inlier_count}\n"
            f"reason: {reason or '<ok>'}\n"
            "==========================================="
        )

    def _log_result(self, result):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_result_log < 2.0:
            return
        self._last_result_log = now
        t = result["t_wc"]
        rpy = result["rpy_wc"]
        q = result["quat_wc"]
        matrix_lines = self._format_matrix_lines(result["t_world_camera_matrix"])
        self.get_logger().info(
            "\n"
            "========== Extrinsic calibration result ==========\n"
            "TF:\n"
            f"    {self.world_frame} -> {self.camera_frame}\n\n"
            "T_world_camera:\n"
            f"{matrix_lines}\n\n"
            "camera_pose_in_world:\n"
            f"    x: {t[0]:.6f}\n"
            f"    y: {t[1]:.6f}\n"
            f"    z: {t[2]:.6f}\n"
            f"    roll: {rpy[0]:.6f}\n"
            f"    pitch: {rpy[1]:.6f}\n"
            f"    yaw: {rpy[2]:.6f}\n\n"
            "quaternion:\n"
            f"    qx: {q[0]:.6f}\n"
            f"    qy: {q[1]:.6f}\n"
            f"    qz: {q[2]:.6f}\n"
            f"    qw: {q[3]:.6f}\n\n"
            "reprojection_error:\n"
            f"    mean_px: {result['mean_error']:.4f}\n"
            f"    max_px: {result['max_error']:.4f}\n\n"
            "debug:\n"
            f"    points_source: {result['points_source']}\n"
            f"    used_points_count: {result['point_count']}\n"
            f"    homography_inliers: {result.get('homography_inlier_count', 0)}/{result.get('homography_total_count', 0)}\n"
            f"    ransac_inliers_count: {result['inliers_count']}\n"
            f"    pose_status: {'ACCEPTED' if result['pose_accepted'] else 'REJECTED'}\n\n"
            "saved_result:\n"
            f"    {self.result_yaml_path}\n"
            "================================================="
        )

    def _print_charuco_debug_log(self, charuco_debug):
        if not self.print_marker_debug or not charuco_debug:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_marker_debug_log < 2.0:
            return
        self._last_marker_debug_log = now
        lines = [
            "========== ChArUco corner correspondence debug ==========",
            "World: origin=center, X+=right, Y+=up, Z+=out_of_board",
        ]
        for item in charuco_debug[:40]:
            lines.append(
                f"ID {item['id']}: row_from_top={item['row_from_top']}, "
                f"col={item['col']}, "
                f"object={self._format_array(item['object'], 6)}, "
                f"image={self._format_array(item['image'], 2)}"
            )
        lines.append("=========================================================")
        self.get_logger().info("\n" + "\n".join(lines))

    def _print_marker_debug_log(self, marker_debug):
        if not self.print_marker_debug or not marker_debug:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_marker_debug_log < 2.0:
            return
        self._last_marker_debug_log = now
        lines = [
            "========== ArUco fallback marker correspondence debug ==========",
            "World: origin=center, X+=right, Y+=up, Z+=out_of_board",
            f"corner_order_mode: {self.corner_order_mode}",
        ]
        point_names = ["A", "B", "C", "D"]
        image_names = ["p0", "p1", "p2", "p3"]
        for item in marker_debug:
            lines.append(f"Marker ID {item['marker_id']}:")
            lines.append(f"    row={item['row']}, col={item['col']}")
            lines.append(f"    center_world = {self._format_array(item['center_world'], 6)}")
            lines.append("    object_points:")
            for name, point in zip(point_names, item["object_points"]):
                lines.append(f"        {name} = {self._format_array(point, 6)}")
            lines.append("    image_points:")
            for name, point in zip(image_names, item["image_points"]):
                lines.append(f"        {name} = {self._format_array(point, 2)}")
        lines.append("===============================================================")
        self.get_logger().info("\n" + "\n".join(lines))

    def _publish_debug(self, img_msg, annotated):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = img_msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f"publish debug image error: {exc}")

    def warn_throttled(self, key, text, period_sec):
        now = self.get_clock().now().nanoseconds / 1e9
        last = self._warn_times.get(key, 0.0)
        if now - last >= period_sec:
            self.get_logger().warn(text)
            self._warn_times[key] = now

    @staticmethod
    def _make_aruco_dictionary(dictionary_name):
        dictionary_id = getattr(cv2.aruco, str(dictionary_name), None)
        if dictionary_id is None:
            dictionary_id = cv2.aruco.DICT_4X4_50
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    @staticmethod
    def _make_detector_parameters():
        try:
            return cv2.aruco.DetectorParameters_create()
        except AttributeError:
            return cv2.aruco.DetectorParameters()

    @staticmethod
    def _float_list(values):
        return FlowList([float(x) for x in np.array(values).reshape(-1)])

    @staticmethod
    def _matrix_to_list(matrix):
        return [FlowList([float(x) for x in row]) for row in np.array(matrix)]

    @staticmethod
    def _format_array(values, precision=6):
        return "[" + ", ".join(f"{float(x):.{precision}f}" for x in np.array(values).reshape(-1)) + "]"

    @staticmethod
    def _format_matrix_lines(matrix, precision=6):
        rows = []
        for row in np.array(matrix):
            rows.append("    [" + " ".join(f"{float(x):.{precision}f}" for x in row) + "]")
        return "\n".join(rows)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoExtrinsicCalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
