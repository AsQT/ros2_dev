#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image


# =========================
# Cac thong so de sua nhanh
# =========================
ARUCO_DICTIONARY_NAME = "DICT_4X4_50"
DEFAULT_MARKER_SIZE = 0.05
DEFAULT_MARKER_DISTANCE_X = 0.345
DEFAULT_MARKER_DISTANCE_Y = 0.345

D = DEFAULT_MARKER_DISTANCE_X

# Bat buoc dat o dau file de de sua ID hoac khoang cach marker.
marker_world_centers = {
    0: [-D / 2.0, -D / 2.0, 0.0],  # ID 0: duoi trai anh
    1: [-D / 2.0, D / 2.0, 0.0],   # ID 1: tren trai anh
    2: [D / 2.0, -D / 2.0, 0.0],   # ID 2: duoi phai anh
    3: [D / 2.0, D / 2.0, 0.0],    # ID 3: tren phai anh
}

POINT_NAMES = [
    "top_left",
    "top_right",
    "bottom_right",
    "bottom_left",
    "center",
]


class FlowList(list):
    pass


def flow_list_representer(dumper, data):
    return dumper.represent_sequence(
        "tag:yaml.org,2002:seq",
        data,
        flow_style=True,
    )


yaml.SafeDumper.add_representer(FlowList, flow_list_representer)


class CalibCameraToWorldAruco5PointsNode(Node):
    def __init__(self):
        super().__init__("calib_camera_to_world_aruco_5points")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("debug_image_topic", "/vision/aruco_extrinsic_debug_image")
        self.declare_parameter("marker_size", DEFAULT_MARKER_SIZE)
        self.declare_parameter("marker_distance_x", DEFAULT_MARKER_DISTANCE_X)
        self.declare_parameter("marker_distance_y", DEFAULT_MARKER_DISTANCE_Y)
        self.declare_parameter("save_yaml_path", "config/Extrinsic_camera_to_world.yaml")
        self.declare_parameter("auto_save", True)
        self.declare_parameter("save_once", True)
        self.declare_parameter("require_all_markers", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.marker_distance_x = float(self.get_parameter("marker_distance_x").value)
        self.marker_distance_y = float(self.get_parameter("marker_distance_y").value)
        self.save_yaml_path = self.resolve_save_yaml_path(
            self.get_parameter("save_yaml_path").value
        )
        self.auto_save = bool(self.get_parameter("auto_save").value)
        self.save_once = bool(self.get_parameter("save_once").value)
        self.require_all_markers = bool(
            self.get_parameter("require_all_markers").value
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.saved_yaml = False
        self.last_waiting_camera_info_warn = 0.0
        self.last_not_enough_warn = 0.0
        self.last_missing_markers_warn = 0.0
        self.last_result_log = 0.0

        # Neu nguoi dung thay doi parameter khoang cach, cap nhat tam marker theo
        # dung dinh nghia world, nhung van giu dictionary mac dinh o dau file.
        self.marker_world_centers = self.build_marker_world_centers()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, ARUCO_DICTIONARY_NAME)
        )
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params,
            )

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
        self.pub_debug = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )

        self.get_logger().info("calib_camera_to_world_aruco_5points started")
        self.get_logger().info(f"Image topic        : {self.image_topic}")
        self.get_logger().info(f"CameraInfo topic   : {self.camera_info_topic}")
        self.get_logger().info(f"Debug image topic  : {self.debug_image_topic}")
        self.get_logger().info(f"Marker size        : {self.marker_size:.4f} m")
        self.get_logger().info(f"Marker distance X  : {self.marker_distance_x:.4f} m")
        self.get_logger().info(f"Marker distance Y  : {self.marker_distance_y:.4f} m")
        self.get_logger().info(f"Save YAML path     : {self.save_yaml_path}")

    def build_marker_world_centers(self):
        # He world tay phai: X sang phai anh, Y len tren anh, Z huong len khoi ban.
        lx = self.marker_distance_x
        wy = self.marker_distance_y
        return {
            0: [-lx / 2.0, -wy / 2.0, 0.0],
            1: [-lx / 2.0, wy / 2.0, 0.0],
            2: [lx / 2.0, -wy / 2.0, 0.0],
            3: [lx / 2.0, wy / 2.0, 0.0],
        }

    def resolve_save_yaml_path(self, path_value):
        path = Path(str(path_value)).expanduser()
        if path.is_absolute():
            return path

        # Uu tien source tree cua workspace hien tai de khop lenh:
        # cat ~/ros_vision/src/robot_vision_pipeline/config/Extrinsic_camera_to_world.yaml
        cwd_source_package = Path.cwd() / "src" / "robot_vision_pipeline"
        if cwd_source_package.is_dir():
            return cwd_source_package / path

        # Khi chay bang --symlink-install, __file__ tro ve source tree.
        package_root = Path(__file__).resolve().parents[2]
        source_config = package_root / path
        if package_root.name == "robot_vision_pipeline":
            return source_config

        return Path.cwd() / path

    def camera_info_callback(self, msg):
        # Lay noi tai truc tiep tu CameraInfo, khong doc Intrinsic.yaml.
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1)
        if self.dist_coeffs.size == 0:
            self.dist_coeffs = np.zeros((5,), dtype=np.float64)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge error: {exc}")
            return

        annotated = frame.copy()

        if self.camera_matrix is None or self.dist_coeffs is None:
            self.warn_throttled("camera_info", "Waiting for CameraInfo...", 2.0)
            self.draw_status(annotated, "Waiting for CameraInfo", ok=False)
            self.publish_debug(msg, annotated)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_markers(gray)

        detected_ids = []
        used_marker_ids = []
        object_points = []
        image_points = []
        calibration_points = []

        if ids is not None and len(ids) > 0:
            detected_ids = [int(x) for x in ids.flatten()]
            self.draw_detected_markers(annotated, corners, ids, detected_ids)

            for index, marker_id in enumerate(detected_ids):
                marker_corners = corners[index].reshape(4, 2).astype(np.float64)
                is_used = marker_id in self.marker_world_centers
                if is_used:
                    used_marker_ids.append(marker_id)
                    world_5pts = self.make_marker_world_points(marker_id)
                    pixel_5pts = self.make_marker_pixel_points(marker_corners)

                    for name, world_pt, pixel_pt in zip(
                        POINT_NAMES,
                        world_5pts,
                        pixel_5pts,
                    ):
                        object_points.append(world_pt)
                        image_points.append(pixel_pt)
                        calibration_points.append(
                            {
                                "marker_id": int(marker_id),
                                "point_name": name,
                                "world": self.float_list(world_pt),
                                "pixel": self.float_list(pixel_pt),
                            }
                        )

                    self.draw_marker_points(annotated, marker_corners, marker_id, True)
                else:
                    self.draw_marker_points(annotated, marker_corners, marker_id, False)

        used_marker_ids = sorted(used_marker_ids)
        point_count = len(object_points)
        required_marker_ids = sorted(self.marker_world_centers.keys())

        if self.require_all_markers and used_marker_ids != required_marker_ids:
            missing_marker_ids = [
                marker_id
                for marker_id in required_marker_ids
                if marker_id not in used_marker_ids
            ]
            self.warn_throttled(
                "missing_markers",
                "Not enough corner markers for calibration. "
                f"Required IDs: {required_marker_ids}, "
                f"used IDs: {used_marker_ids}, "
                f"missing IDs: {missing_marker_ids}",
                2.0,
            )
            self.draw_status(
                annotated,
                f"Missing marker IDs: {missing_marker_ids}",
                ok=False,
            )
            self.draw_summary(annotated, detected_ids, used_marker_ids, point_count)
            self.publish_debug(msg, annotated)
            return

        if point_count < 6:
            self.warn_throttled(
                "not_enough",
                "Not enough valid ArUco points for solvePnP",
                2.0,
            )
            self.draw_status(
                annotated,
                f"Not enough points: {point_count}",
                ok=False,
            )
            self.draw_summary(annotated, detected_ids, used_marker_ids, point_count)
            self.publish_debug(msg, annotated)
            return

        object_points_np = np.array(object_points, dtype=np.float64).reshape(-1, 3)
        image_points_np = np.array(image_points, dtype=np.float64).reshape(-1, 2)

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points_np,
                image_points_np,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        except Exception as exc:
            self.get_logger().warn(f"solvePnP failed with exception: {exc}")
            self.draw_status(annotated, "solvePnP failed", ok=False)
            self.draw_summary(annotated, detected_ids, used_marker_ids, point_count)
            self.publish_debug(msg, annotated)
            return

        if not success:
            self.get_logger().warn("solvePnP failed: OpenCV returned success=False")
            self.draw_status(annotated, "solvePnP failed", ok=False)
            self.draw_summary(annotated, detected_ids, used_marker_ids, point_count)
            self.publish_debug(msg, annotated)
            return

        R_world_to_camera, _ = cv2.Rodrigues(rvec)
        t_world_to_camera = tvec.reshape(3, 1)
        T_world_to_camera = np.eye(4, dtype=np.float64)
        T_world_to_camera[:3, :3] = R_world_to_camera
        T_world_to_camera[:3, 3] = t_world_to_camera.reshape(3)
        T_camera_to_world = np.linalg.inv(T_world_to_camera)
        R_camera_to_world = T_camera_to_world[:3, :3]
        t_camera_to_world = T_camera_to_world[:3, 3].reshape(3, 1)
        camera_z_axis_in_world = R_camera_to_world[:, 2]

        if float(camera_z_axis_in_world[2]) > 0.0:
            self.get_logger().warn(
                "WARNING: Camera optical Z is aligned with positive world Z. "
                "Z axis may be flipped."
            )

        reprojection_error_px = self.compute_reprojection_error(
            object_points_np,
            image_points_np,
            rvec,
            tvec,
        )

        self.draw_reprojected_points(
            annotated,
            object_points_np,
            rvec,
            tvec,
        )
        self.draw_world_axes(annotated, rvec, tvec)
        self.draw_status(annotated, "OK", ok=True)
        self.draw_summary(
            annotated,
            detected_ids,
            used_marker_ids,
            point_count,
            reprojection_error_px,
        )

        result = {
            "detected_ids": detected_ids,
            "used_marker_ids": used_marker_ids,
            "used_points_count": point_count,
            "camera_matrix": self.camera_matrix,
            "distortion_coefficients": self.dist_coeffs,
            "rvec_world_to_camera": rvec.reshape(3),
            "tvec_world_to_camera": tvec.reshape(3),
            "R_world_to_camera": R_world_to_camera,
            "t_world_to_camera": t_world_to_camera.reshape(3),
            "T_world_to_camera": T_world_to_camera,
            "R_camera_to_world": R_camera_to_world,
            "t_camera_to_world": t_camera_to_world.reshape(3),
            "T_camera_to_world": T_camera_to_world,
            "camera_z_axis_in_world": camera_z_axis_in_world,
            "reprojection_error_px": reprojection_error_px,
            "calibration_points": calibration_points,
        }

        self.log_result(result)

        if self.auto_save and (not self.save_once or not self.saved_yaml):
            self.save_yaml(result)
            self.saved_yaml = True

        self.publish_debug(msg, annotated)

    def detect_markers(self, gray):
        try:
            if self.aruco_detector is not None:
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray,
                    self.aruco_dict,
                    parameters=self.aruco_params,
                )
            return corners, ids
        except Exception as exc:
            self.get_logger().error(f"ArUco detect error: {exc}")
            return [], None

    def make_marker_world_points(self, marker_id):
        cx, cy, cz = self.marker_world_centers[marker_id]
        half = self.marker_size / 2.0
        return [
            [cx - half, cy + half, cz],
            [cx + half, cy + half, cz],
            [cx + half, cy - half, cz],
            [cx - half, cy - half, cz],
            [cx, cy, cz],
        ]

    @staticmethod
    def make_marker_pixel_points(marker_corners):
        center = np.mean(marker_corners, axis=0)
        return [
            marker_corners[0].tolist(),
            marker_corners[1].tolist(),
            marker_corners[2].tolist(),
            marker_corners[3].tolist(),
            center.tolist(),
        ]

    def compute_reprojection_error(self, object_points, image_points, rvec, tvec):
        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        projected_points = projected_points.reshape(-1, 2)
        errors = np.linalg.norm(projected_points - image_points, axis=1)
        return float(np.mean(errors))

    def draw_detected_markers(self, annotated, corners, ids, detected_ids):
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        for index, marker_id in enumerate(detected_ids):
            pts = corners[index].reshape(4, 2).astype(int)
            center = np.mean(pts, axis=0).astype(int)
            color = (0, 220, 0) if marker_id in self.marker_world_centers else (0, 165, 255)
            cv2.putText(
                annotated,
                f"ID {marker_id}",
                (int(center[0]) + 8, int(center[1]) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

    def draw_marker_points(self, annotated, marker_corners, marker_id, is_used):
        color = (0, 255, 0) if is_used else (0, 165, 255)
        radius = 5 if is_used else 4
        thickness = -1
        center = np.mean(marker_corners, axis=0)

        # Ve 4 goc va tam marker. Marker dung calib mau xanh, marker bo qua mau cam.
        for point in marker_corners:
            cv2.circle(
                annotated,
                (int(point[0]), int(point[1])),
                radius,
                color,
                thickness,
            )
        cv2.circle(
            annotated,
            (int(center[0]), int(center[1])),
            radius + 2,
            (255, 0, 255) if is_used else color,
            thickness,
        )

        label = "USED" if is_used else "SKIP"
        cv2.putText(
            annotated,
            f"{label} {marker_id}",
            (int(center[0]) + 8, int(center[1]) + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )

    def draw_reprojected_points(self, annotated, object_points, rvec, tvec):
        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        for point in projected_points.reshape(-1, 2):
            cv2.drawMarker(
                annotated,
                (int(point[0]), int(point[1])),
                (255, 0, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=12,
                thickness=2,
            )

    def draw_world_axes(self, annotated, rvec, tvec):
        try:
            cv2.drawFrameAxes(
                annotated,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_size * 1.5,
            )
        except Exception as exc:
            self.get_logger().warn(f"drawFrameAxes failed: {exc}")

    @staticmethod
    def draw_status(annotated, text, ok):
        color = (0, 220, 0) if ok else (0, 0, 255)
        cv2.putText(
            annotated,
            f"Status: {text}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def draw_summary(
        annotated,
        detected_ids,
        used_marker_ids,
        point_count,
        reprojection_error_px=None,
    ):
        lines = [
            f"Detected IDs: {detected_ids}",
            f"Used IDs: {used_marker_ids}",
            f"solvePnP points: {point_count}",
        ]
        if reprojection_error_px is not None:
            lines.append(f"Reprojection error: {reprojection_error_px:.3f} px")

        y = 65
        for line in lines:
            cv2.putText(
                annotated,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            y += 26

    def save_yaml(self, result):
        data = {
            "camera_frame": "camera_color_optical_frame",
            "world_frame": "world",
            "image_topic": self.image_topic,
            "camera_info_topic": self.camera_info_topic,
            "aruco_dictionary": ARUCO_DICTIONARY_NAME,
            "marker_size_m": float(self.marker_size),
            "marker_distance_x_m": float(self.marker_distance_x),
            "marker_distance_y_m": float(self.marker_distance_y),
            "world_definition": {
                "origin": "Tam vung mau trang o giua workspace",
                "x_axis": "Huong tu tam workspace sang phai anh",
                "y_axis": "Huong tu tam workspace len hang marker phia tren",
                "z_axis": "Vuong goc mat ban, huong len khoi mat ban",
                "handedness": "right-handed",
                "plane": "Mat ban va cac marker ArUco nam tren mat phang Z_world = 0",
            },
            "marker_world_centers": {
                int(marker_id): self.float_list(center)
                for marker_id, center in self.marker_world_centers.items()
            },
            "used_marker_ids": [int(x) for x in result["used_marker_ids"]],
            "used_points_count": int(result["used_points_count"]),
            "camera_matrix": self.matrix_to_list(result["camera_matrix"]),
            "distortion_coefficients": self.float_list(
                result["distortion_coefficients"]
            ),
            "rvec_world_to_camera": self.float_list(result["rvec_world_to_camera"]),
            "tvec_world_to_camera": self.float_list(result["tvec_world_to_camera"]),
            "R_world_to_camera": self.matrix_to_list(result["R_world_to_camera"]),
            "t_world_to_camera": self.float_list(result["t_world_to_camera"]),
            "T_world_to_camera": self.matrix_to_list(result["T_world_to_camera"]),
            "R_camera_to_world": self.matrix_to_list(result["R_camera_to_world"]),
            "t_camera_to_world": self.float_list(result["t_camera_to_world"]),
            "T_camera_to_world": self.matrix_to_list(result["T_camera_to_world"]),
            "camera_z_axis_in_world": self.float_list(
                result["camera_z_axis_in_world"]
            ),
            "reprojection_error_px": float(result["reprojection_error_px"]),
            "calibration_points": result["calibration_points"],
        }

        try:
            self.save_yaml_path.parent.mkdir(parents=True, exist_ok=True)
            with self.save_yaml_path.open("w", encoding="utf-8") as file:
                yaml.safe_dump(
                    data,
                    file,
                    sort_keys=False,
                    default_flow_style=False,
                    allow_unicode=True,
                )
            self.get_logger().info(f"Saved extrinsic YAML: {self.save_yaml_path}")
        except Exception as exc:
            self.get_logger().error(f"Cannot save YAML {self.save_yaml_path}: {exc}")

    def log_result(self, result):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_result_log < 2.0:
            return
        self.last_result_log = now

        self.get_logger().info(
            "\n"
            "========== ArUco camera-to-world calibration ==========\n"
            f"So marker detect duoc     : {len(result['detected_ids'])}\n"
            f"ID marker detect duoc     : {result['detected_ids']}\n"
            f"ID marker dung calib      : {result['used_marker_ids']}\n"
            f"So diem dung solvePnP     : {result['used_points_count']}\n"
            f"camera_matrix K:\n{result['camera_matrix']}\n"
            f"distortion_coefficients D : {result['distortion_coefficients']}\n"
            f"rvec_world_to_camera      : {result['rvec_world_to_camera']}\n"
            f"tvec_world_to_camera      : {result['tvec_world_to_camera']}\n"
            f"T_world_to_camera:\n{result['T_world_to_camera']}\n"
            f"T_camera_to_world:\n{result['T_camera_to_world']}\n"
            f"camera_z_axis_in_world : {result['camera_z_axis_in_world']}\n"
            f"reprojection_error_px     : {result['reprojection_error_px']:.4f}\n"
            "======================================================="
        )

    def warn_throttled(self, key, text, period_sec):
        now = self.get_clock().now().nanoseconds / 1e9
        attr = f"last_{key}_warn"
        last = getattr(self, attr, 0.0)
        if now - last >= period_sec:
            self.get_logger().warn(text)
            setattr(self, attr, now)

    def publish_debug(self, img_msg, annotated):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = img_msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as exc:
            self.get_logger().error(f"publish debug image error: {exc}")

    @staticmethod
    def float_list(values):
        return FlowList([float(x) for x in np.array(values).reshape(-1)])

    @staticmethod
    def matrix_to_list(matrix):
        return [FlowList([float(x) for x in row]) for row in np.array(matrix)]


def main(args=None):
    rclpy.init(args=args)
    node = CalibCameraToWorldAruco5PointsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
