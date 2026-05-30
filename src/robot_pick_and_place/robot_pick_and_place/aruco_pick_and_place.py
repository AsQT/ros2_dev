#!/usr/bin/env python3

import sys
import math
from collections import deque
from typing import Optional, Tuple, Dict

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from robot_task_manager.action import PickPlace

from PyQt6 import QtCore, QtGui, QtWidgets


# ============================================================
# CONFIG
# ============================================================
RGB_IMAGE_TOPIC = "/astra/rgb/image_raw"
DEPTH_IMAGE_TOPIC = "/astra/depth/image_raw"

RGB_INFO_TOPIC = "/astra/rgb/camera_info"
DEPTH_INFO_TOPIC = "/astra/depth/camera_info"

TARGET_FRAME = "world"
CAMERA_TF_FRAME = "astra_link_optical"

ACTION_NAME = "/pickplace"

DICTIONARY_NAME = "DICT_4X4_50"
MARKER_SIZE_M = 0.04

# TCP/gripper chúc xuống, giống lệnh test của bạn
PICK_QX = 0.7071
PICK_QY = 0.7071
PICK_QZ = 0.0
PICK_QW = 0.0

# True: q_pick = Rz(yaw_robot) * q_down
# False: giữ hướng cố định
USE_YAW_FOR_GRASP = True

# ============================================================
# XYZ METHOD
# ============================================================
# True: lấy XYZ từ tâm ArUco pixel + depth, KHÔNG lấy XYZ từ solvePnP/tvec nữa.
# False: fallback dùng solvePnP/tvec kiểu cũ.
USE_DEPTH_FOR_XYZ = True

# Cửa sổ lấy median depth quanh tâm marker.
DEPTH_RADIUS = 2

# Lọc trung bình nhiều frame giống YOLO để tọa độ đỡ nhảy.
AVG_FRAMES = 5

# Nếu depth sau khi TF ra Z là mặt trên vật, thường không cần trừ nhiều.
# Nếu robot còn chạm cao/thấp thì chỉnh dòng này.
USE_DETECTED_Z = True
FIXED_PICK_Z = 0.035
Z_PICK_CORRECTION = 0.000

# Offset từ TÂM MARKER tới ĐIỂM GẮP trong hệ XY của marker/vật.
# Đơn vị mét. Mặc định để 0 để test đúng tâm ArUco trước.
# Nếu marker không dán đúng tâm vật thì chỉnh 2 dòng này sau.
MARKER_TO_PICK_X_M = 0.000
MARKER_TO_PICK_Y_M = 0.000
MARKER_TO_PICK_Z_M = 0.000

PLACE_X = 0.30
PLACE_Y = 0.00
PLACE_Z = 0.045

GRIPPER_CLOSE = 0.01
VELOCITY_SCALE = 0.3

# Offset giữa trục ArUco và trục đầu kẹp/vật.
# Nếu yaw còn lệch 90 độ thì đổi 90.0 hoặc -90.0.
YAW_OFFSET_DEG = 90.0

X_MIN = 0.10
X_MAX = 0.65
Y_MIN = -0.35
Y_MAX = 0.35


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
}


# ============================================================
# Helpers
# ============================================================
def normalize_axis_deg(a: float) -> float:
    # Góc trục: 0 và 180 độ là cùng hướng kẹp.
    return float((float(a) + 90.0) % 180.0 - 90.0)


def normalize_quat_xyzw(q) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))

    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    return q / n


def mean_axis_deg(vals) -> Optional[float]:
    good = []

    for v in vals:
        if v is None:
            continue
        if not np.isfinite(v):
            continue
        good.append(float(v))

    if not good:
        return None

    sum_c = 0.0
    sum_s = 0.0

    for a in good:
        r = math.radians(a)
        sum_c += math.cos(2.0 * r)
        sum_s += math.sin(2.0 * r)

    yaw = 0.5 * math.degrees(math.atan2(sum_s, sum_c))
    return normalize_axis_deg(yaw)


def aruco_x_axis_yaw_img_deg(pts4: np.ndarray) -> float:
    """
    Lấy hướng trục X của marker trên ảnh từ corner[0] -> corner[1].
    OpenCV ArUco trả corners theo thứ tự quanh marker, nên vector này có hướng ổn định theo ID.
    """
    pts = np.asarray(pts4, dtype=np.float32).reshape(4, 2)
    dx = float(pts[1, 0] - pts[0, 0])
    dy = float(pts[1, 1] - pts[0, 1])

    if math.hypot(dx, dy) < 1e-6:
        return 0.0

    return normalize_axis_deg(math.degrees(math.atan2(dy, dx)))


def grasp_quat_from_robot_yaw_deg(yaw_robot_deg: float) -> Tuple[float, float, float, float]:
    """
    q_pick = Rz(yaw_robot) * q_gripper_down
    """
    yaw_robot_deg = normalize_axis_deg(float(yaw_robot_deg))

    r_yaw = R.from_euler("z", yaw_robot_deg, degrees=True)
    r_down = R.from_quat([PICK_QX, PICK_QY, PICK_QZ, PICK_QW])

    q = (r_yaw * r_down).as_quat()
    q = normalize_quat_xyzw(q)

    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def bgr_to_qimage(bgr: np.ndarray) -> QtGui.QImage:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = np.ascontiguousarray(rgb)

    h, w, ch = rgb.shape
    return QtGui.QImage(
        rgb.data,
        w,
        h,
        ch * w,
        QtGui.QImage.Format.Format_RGB888,
    ).copy()


def rosimg_to_depth_m(msg: Image, bridge: CvBridge) -> np.ndarray:
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    depth = np.asarray(depth)

    enc = (msg.encoding or "").lower()

    if enc in ("16uc1", "mono16") or depth.dtype == np.uint16:
        return depth.astype(np.float32) / 1000.0

    if enc == "32fc1" or depth.dtype == np.float32:
        return depth.astype(np.float32)

    # Fallback: nếu driver trả kiểu lạ, vẫn cố ép float.
    return depth.astype(np.float32)


# ============================================================
# ROS Node
# ============================================================
class ArucoPickGuiNode(Node):
    def __init__(self):
        super().__init__("aruco_pick_place_gui_node")

        self.bridge = CvBridge()

        self.image_topic = RGB_IMAGE_TOPIC
        self.depth_topic = DEPTH_IMAGE_TOPIC
        self.rgb_info_topic = RGB_INFO_TOPIC
        self.depth_info_topic = DEPTH_INFO_TOPIC
        self.base_frame = TARGET_FRAME
        self.camera_tf_frame = CAMERA_TF_FRAME
        self.action_name = ACTION_NAME

        self.marker_size = float(MARKER_SIZE_M)

        self.rgb_camera_matrix = None
        self.rgb_dist_coeffs = None
        self.rgb_width = None
        self.rgb_height = None

        self.depth_info = {
            "fx": None,
            "fy": None,
            "cx": None,
            "cy": None,
            "width": None,
            "height": None,
            "frame_id": "",
        }

        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_shape: Optional[Tuple[int, int]] = None

        self.latest_annotated: Optional[np.ndarray] = None
        self.latest_detection: Optional[Dict] = None
        self.latest_count = 0
        self.avg_buffers: Dict[int, deque] = {}

        # ============================================================
        # ArUco setup
        # ============================================================
        if DICTIONARY_NAME not in ARUCO_DICTS:
            self.get_logger().warn(
                f"Unknown dictionary {DICTIONARY_NAME}, fallback DICT_4X4_50"
            )
            dict_id = ARUCO_DICTS["DICT_4X4_50"]
        else:
            dict_id = ARUCO_DICTS[DICTIONARY_NAME]

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # ============================================================
        # TF + Action
        # ============================================================
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pickplace_client = ActionClient(
            self,
            PickPlace,
            self.action_name,
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_rgb_info = self.create_subscription(
            CameraInfo,
            self.rgb_info_topic,
            self.rgb_info_callback,
            qos_sensor,
        )

        self.sub_depth_info = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.depth_info_callback,
            qos_sensor,
        )

        self.sub_depth = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_sensor,
        )

        self.sub_img = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_sensor,
        )

        self.get_logger().info("Aruco Pick GUI started - DEPTH XYZ mode")
        self.get_logger().info(f"RGB topic        : {self.image_topic}")
        self.get_logger().info(f"Depth topic      : {self.depth_topic}")
        self.get_logger().info(f"RGB CameraInfo   : {self.rgb_info_topic}")
        self.get_logger().info(f"Depth CameraInfo : {self.depth_info_topic}")
        self.get_logger().info(f"Dictionary       : {DICTIONARY_NAME}")
        self.get_logger().info(f"Marker size      : {self.marker_size} m")
        self.get_logger().info(f"Base frame       : {self.base_frame}")
        self.get_logger().info(f"Camera TF frame  : {self.camera_tf_frame}")
        self.get_logger().info(f"Action           : {self.action_name}")

    # ------------------------------------------------------------
    # Camera info + depth
    # ------------------------------------------------------------
    def rgb_info_callback(self, msg: CameraInfo):
        self.rgb_camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if len(msg.d) > 0:
            self.rgb_dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.rgb_dist_coeffs = np.zeros((5,), dtype=np.float64)

        self.rgb_width = int(msg.width)
        self.rgb_height = int(msg.height)

    def depth_info_callback(self, msg: CameraInfo):
        self.depth_info["fx"] = float(msg.k[0])
        self.depth_info["cx"] = float(msg.k[2])
        self.depth_info["fy"] = float(msg.k[4])
        self.depth_info["cy"] = float(msg.k[5])
        self.depth_info["width"] = int(msg.width)
        self.depth_info["height"] = int(msg.height)
        self.depth_info["frame_id"] = msg.header.frame_id

    def depth_callback(self, msg: Image):
        try:
            depth_m = rosimg_to_depth_m(msg, self.bridge)
        except Exception as e:
            self.get_logger().warn(f"Depth convert failed: {e}")
            return

        if depth_m.ndim != 2:
            self.get_logger().warn(f"Depth image ndim invalid: {depth_m.ndim}")
            return

        self.latest_depth = depth_m
        self.latest_depth_shape = depth_m.shape[:2]

    def depth_info_ready(self) -> bool:
        return (
            self.depth_info["fx"] is not None
            and self.depth_info["fy"] is not None
            and self.depth_info["cx"] is not None
            and self.depth_info["cy"] is not None
        )

    # ------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------
    def lookup_camera_tf(self, source_frame: Optional[str] = None):
        if source_frame is None or source_frame == "":
            source_frame = self.depth_info.get("frame_id") or self.camera_tf_frame

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )

            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation

            rot = R.from_quat([q.x, q.y, q.z, q.w])
            trans = np.array([t.x, t.y, t.z], dtype=float)

            return rot, trans

        except Exception as e:
            if source_frame != self.camera_tf_frame:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.camera_tf_frame,
                        Time(),
                        timeout=Duration(seconds=0.05),
                    )
                    t = tf_msg.transform.translation
                    q = tf_msg.transform.rotation

                    rot = R.from_quat([q.x, q.y, q.z, q.w])
                    trans = np.array([t.x, t.y, t.z], dtype=float)

                    self.get_logger().warn(
                        f"TF lookup failed for {source_frame}, falling back to {self.camera_tf_frame}"
                    )
                    return rot, trans
                except Exception as e2:
                    self.get_logger().warn(
                        f"TF failed: {source_frame} -> {self.base_frame}: {e}; fallback {self.camera_tf_frame} -> {self.base_frame}: {e2}"
                    )
                    return None, None

            self.get_logger().warn(
                f"TF failed: {source_frame} -> {self.base_frame}: {e}"
            )
            return None, None

    def camera_xyz_to_robot_xyz(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        rot, trans = self.lookup_camera_tf()

        if rot is None or trans is None:
            return None, None, None

        p_cam = np.array([float(x), float(y), float(z)], dtype=float)
        p_robot = rot.apply(p_cam) + trans

        return float(p_robot[0]), float(p_robot[1]), float(p_robot[2])

    # ------------------------------------------------------------
    # Depth XYZ helpers
    # ------------------------------------------------------------
    def rgb_px_to_depth_px(
        self,
        u_rgb: float,
        v_rgb: float,
        rgb_w: int,
        rgb_h: int,
    ) -> Tuple[int, int]:
        if self.latest_depth is None:
            return int(round(u_rgb)), int(round(v_rgb))

        depth_h, depth_w = self.latest_depth.shape[:2]

        u_depth = int(round(float(u_rgb) * float(depth_w) / float(rgb_w)))
        v_depth = int(round(float(v_rgb) * float(depth_h) / float(rgb_h)))

        u_depth = max(0, min(depth_w - 1, u_depth))
        v_depth = max(0, min(depth_h - 1, v_depth))

        return u_depth, v_depth

    def median_depth_at(self, u: int, v: int) -> Optional[float]:
        if self.latest_depth is None:
            return None

        depth = self.latest_depth
        h, w = depth.shape[:2]

        if not (0 <= u < w and 0 <= v < h):
            return None

        r = int(max(0, DEPTH_RADIUS))
        roi = depth[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1),
        ]

        valid = roi[np.isfinite(roi) & (roi > 0.02)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def depth_pixel_to_camera_xyz(
        self,
        u_depth: int,
        v_depth: int,
        zc: float,
    ) -> Tuple[float, float, float]:
        fx = float(self.depth_info["fx"])
        fy = float(self.depth_info["fy"])
        cx = float(self.depth_info["cx"])
        cy = float(self.depth_info["cy"])

        xc = (float(u_depth) - cx) * float(zc) / fx
        yc = (float(v_depth) - cy) * float(zc) / fy

        return float(xc), float(yc), float(zc)

    def marker_center_depth_to_robot_xyz(
        self,
        cx_rgb: float,
        cy_rgb: float,
        rgb_w: int,
        rgb_h: int,
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[int], Optional[int]]:
        """
        Cách mới:
            center ArUco trên ảnh RGB -> scale sang depth pixel
            -> lấy median depth
            -> pinhole depth -> camera XYZ
            -> TF camera -> robot/world XYZ

        Không dùng tvec của solvePnP cho XYZ nữa.
        """
        if self.latest_depth is None or not self.depth_info_ready():
            return None, None, None, None, None, None, None, None

        u_depth, v_depth = self.rgb_px_to_depth_px(cx_rgb, cy_rgb, rgb_w, rgb_h)
        zc = self.median_depth_at(u_depth, v_depth)

        if zc is None:
            return None, None, None, None, None, None, u_depth, v_depth

        xc, yc, zc = self.depth_pixel_to_camera_xyz(u_depth, v_depth, zc)
        xr, yr, zr = self.camera_xyz_to_robot_xyz(xc, yc, zc)

        return xc, yc, zc, xr, yr, zr, u_depth, v_depth

    def transform_pixel_yaw_to_robot_yaw(
        self,
        cx_rgb: float,
        cy_rgb: float,
        yaw_img_deg: float,
        zc: float,
        rgb_w: int,
        rgb_h: int,
        step_rgb_px: float = 60.0,
    ) -> Optional[float]:
        """
        Đổi yaw trên ảnh sang yaw robot bằng TF, giống cách YOLO đang làm.

        P1 = tâm ArUco.
        P2 = P1 + vector theo yaw ảnh.
        P1/P2 RGB pixel -> depth pixel -> camera XYZ -> robot XYZ.
        yaw_robot = atan2(P2_robot.y - P1_robot.y, P2_robot.x - P1_robot.x)
        """
        if zc is None or not np.isfinite(zc) or zc <= 0.02:
            return None

        if not self.depth_info_ready():
            return None

        yaw_rad = math.radians(float(yaw_img_deg))

        u1_rgb = float(cx_rgb)
        v1_rgb = float(cy_rgb)
        u2_rgb = u1_rgb + float(step_rgb_px) * math.cos(yaw_rad)
        v2_rgb = v1_rgb + float(step_rgb_px) * math.sin(yaw_rad)

        u1_depth, v1_depth = self.rgb_px_to_depth_px(u1_rgb, v1_rgb, rgb_w, rgb_h)
        u2_depth, v2_depth = self.rgb_px_to_depth_px(u2_rgb, v2_rgb, rgb_w, rgb_h)

        p1_cam = np.array(self.depth_pixel_to_camera_xyz(u1_depth, v1_depth, zc), dtype=float)
        p2_cam = np.array(self.depth_pixel_to_camera_xyz(u2_depth, v2_depth, zc), dtype=float)

        rot, trans = self.lookup_camera_tf()

        if rot is None or trans is None:
            return None

        p1_robot = rot.apply(p1_cam) + trans
        p2_robot = rot.apply(p2_cam) + trans

        dx = float(p2_robot[0] - p1_robot[0])
        dy = float(p2_robot[1] - p1_robot[1])

        if math.hypot(dx, dy) < 1e-9:
            return None

        yaw_robot = math.degrees(math.atan2(dy, dx))
        return normalize_axis_deg(yaw_robot + float(YAW_OFFSET_DEG))

    def apply_marker_to_pick_offset(
        self,
        xr: float,
        yr: float,
        zr: float,
        yaw_robot_deg: float,
    ) -> Tuple[float, float, float]:
        """
        Offset từ tâm marker sang điểm gắp.
        Dùng yaw_robot để xoay offset theo hướng vật.
        Mặc định offset = 0 nên không ảnh hưởng.
        """
        dx = float(MARKER_TO_PICK_X_M)
        dy = float(MARKER_TO_PICK_Y_M)
        dz = float(MARKER_TO_PICK_Z_M)

        if abs(dx) < 1e-12 and abs(dy) < 1e-12 and abs(dz) < 1e-12:
            return float(xr), float(yr), float(zr)

        yaw = math.radians(float(yaw_robot_deg))

        off_x = math.cos(yaw) * dx - math.sin(yaw) * dy
        off_y = math.sin(yaw) * dx + math.cos(yaw) * dy

        return float(xr + off_x), float(yr + off_y), float(zr + dz)

    # ------------------------------------------------------------
    # Optional solvePnP fallback/debug
    # ------------------------------------------------------------
    def estimate_pose_solvepnp(self, corners):
        if self.rgb_camera_matrix is None or self.rgb_dist_coeffs is None:
            return None, None

        s = self.marker_size / 2.0

        obj_points = np.array(
            [
                [-s,  s, 0.0],
                [ s,  s, 0.0],
                [ s, -s, 0.0],
                [-s, -s, 0.0],
            ],
            dtype=np.float32,
        )

        img_points = corners.reshape(4, 2).astype(np.float32)

        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.rgb_camera_matrix,
                self.rgb_dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except Exception:
            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.rgb_camera_matrix,
                self.rgb_dist_coeffs,
            )

        if not ok:
            return None, None

        return rvec, tvec

    def solvepnp_xyz_to_robot(self, corners):
        rvec, tvec = self.estimate_pose_solvepnp(corners)

        if rvec is None or tvec is None:
            return None, None, None, None, None, None

        cam_x = float(tvec[0][0])
        cam_y = float(tvec[1][0])
        cam_z = float(tvec[2][0])
        robot_x, robot_y, robot_z = self.camera_xyz_to_robot_xyz(cam_x, cam_y, cam_z)

        return cam_x, cam_y, cam_z, robot_x, robot_y, robot_z

    # ------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if frame is None:
            return

        rgb_h, rgb_w = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        annotated = frame.copy()

        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )
        except Exception as e:
            self.get_logger().error(f"aruco detect error: {e}")
            return

        best_det = None
        count = 0

        if ids is not None and len(ids) > 0:
            count = len(ids)
            ids_flat = ids.flatten()

            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            for i, marker_id in enumerate(ids_flat):
                pts = corners[i].reshape(4, 2)

                cx_rgb = float(np.mean(pts[:, 0]))
                cy_rgb = float(np.mean(pts[:, 1]))

                yaw_img_deg = aruco_x_axis_yaw_img_deg(pts)

                xc = yc = zc = None
                robot_x = robot_y = robot_z = None
                u_depth = v_depth = None

                if USE_DEPTH_FOR_XYZ:
                    (
                        xc,
                        yc,
                        zc,
                        robot_x,
                        robot_y,
                        robot_z,
                        u_depth,
                        v_depth,
                    ) = self.marker_center_depth_to_robot_xyz(
                        cx_rgb,
                        cy_rgb,
                        rgb_w,
                        rgb_h,
                    )
                else:
                    xc, yc, zc, robot_x, robot_y, robot_z = self.solvepnp_xyz_to_robot(corners[i])

                if robot_x is None or robot_y is None or robot_z is None:
                    continue

                yaw_robot_deg = self.transform_pixel_yaw_to_robot_yaw(
                    cx_rgb,
                    cy_rgb,
                    yaw_img_deg,
                    zc,
                    rgb_w,
                    rgb_h,
                )

                if yaw_robot_deg is None:
                    # Fallback nếu thiếu depth/TF.
                    yaw_robot_deg = normalize_axis_deg(yaw_img_deg + float(YAW_OFFSET_DEG))

                # Nếu marker không ở tâm vật, offset sẽ dịch điểm pick theo hướng vật.
                pick_x, pick_y, pick_z = self.apply_marker_to_pick_offset(
                    robot_x,
                    robot_y,
                    robot_z,
                    yaw_robot_deg,
                )

                raw_det = {
                    "id": int(marker_id),
                    "center_px": (cx_rgb, cy_rgb),
                    "depth_px": (u_depth, v_depth),
                    "marker_robot_xyz": (robot_x, robot_y, robot_z),
                    "robot_xyz": (pick_x, pick_y, pick_z),
                    "yaw_img_deg": float(yaw_img_deg),
                    "yaw_robot_deg": float(yaw_robot_deg),
                }

                # Average theo ID marker.
                mid = int(marker_id)
                if mid not in self.avg_buffers:
                    self.avg_buffers[mid] = deque(maxlen=AVG_FRAMES)

                self.avg_buffers[mid].append(raw_det)
                buf = list(self.avg_buffers[mid])

                avg_yaw = mean_axis_deg([d["yaw_robot_deg"] for d in buf])
                if avg_yaw is None:
                    avg_yaw = float(yaw_robot_deg)

                avg_det = dict(raw_det)
                avg_det["robot_xyz"] = (
                    float(np.mean([d["robot_xyz"][0] for d in buf])),
                    float(np.mean([d["robot_xyz"][1] for d in buf])),
                    float(np.mean([d["robot_xyz"][2] for d in buf])),
                )
                avg_det["marker_robot_xyz"] = (
                    float(np.mean([d["marker_robot_xyz"][0] for d in buf])),
                    float(np.mean([d["marker_robot_xyz"][1] for d in buf])),
                    float(np.mean([d["marker_robot_xyz"][2] for d in buf])),
                )
                avg_det["yaw_robot_deg"] = float(avg_yaw)

                if best_det is None:
                    best_det = avg_det

                px = int(round(cx_rgb))
                py = int(round(cy_rgb))
                ax, ay, az = avg_det["robot_xyz"]

                cv2.circle(annotated, (px, py), 5, (0, 0, 255), -1)

                # Mũi tên yaw ảnh.
                ex = int(px + 50 * math.cos(math.radians(yaw_img_deg)))
                ey = int(py + 50 * math.sin(math.radians(yaw_img_deg)))
                cv2.arrowedLine(
                    annotated,
                    (px, py),
                    (ex, ey),
                    (255, 0, 0),
                    2,
                    tipLength=0.25,
                )

                cv2.putText(
                    annotated,
                    f"id:{marker_id} X:{ax:.3f} Y:{ay:.3f} Z:{az:.3f}",
                    (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

                cv2.putText(
                    annotated,
                    f"yaw_robot:{avg_det['yaw_robot_deg']:.1f}",
                    (px + 10, py + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

        self.latest_count = count
        self.latest_detection = best_det

        cv2.putText(
            annotated,
            f"ArUco det:{count}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        if self.latest_depth is None:
            cv2.putText(
                annotated,
                "NO DEPTH YET",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        self.latest_annotated = annotated


# ============================================================
# GUI chỉ có 1 nút GẮP
# ============================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: ArucoPickGuiNode):
        super().__init__()

        self.node = node

        self.setWindowTitle("ArUco PickPlace - Depth XYZ")
        self.resize(1100, 720)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        self.image_label = QtWidgets.QLabel("Waiting image...")
        self.image_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(820, 620)
        self.image_label.setStyleSheet("background:#111; color:#ddd;")

        self.btn_pick = QtWidgets.QPushButton("GẮP")
        self.btn_pick.setMinimumHeight(90)
        self.btn_pick.setStyleSheet(
            "font-size: 32px; font-weight: bold; background: #2ecc71;"
        )
        self.btn_pick.clicked.connect(self.send_pickplace)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(240)
        self.log_box.setStyleSheet(
            "background:#f4f4f4; font-family: monospace; font-size: 13px;"
        )

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self.btn_pick)
        right.addWidget(QtWidgets.QLabel("Log:"))
        right.addWidget(self.log_box)

        layout = QtWidgets.QHBoxLayout(central)
        layout.addWidget(self.image_label, 4)
        layout.addLayout(right, 1)

        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros_once)
        self.spin_timer.start(10)

        self.gui_timer = QtCore.QTimer(self)
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(40)

        self.append_log("[GUI] Started. ArUco lấy tâm/id/yaw, XYZ lấy từ depth.")

    def append_log(self, text: str):
        self.log_box.appendPlainText(text)
        scrollbar = self.log_box.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

        try:
            self.node.get_logger().info(text)
        except Exception:
            pass

    def spin_ros_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def update_gui(self):
        img = self.node.latest_annotated

        if img is not None:
            qimg = bgr_to_qimage(img)
            pix = QtGui.QPixmap.fromImage(qimg)

            self.image_label.setPixmap(
                pix.scaled(
                    self.image_label.size(),
                    QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                    QtCore.Qt.TransformationMode.SmoothTransformation,
                )
            )

        if self.node.latest_detection is None:
            self.btn_pick.setText("GẮP\n(chưa detect/depth)")
        else:
            d = self.node.latest_detection
            x, y, z = d["robot_xyz"]
            self.btn_pick.setText(f"GẮP\nX:{x:.3f} Y:{y:.3f} Z:{z:.3f}")

    def send_pickplace(self):
        det = self.node.latest_detection

        if det is None:
            self.append_log("[PickPlace] Chưa detect được ArUco + depth hợp lệ")
            return

        if not self.node.pickplace_client.wait_for_server(timeout_sec=1.0):
            self.append_log("[PickPlace] ERROR: /pickplace action server chưa chạy")
            return

        marker_id = int(det["id"])
        x, y, z_detect = det["robot_xyz"]
        yaw_robot = float(det["yaw_robot_deg"])

        if x < X_MIN or x > X_MAX or y < Y_MIN or y > Y_MAX:
            self.append_log(
                "[PickPlace] Pose ngoài vùng an toàn: "
                f"x={x:.3f}, y={y:.3f}, z={z_detect:.3f}"
            )
            return

        if USE_DETECTED_Z:
            pick_z = float(z_detect) + float(Z_PICK_CORRECTION)
        else:
            pick_z = float(FIXED_PICK_Z)

        if USE_YAW_FOR_GRASP:
            qx, qy, qz, qw = grasp_quat_from_robot_yaw_deg(yaw_robot)
        else:
            qx, qy, qz, qw = PICK_QX, PICK_QY, PICK_QZ, PICK_QW

        goal = PickPlace.Goal()

        goal.pose_pick.position.x = float(x)
        goal.pose_pick.position.y = float(y)
        goal.pose_pick.position.z = float(pick_z)

        goal.pose_pick.orientation.x = float(qx)
        goal.pose_pick.orientation.y = float(qy)
        goal.pose_pick.orientation.z = float(qz)
        goal.pose_pick.orientation.w = float(qw)

        goal.pose_place.position.x = float(PLACE_X)
        goal.pose_place.position.y = float(PLACE_Y)
        goal.pose_place.position.z = float(PLACE_Z)

        goal.pose_place.orientation.x = float(PICK_QX)
        goal.pose_place.orientation.y = float(PICK_QY)
        goal.pose_place.orientation.z = float(PICK_QZ)
        goal.pose_place.orientation.w = float(PICK_QW)

        goal.gripper = float(GRIPPER_CLOSE)
        goal.velocity_scale = float(VELOCITY_SCALE)

        mx, my, mz = det.get("marker_robot_xyz", det["robot_xyz"])
        u_depth, v_depth = det.get("depth_px", (None, None))

        self.append_log(
            "[PickPlace] Send goal | "
            f"id={marker_id} | "
            f"pick=({goal.pose_pick.position.x:.3f}, "
            f"{goal.pose_pick.position.y:.3f}, "
            f"{goal.pose_pick.position.z:.3f}) | "
            f"marker=({mx:.3f},{my:.3f},{mz:.3f}) | "
            f"depth_px=({u_depth},{v_depth}) | "
            f"yaw={yaw_robot:.2f} | "
            f"q=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})"
        )

        future = self.node.pickplace_client.send_goal_async(
            goal,
            feedback_callback=self.pickplace_feedback_callback,
        )

        future.add_done_callback(self.pickplace_goal_response_callback)

    def pickplace_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.append_log(f"[PickPlace] Send goal exception: {e}")
            return

        if not goal_handle.accepted:
            self.append_log("[PickPlace] Goal rejected")
            return

        self.append_log("[PickPlace] Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.pickplace_result_callback)

    def pickplace_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.append_log(
            f"[PickPlace feedback] {feedback.stage} | {feedback.progress:.1f}%"
        )

    def pickplace_result_callback(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
        except Exception as e:
            self.append_log(f"[PickPlace] Result exception: {e}")
            return

        self.append_log(
            f"[PickPlace result] status={status} | "
            f"success={result.success} | message={result.message}"
        )

    def closeEvent(self, event: QtGui.QCloseEvent):
        try:
            self.spin_timer.stop()
            self.gui_timer.stop()
            self.node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        event.accept()


def main(args=None):
    rclpy.init(args=args)

    node = ArucoPickGuiNode()

    app = QtWidgets.QApplication(sys.argv)

    w = MainWindow(node)
    w.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()