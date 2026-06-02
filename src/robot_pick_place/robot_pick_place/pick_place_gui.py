#!/usr/bin/env python3

import math
import queue
import random
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
from PyQt6.QtCore import QThread, Qt, pyqtSignal
from PyQt6.QtGui import QFont, QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image as RosImage

from robot_task_manager.action import PickPlace  # type: ignore
from robot_vision_pipeline_msgs.msg import ArucoPoseArray  # type: ignore


@dataclass
class GuiConfig:
    aruco_image_topic: str = "/aruco/image_annotated"
    aruco_pose_topic: str = "/aruco_pose"
    pick_place_action_name: str = "/pickplace"
    place_x: float = 0.300
    place_y: float = -0.2
    place_z: float = 0.10
    place_qx: float = 0.7071
    place_qy: float = 0.7071
    place_qz: float = 0.0000
    place_qw: float = 0.0000
    # Khi gửi PickPlace, thường KHÔNG nên dùng nguyên quaternion của marker ArUco
    # làm orientation TCP, vì quaternion đó là hướng của mặt marker chứ không hẳn là
    # hướng hợp lệ của gripper. Mặc định dùng orientation cố định giống CLI test.
    use_fixed_pick_orientation: bool = True
    pick_qx: float = 0.7071
    pick_qy: float = 0.7071
    pick_qz: float = 0.0000
    pick_qw: float = 0.0000
    pick_z_offset: float = 0.000
    min_pick_z: float = -10.0
    gripper: float = 0.022
    velocity_scale: float = 0.9


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _normalize_quaternion(qx: float, qy: float, qz: float, qw: float):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return 0.7071, 0.7071, 0.0, 0.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def pose_msg_to_dict(pose: Pose) -> Dict[str, Dict[str, float]]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def dict_to_pose(data: Dict[str, Dict[str, float]], fallback_q=None) -> Pose:
    pose = Pose()
    pose.position.x = float(data["position"]["x"])
    pose.position.y = float(data["position"]["y"])
    pose.position.z = float(data["position"]["z"])

    q = data.get("orientation", {})
    qx = float(q.get("x", 0.0))
    qy = float(q.get("y", 0.0))
    qz = float(q.get("z", 0.0))
    qw = float(q.get("w", 0.0))

    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if q_norm < 1e-6 and fallback_q is not None:
        qx, qy, qz, qw = fallback_q
    else:
        qx, qy, qz, qw = _normalize_quaternion(qx, qy, qz, qw)

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class ImageDisplayWidget(QWidget):
    def __init__(self, title: str = "ArUco Detection"):
        super().__init__()
        self.title = title
        self.image: Optional[np.ndarray] = None
        self.scale_factor = 100.0
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()

        title_label = QLabel(self.title)
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(12)
        title_label.setFont(title_font)
        layout.addWidget(title_label)

        self.image_label = QLabel("Waiting for /aruco/image_annotated ...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(800, 520)
        self.image_label.setStyleSheet("border: 1px solid black; background-color: #444; color: white;")
        layout.addWidget(self.image_label)

        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Zoom:"))
        self.zoom_spin = QSpinBox()
        self.zoom_spin.setMinimum(10)
        self.zoom_spin.setMaximum(300)
        self.zoom_spin.setValue(100)
        self.zoom_spin.setSuffix("%")
        self.zoom_spin.valueChanged.connect(self._on_zoom_changed)
        control_layout.addWidget(self.zoom_spin)

        fit_button = QPushButton("Fit")
        fit_button.clicked.connect(self.fit_to_window)
        control_layout.addWidget(fit_button)
        control_layout.addStretch()

        self.info_label = QLabel("No image")
        self.info_label.setStyleSheet("color: gray;")
        control_layout.addWidget(self.info_label)
        layout.addLayout(control_layout)
        self.setLayout(layout)

    def set_image(self, cv_image: np.ndarray):
        if cv_image is None:
            self.image_label.setText("No image")
            return
        self.image = cv_image
        self._update_display()
        h, w = cv_image.shape[:2]
        channels = cv_image.shape[2] if len(cv_image.shape) > 2 else 1
        self.info_label.setText(f"Size: {w}x{h} | Channels: {channels}")

    def _update_display(self):
        if self.image is None:
            return

        if len(self.image.shape) == 3 and self.image.shape[2] == 3:
            rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        else:
            rgb_image = self.image

        h, w = rgb_image.shape[:2]
        new_w = max(1, int(w * self.scale_factor / 100.0))
        new_h = max(1, int(h * self.scale_factor / 100.0))
        resized = cv2.resize(rgb_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        resized = np.ascontiguousarray(resized)

        if len(resized.shape) == 3:
            h, w, ch = resized.shape
            qt_image = QImage(
                resized.data,
                w,
                h,
                ch * w,
                QImage.Format.Format_RGB888,
            ).copy()
        else:
            h, w = resized.shape
            qt_image = QImage(
                resized.data,
                w,
                h,
                w,
                QImage.Format.Format_Grayscale8,
            ).copy()

        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def _on_zoom_changed(self, value: int):
        self.scale_factor = float(value)
        self._update_display()

    def fit_to_window(self):
        if self.image is None:
            return
        label_w = max(1, self.image_label.width())
        label_h = max(1, self.image_label.height())
        img_h, img_w = self.image.shape[:2]
        if img_w <= 0 or img_h <= 0:
            return
        scale = min((label_w / img_w) * 100.0, (label_h / img_h) * 100.0)
        self.zoom_spin.setValue(max(10, min(300, int(scale))))


class RosWorker(QThread):
    aruco_image_received = pyqtSignal(object)
    aruco_pose_received = pyqtSignal(object)
    config_received = pyqtSignal(object)
    action_busy_changed = pyqtSignal(bool)
    log_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, ros_args=None):
        super().__init__()
        self.ros_args = ros_args
        self.node: Optional[Node] = None
        self.bridge = CvBridge()
        self.running = True
        self.config = GuiConfig()
        self._latest_poses: List[Dict[str, Any]] = []
        self._cmd_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self._action_client: Optional[ActionClient] = None
        self._busy = False

    def run(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=self.ros_args)

            self.node = Node("robot_pick_place_gui")
            self._load_parameters()
            self.config_received.emit(self.config.__dict__.copy())

            self.node.create_subscription(
                RosImage,
                self.config.aruco_image_topic,
                self._aruco_image_callback,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                ArucoPoseArray,
                self.config.aruco_pose_topic,
                self._aruco_pose_callback,
                qos_profile_sensor_data,
            )
            self._action_client = ActionClient(
                self.node,
                PickPlace,
                self.config.pick_place_action_name,
            )

            self._log(
                f"Started. image_topic={self.config.aruco_image_topic}, "
                f"pose_topic={self.config.aruco_pose_topic}, "
                f"action={self.config.pick_place_action_name}"
            )

            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                self._process_one_command_if_any()

        except Exception as exc:
            self.error_occurred.emit(f"ROS worker error: {exc}")
        finally:
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _load_parameters(self):
        assert self.node is not None
        n = self.node
        n.declare_parameter("aruco_image_topic", self.config.aruco_image_topic)
        n.declare_parameter("aruco_pose_topic", self.config.aruco_pose_topic)
        n.declare_parameter("pick_place_action_name", self.config.pick_place_action_name)
        n.declare_parameter("place_x", self.config.place_x)
        n.declare_parameter("place_y", self.config.place_y)
        n.declare_parameter("place_z", self.config.place_z)
        n.declare_parameter("place_qx", self.config.place_qx)
        n.declare_parameter("place_qy", self.config.place_qy)
        n.declare_parameter("place_qz", self.config.place_qz)
        n.declare_parameter("place_qw", self.config.place_qw)
        n.declare_parameter("use_fixed_pick_orientation", self.config.use_fixed_pick_orientation)
        n.declare_parameter("pick_qx", self.config.pick_qx)
        n.declare_parameter("pick_qy", self.config.pick_qy)
        n.declare_parameter("pick_qz", self.config.pick_qz)
        n.declare_parameter("pick_qw", self.config.pick_qw)
        n.declare_parameter("pick_z_offset", self.config.pick_z_offset)
        n.declare_parameter("min_pick_z", self.config.min_pick_z)
        n.declare_parameter("gripper", self.config.gripper)
        n.declare_parameter("velocity_scale", self.config.velocity_scale)

        self.config.aruco_image_topic = str(n.get_parameter("aruco_image_topic").value)
        self.config.aruco_pose_topic = str(n.get_parameter("aruco_pose_topic").value)
        self.config.pick_place_action_name = str(n.get_parameter("pick_place_action_name").value)
        self.config.place_x = _to_float(n.get_parameter("place_x").value, self.config.place_x)
        self.config.place_y = _to_float(n.get_parameter("place_y").value, self.config.place_y)
        self.config.place_z = _to_float(n.get_parameter("place_z").value, self.config.place_z)
        self.config.place_qx = _to_float(n.get_parameter("place_qx").value, self.config.place_qx)
        self.config.place_qy = _to_float(n.get_parameter("place_qy").value, self.config.place_qy)
        self.config.place_qz = _to_float(n.get_parameter("place_qz").value, self.config.place_qz)
        self.config.place_qw = _to_float(n.get_parameter("place_qw").value, self.config.place_qw)
        self.config.use_fixed_pick_orientation = bool(n.get_parameter("use_fixed_pick_orientation").value)
        self.config.pick_qx = _to_float(n.get_parameter("pick_qx").value, self.config.pick_qx)
        self.config.pick_qy = _to_float(n.get_parameter("pick_qy").value, self.config.pick_qy)
        self.config.pick_qz = _to_float(n.get_parameter("pick_qz").value, self.config.pick_qz)
        self.config.pick_qw = _to_float(n.get_parameter("pick_qw").value, self.config.pick_qw)
        self.config.pick_z_offset = _to_float(n.get_parameter("pick_z_offset").value, self.config.pick_z_offset)
        self.config.min_pick_z = _to_float(n.get_parameter("min_pick_z").value, self.config.min_pick_z)
        self.config.gripper = _to_float(n.get_parameter("gripper").value, self.config.gripper)
        self.config.velocity_scale = _to_float(n.get_parameter("velocity_scale").value, self.config.velocity_scale)

        qx, qy, qz, qw = _normalize_quaternion(
            self.config.place_qx,
            self.config.place_qy,
            self.config.place_qz,
            self.config.place_qw,
        )
        self.config.place_qx = qx
        self.config.place_qy = qy
        self.config.place_qz = qz
        self.config.place_qw = qw

        qx, qy, qz, qw = _normalize_quaternion(
            self.config.pick_qx,
            self.config.pick_qy,
            self.config.pick_qz,
            self.config.pick_qw,
        )
        self.config.pick_qx = qx
        self.config.pick_qy = qy
        self.config.pick_qz = qz
        self.config.pick_qw = qw

    def request_pick_all(self):
        self._cmd_queue.put({"type": "pick_all"})

    def request_pick_id(self, marker_id: int):
        self._cmd_queue.put({"type": "pick_id", "id": int(marker_id)})

    def stop(self):
        self.running = False

    def _aruco_image_callback(self, msg: RosImage):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.aruco_image_received.emit(cv_image)
        except Exception as exc:
            self.error_occurred.emit(f"ArUco image conversion error: {exc}")

    def _aruco_pose_callback(self, msg: ArucoPoseArray):
        try:
            detections: List[Dict[str, Any]] = []
            for item in msg.poses:
                detections.append({
                    "id": int(item.id),
                    "frame_cam": str(item.frame_cam),
                    "frame_base": str(item.frame_base),
                    "has_pose_base": bool(item.has_pose_base),
                    "yaw_deg": float(item.yaw_deg),
                    "pose_base": pose_msg_to_dict(item.pose_base),
                })
            self._latest_poses = detections
            self.aruco_pose_received.emit(detections)
        except Exception as exc:
            self.error_occurred.emit(f"ArUco pose conversion error: {exc}")

    def _process_one_command_if_any(self):
        if self._busy:
            return
        try:
            cmd = self._cmd_queue.get_nowait()
        except queue.Empty:
            return

        self._busy = True
        self.action_busy_changed.emit(True)
        try:
            if cmd["type"] == "pick_all":
                self._handle_pick_all()
            elif cmd["type"] == "pick_id":
                self._handle_pick_id(int(cmd["id"]))
        finally:
            self._busy = False
            self.action_busy_changed.emit(False)

    def _valid_snapshot(self) -> List[Dict[str, Any]]:
        return [p for p in list(self._latest_poses) if p.get("has_pose_base", False)]

    def _handle_pick_all(self):
        poses = self._valid_snapshot()
        if not poses:
            self._log("Pick All skipped: chưa có ArUco pose hợp lệ trong frame base.")
            return

        self._log(f"Pick All: sẽ gửi {len(poses)} goal theo snapshot hiện tại.")
        for index, pose_info in enumerate(poses, start=1):
            if not self.running:
                break
            self._log(
                f"[{index}/{len(poses)}] Pick marker ID={pose_info['id']} "
                f"at {self._format_xyz(pose_info)}"
            )
            ok = self._send_pick_place_goal(pose_info)
            if not ok:
                self._log(f"Dừng Pick All vì goal ID={pose_info['id']} thất bại.")
                break

    def _handle_pick_id(self, marker_id: int):
        matches = [p for p in self._valid_snapshot() if int(p.get("id", -1)) == marker_id]
        if not matches:
            self._log(f"Pick ID skipped: không thấy ID={marker_id} có pose_base hợp lệ.")
            return

        selected = random.choice(matches)
        self._log(
            f"Pick ID={marker_id}: tìm thấy {len(matches)} pose, chọn ngẫu nhiên 1 pose "
            f"at {self._format_xyz(selected)}"
        )
        self._send_pick_place_goal(selected)

    def _build_place_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = self.config.place_x
        pose.position.y = self.config.place_y
        pose.position.z = self.config.place_z
        pose.orientation.x = self.config.place_qx
        pose.orientation.y = self.config.place_qy
        pose.orientation.z = self.config.place_qz
        pose.orientation.w = self.config.place_qw
        return pose

    def _build_pick_pose(self, pose_info: Dict[str, Any]) -> Pose:
        pose = Pose()
        src = pose_info["pose_base"]
        pos = src["position"]
        pose.position.x = float(pos["x"])
        pose.position.y = float(pos["y"])
        pose.position.z = max(float(pos["z"]) + self.config.pick_z_offset, self.config.min_pick_z)

        if self.config.use_fixed_pick_orientation:
            pose.orientation.x = self.config.pick_qx
            pose.orientation.y = self.config.pick_qy
            pose.orientation.z = self.config.pick_qz
            pose.orientation.w = self.config.pick_qw
        else:
            fallback_q = (
                self.config.pick_qx,
                self.config.pick_qy,
                self.config.pick_qz,
                self.config.pick_qw,
            )
            raw_pose = dict_to_pose(src, fallback_q=fallback_q)
            pose.orientation = raw_pose.orientation
        return pose

    def _send_pick_place_goal(self, pose_info: Dict[str, Any]) -> bool:
        if self.node is None or self._action_client is None:
            self._log("Action client chưa sẵn sàng.")
            return False

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self._log(
                f"Không thấy action server {self.config.pick_place_action_name}. "
                "Hãy chạy pickplace_server của robot_task_manager trước."
            )
            return False

        goal = PickPlace.Goal()
        goal.pose_pick = self._build_pick_pose(pose_info)
        goal.pose_place = self._build_place_pose()
        goal.gripper = float(self.config.gripper)
        goal.velocity_scale = float(self.config.velocity_scale)

        self._log(
            "Send PickPlace goal | "
            f"ID={pose_info['id']} | "
            f"pick=({goal.pose_pick.position.x:.3f}, {goal.pose_pick.position.y:.3f}, {goal.pose_pick.position.z:.3f}) | "
            f"place=({goal.pose_place.position.x:.3f}, {goal.pose_place.position.y:.3f}, {goal.pose_place.position.z:.3f}) | "
            f"gripper={goal.gripper:.3f}, v={goal.velocity_scale:.2f} | "
            f"pick_q=({goal.pose_pick.orientation.x:.4f}, {goal.pose_pick.orientation.y:.4f}, "
            f"{goal.pose_pick.orientation.z:.4f}, {goal.pose_pick.orientation.w:.4f})"
        )

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._pick_feedback_callback,
        )
        if not self._wait_for_future(send_future, timeout_sec=5.0):
            self._log("Timeout khi gửi PickPlace goal.")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._log("PickPlace goal bị action server từ chối.")
            return False

        self._log("PickPlace goal accepted.")
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=None):
            self._log("PickPlace goal chưa trả result do GUI đang dừng.")
            return False

        result = result_future.result().result
        if bool(result.success):
            self._log(f"PickPlace success: {result.message}")
            return True

        self._log(f"PickPlace failed: {result.message}")
        return False

    def _pick_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._log(f"Feedback: {feedback.stage} | {float(feedback.progress):.1f}%")

    def _wait_for_future(self, future, timeout_sec: Optional[float]) -> bool:
        assert self.node is not None
        start = time.monotonic()
        while self.running and rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if timeout_sec is not None and (time.monotonic() - start) > timeout_sec:
                return False
        return future.done()

    def _format_xyz(self, pose_info: Dict[str, Any]) -> str:
        pos = pose_info["pose_base"]["position"]
        return f"x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}"

    def _log(self, text: str):
        if self.node is not None:
            self.node.get_logger().info(text)
        self.log_received.emit(text)


class PickPlaceGUI(QMainWindow):
    def __init__(self, ros_args=None):
        super().__init__()
        self.setWindowTitle("Robot Pick Place - ArUco GUI")
        self.setGeometry(100, 100, 1400, 850)
        self.worker = RosWorker(ros_args=ros_args)
        self._latest_pose_count = 0
        self._setup_ui()
        self._connect_worker()
        self.worker.start()

    def _setup_ui(self):
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        tab = QWidget()
        root = QHBoxLayout()

        self.detected_display = ImageDisplayWidget("ArUco Detection")
        root.addWidget(self.detected_display, stretch=3)

        side = QVBoxLayout()

        topic_group = QGroupBox("ROS topics / action")
        topic_layout = QVBoxLayout()
        self.image_topic_label = QLabel("Image: /aruco/image_annotated")
        self.pose_topic_label = QLabel("Pose: /aruco_pose")
        self.action_label = QLabel("Action: /pickplace")
        topic_layout.addWidget(self.image_topic_label)
        topic_layout.addWidget(self.pose_topic_label)
        topic_layout.addWidget(self.action_label)
        topic_group.setLayout(topic_layout)
        side.addWidget(topic_group)

        place_group = QGroupBox("Fixed place pose")
        place_layout = QVBoxLayout()
        self.place_label = QLabel("place=(0.300, 0.000, 0.250)")
        self.place_quat_label = QLabel("place quat=(0.7071, 0.7071, 0.0000, 0.0000)")
        self.pick_quat_label = QLabel("pick fixed quat=(0.7071, 0.7071, 0.0000, 0.0000)")
        self.pick_option_label = QLabel("pick orientation: fixed")
        place_layout.addWidget(self.place_label)
        place_layout.addWidget(self.place_quat_label)
        place_layout.addWidget(self.pick_quat_label)
        place_layout.addWidget(self.pick_option_label)
        place_group.setLayout(place_layout)
        side.addWidget(place_group)

        control_group = QGroupBox("Pick commands")
        control_layout = QVBoxLayout()

        self.pose_count_label = QLabel("Detected markers: 0")
        control_layout.addWidget(self.pose_count_label)

        self.pick_all_button = QPushButton("Pick All")
        self.pick_all_button.clicked.connect(self._on_pick_all_clicked)
        control_layout.addWidget(self.pick_all_button)

        id_row = QHBoxLayout()
        id_row.addWidget(QLabel("ID:"))
        self.id_spin = QSpinBox()
        self.id_spin.setMinimum(-2147483648)
        self.id_spin.setMaximum(2147483647)
        self.id_spin.setValue(0)
        id_row.addWidget(self.id_spin)
        self.pick_id_button = QPushButton("Pick ID")
        self.pick_id_button.clicked.connect(self._on_pick_id_clicked)
        id_row.addWidget(self.pick_id_button)
        control_layout.addLayout(id_row)

        param_row = QHBoxLayout()
        param_row.addWidget(QLabel("Gripper:"))
        self.gripper_spin = QDoubleSpinBox()
        self.gripper_spin.setDecimals(3)
        self.gripper_spin.setRange(0.0, 0.100)
        self.gripper_spin.setSingleStep(0.001)
        self.gripper_spin.setValue(0.010)
        self.gripper_spin.setEnabled(False)
        param_row.addWidget(self.gripper_spin)
        param_row.addWidget(QLabel("Vel:"))
        self.velocity_spin = QDoubleSpinBox()
        self.velocity_spin.setDecimals(2)
        self.velocity_spin.setRange(0.01, 1.00)
        self.velocity_spin.setSingleStep(0.05)
        self.velocity_spin.setValue(0.30)
        self.velocity_spin.setEnabled(False)
        param_row.addWidget(self.velocity_spin)
        control_layout.addLayout(param_row)

        control_group.setLayout(control_layout)
        side.addWidget(control_group)

        pose_group = QGroupBox("Current ArUco poses")
        pose_layout = QVBoxLayout()
        self.pose_display = QPlainTextEdit()
        self.pose_display.setReadOnly(True)
        self.pose_display.setPlaceholderText("Waiting for /aruco_pose ...")
        pose_layout.addWidget(self.pose_display)
        pose_group.setLayout(pose_layout)
        side.addWidget(pose_group, stretch=2)

        log_group = QGroupBox("PickPlace log")
        log_layout = QVBoxLayout()
        self.log_display = QPlainTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumBlockCount(300)
        log_layout.addWidget(self.log_display)
        log_group.setLayout(log_layout)
        side.addWidget(log_group, stretch=2)

        root.addLayout(side, stretch=2)
        tab.setLayout(root)
        self.tabs.addTab(tab, "ArUco Pick Place")
        self.statusBar().showMessage("Starting ROS worker...")

    def _connect_worker(self):
        self.worker.aruco_image_received.connect(self._on_aruco_image_received)
        self.worker.aruco_pose_received.connect(self._on_aruco_pose_received)
        self.worker.config_received.connect(self._on_config_received)
        self.worker.action_busy_changed.connect(self._on_action_busy_changed)
        self.worker.log_received.connect(self._append_log)
        self.worker.error_occurred.connect(self._on_ros_error)

    def _on_config_received(self, cfg: Dict[str, Any]):
        self.image_topic_label.setText(f"Image: {cfg['aruco_image_topic']}")
        self.pose_topic_label.setText(f"Pose: {cfg['aruco_pose_topic']}")
        self.action_label.setText(f"Action: {cfg['pick_place_action_name']}")
        self.place_label.setText(
            f"place=({float(cfg['place_x']):.3f}, {float(cfg['place_y']):.3f}, {float(cfg['place_z']):.3f})"
        )
        self.place_quat_label.setText(
            "place quat="
            f"({float(cfg['place_qx']):.4f}, {float(cfg['place_qy']):.4f}, "
            f"{float(cfg['place_qz']):.4f}, {float(cfg['place_qw']):.4f})"
        )
        self.pick_quat_label.setText(
            "pick fixed quat="
            f"({float(cfg['pick_qx']):.4f}, {float(cfg['pick_qy']):.4f}, "
            f"{float(cfg['pick_qz']):.4f}, {float(cfg['pick_qw']):.4f})"
        )
        mode = "fixed" if bool(cfg["use_fixed_pick_orientation"]) else "aruco pose_base quaternion"
        self.pick_option_label.setText(
            f"pick orientation: {mode} | z_offset={float(cfg['pick_z_offset']):.3f} | min_z={float(cfg['min_pick_z']):.3f}"
        )
        self.gripper_spin.setValue(float(cfg["gripper"]))
        self.velocity_spin.setValue(float(cfg["velocity_scale"]))

    def _on_aruco_image_received(self, cv_image: np.ndarray):
        self.detected_display.set_image(cv_image)
        self.statusBar().showMessage(f"ArUco image received: {cv_image.shape}")

    def _on_aruco_pose_received(self, pose_list: List[Dict[str, Any]]):
        self._latest_pose_count = len(pose_list)
        valid_count = sum(1 for p in pose_list if p.get("has_pose_base", False))
        self.pose_count_label.setText(f"Detected markers: {len(pose_list)} | valid base pose: {valid_count}")

        if not pose_list:
            self.pose_display.setPlainText("No ArUco poses received yet.")
            return

        lines = []
        for idx, pose in enumerate(pose_list, start=1):
            pos = pose["pose_base"]["position"]
            ori = pose["pose_base"]["orientation"]
            status = "OK" if pose["has_pose_base"] else "NO_BASE_POSE"
            lines.append(
                f"#{idx} | ID={pose['id']} | {status} | frame_base={pose['frame_base']}"
            )
            lines.append(
                f"  xyz=({pos['x']:.4f}, {pos['y']:.4f}, {pos['z']:.4f}) m | yaw={pose['yaw_deg']:.2f} deg"
            )
            lines.append(
                f"  q=({ori['x']:.4f}, {ori['y']:.4f}, {ori['z']:.4f}, {ori['w']:.4f})"
            )
        self.pose_display.setPlainText("\n".join(lines))

    def _on_pick_all_clicked(self):
        self.worker.request_pick_all()
        self._append_log("GUI: Pick All requested.")

    def _on_pick_id_clicked(self):
        marker_id = int(self.id_spin.value())
        self.worker.request_pick_id(marker_id)
        self._append_log(f"GUI: Pick ID={marker_id} requested.")

    def _on_action_busy_changed(self, busy: bool):
        self.pick_all_button.setEnabled(not busy)
        self.pick_id_button.setEnabled(not busy)
        self.id_spin.setEnabled(not busy)
        if busy:
            self.statusBar().showMessage("PickPlace action is running...")
        else:
            self.statusBar().showMessage("Ready")

    def _append_log(self, text: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_display.appendPlainText(f"[{timestamp}] {text}")

    def _on_ros_error(self, error_msg: str):
        self._append_log(f"ERROR: {error_msg}")
        self.statusBar().showMessage(f"Error: {error_msg}")

    def closeEvent(self, event):
        self.worker.stop()
        self.worker.wait(2000)
        event.accept()


def main():
    ros_args = sys.argv[:]
    qt_args = remove_ros_args(args=sys.argv)
    app = QApplication(qt_args)
    gui = PickPlaceGUI(ros_args=ros_args)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
