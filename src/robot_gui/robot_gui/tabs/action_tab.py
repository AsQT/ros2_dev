import inspect
import math
import threading
from typing import Tuple

from qtpy import QtCore, QtWidgets
from robot_gui.backends.pee_backend import PeeBackend


BASE_FRAME_ID = "base_link"


class ActionTab(QtWidgets.QWidget):
    worker_finished = QtCore.Signal(bool, str)

    ACTION_LIST = [
        ("checker_board", "CheckerBoard", True),
        ("move_to_pose", "Move To Pose", True),
        ("move_to_pose_cartesian", "Move To Pose Cartesian", True),
    ]

    def __init__(self, backend: PeeBackend, parent=None):
        super().__init__(parent)

        self.backend = backend
        self._busy = False

        layout = QtWidgets.QVBoxLayout(self)

        # =====================================================
        # ACTION SELECT
        # Frame ID không hiển thị trên GUI nữa.
        # Khi gửi action luôn dùng BASE_FRAME_ID = "base_link".
        # =====================================================
        action_box = QtWidgets.QGroupBox("Gọi Action")
        action_form = QtWidgets.QFormLayout(action_box)

        self.action_combo = QtWidgets.QComboBox()
        for key, name, need_pose in self.ACTION_LIST:
            self.action_combo.addItem(name, {
                "key": key,
                "need_pose": need_pose,
            })

        self.vel_scale = QtWidgets.QDoubleSpinBox()
        self.vel_scale.setRange(0.01, 1.00)
        self.vel_scale.setDecimals(2)
        self.vel_scale.setSingleStep(0.05)
        self.vel_scale.setValue(0.30)

        action_form.addRow("Action:", self.action_combo)
        action_form.addRow("Vel scale:", self.vel_scale)

        layout.addWidget(action_box)

        # =====================================================
        # POSE INPUT
        # XYZ dùng mét.
        # RPY nhập độ, khi gửi action sẽ đổi sang quaternion.
        # =====================================================
        pose_box = QtWidgets.QGroupBox("Nhập pose đích")
        pose_grid = QtWidgets.QGridLayout(pose_box)

        self.in_x = self._make_spin(-10.0, 10.0, 0.001, 4, 0.30)
        self.in_y = self._make_spin(-10.0, 10.0, 0.001, 4, 0.00)
        self.in_z = self._make_spin(-10.0, 10.0, 0.001, 4, 0.30)

        self.in_r = self._make_spin(-360.0, 360.0, 1.0, 2, 180.0)
        self.in_p = self._make_spin(-360.0, 360.0, 1.0, 2, 0.0)
        self.in_yaw = self._make_spin(-360.0, 360.0, 1.0, 2, 0.0)

        pose_grid.addWidget(QtWidgets.QLabel("X [m]"), 0, 0)
        pose_grid.addWidget(self.in_x, 0, 1)

        pose_grid.addWidget(QtWidgets.QLabel("Y [m]"), 1, 0)
        pose_grid.addWidget(self.in_y, 1, 1)

        pose_grid.addWidget(QtWidgets.QLabel("Z [m]"), 2, 0)
        pose_grid.addWidget(self.in_z, 2, 1)

        pose_grid.addWidget(QtWidgets.QLabel("Roll [deg]"), 0, 2)
        pose_grid.addWidget(self.in_r, 0, 3)

        pose_grid.addWidget(QtWidgets.QLabel("Pitch [deg]"), 1, 2)
        pose_grid.addWidget(self.in_p, 1, 3)

        pose_grid.addWidget(QtWidgets.QLabel("Yaw [deg]"), 2, 2)
        pose_grid.addWidget(self.in_yaw, 2, 3)

        layout.addWidget(pose_box)

        # =====================================================
        # QUATERNION PREVIEW
        # Hiển thị nhỏ gọn trên 1 hàng.
        # =====================================================
        quat_box = QtWidgets.QGroupBox("Quaternion tự động")
        quat_grid = QtWidgets.QGridLayout(quat_box)
        quat_grid.setHorizontalSpacing(8)
        quat_grid.setVerticalSpacing(4)

        self.out_qx = self._make_readonly_line()
        self.out_qy = self._make_readonly_line()
        self.out_qz = self._make_readonly_line()
        self.out_qw = self._make_readonly_line()

        quat_grid.addWidget(QtWidgets.QLabel("qx"), 0, 0)
        quat_grid.addWidget(self.out_qx, 0, 1)

        quat_grid.addWidget(QtWidgets.QLabel("qy"), 0, 2)
        quat_grid.addWidget(self.out_qy, 0, 3)

        quat_grid.addWidget(QtWidgets.QLabel("qz"), 0, 4)
        quat_grid.addWidget(self.out_qz, 0, 5)

        quat_grid.addWidget(QtWidgets.QLabel("qw"), 0, 6)
        quat_grid.addWidget(self.out_qw, 0, 7)

        layout.addWidget(quat_box)

        # =====================================================
        # BUTTON
        # =====================================================
        btn_row = QtWidgets.QHBoxLayout()

        self.btn_send = QtWidgets.QPushButton("Call Action")
        btn_row.addWidget(self.btn_send)

        layout.addLayout(btn_row)

        # =====================================================
        # LOG
        # =====================================================
        self.log_box = QtWidgets.QTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box)

        # =====================================================
        # SIGNALS
        # =====================================================
        self.worker_finished.connect(self._finish_worker)

        self.btn_send.clicked.connect(self.on_send_clicked)
        self.action_combo.currentIndexChanged.connect(self._on_action_changed)

        for spin in [
            self.in_x,
            self.in_y,
            self.in_z,
            self.in_r,
            self.in_p,
            self.in_yaw,
        ]:
            spin.valueChanged.connect(self.update_quaternion_preview)

        self.update_quaternion_preview()
        self._on_action_changed()

    def _make_spin(
        self,
        min_v: float,
        max_v: float,
        step: float,
        decimals: int,
        value: float,
    ) -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(min_v, max_v)
        box.setDecimals(decimals)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def _make_readonly_line(self) -> QtWidgets.QLineEdit:
        line = QtWidgets.QLineEdit()
        line.setReadOnly(True)
        line.setMaximumHeight(26)
        return line

    def append_log(self, text: str) -> None:
        self.log_box.append(text)

    def set_busy(self, busy: bool) -> None:
        self._busy = busy
        self.btn_send.setEnabled(not busy)
        self.action_combo.setEnabled(not busy)
        self.vel_scale.setEnabled(not busy)

    def _on_action_changed(self) -> None:
        data = self.action_combo.currentData()
        need_pose = bool(data.get("need_pose", True))

        for widget in [
            self.in_x,
            self.in_y,
            self.in_z,
            self.in_r,
            self.in_p,
            self.in_yaw,
        ]:
            widget.setEnabled(need_pose)

        self.vel_scale.setEnabled(need_pose and not self._busy)
        self.update_quaternion_preview()

    def update_quaternion_preview(self) -> None:
        qx, qy, qz, qw = self.rpy_deg_to_quaternion(
            self.in_r.value(),
            self.in_p.value(),
            self.in_yaw.value(),
        )

        self.out_qx.setText(f"{qx:.6f}")
        self.out_qy.setText(f"{qy:.6f}")
        self.out_qz.setText(f"{qz:.6f}")
        self.out_qw.setText(f"{qw:.6f}")

    @staticmethod
    def rpy_deg_to_quaternion(
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> Tuple[float, float, float, float]:
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def on_send_clicked(self) -> None:
        if self._busy:
            return

        if not hasattr(self.backend, "send_gui_action"):
            self.append_log("[ERR] Backend chưa có hàm send_gui_action().")
            return

        self.set_busy(True)
        self.append_log("Đang gọi action...")

        threading.Thread(target=self._send_worker, daemon=True).start()

    def _backend_accepts_argument(self, arg_name: str) -> bool:
        try:
            sig = inspect.signature(self.backend.send_gui_action)
        except Exception:
            return False

        for param in sig.parameters.values():
            if param.kind == inspect.Parameter.VAR_KEYWORD:
                return True

        return arg_name in sig.parameters

    def _send_worker(self) -> None:
        data = self.action_combo.currentData()
        action_key = str(data.get("key", ""))
        need_pose = bool(data.get("need_pose", True))

        qx, qy, qz, qw = self.rpy_deg_to_quaternion(
            self.in_r.value(),
            self.in_p.value(),
            self.in_yaw.value(),
        )

        kwargs = {
            "action_key": action_key,
            "need_pose": need_pose,
            "frame_id": BASE_FRAME_ID,
            "x": self.in_x.value(),
            "y": self.in_y.value(),
            "z": self.in_z.value(),
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
        }

        vel = float(self.vel_scale.value())

        # Tương thích 2 kiểu tên tham số ở backend:
        #   vel_scale hoặc velocity_scale.
        # Nếu backend chưa có tham số này thì GUI vẫn chạy bình thường.
        if self._backend_accepts_argument("vel_scale"):
            kwargs["vel_scale"] = vel
        elif self._backend_accepts_argument("velocity_scale"):
            kwargs["velocity_scale"] = vel

        ok, msg = self.backend.send_gui_action(**kwargs)
        self.worker_finished.emit(ok, msg)

    def _finish_worker(self, ok: bool, msg: str) -> None:
        self.append_log(("[OK] " if ok else "[ERR] ") + msg)
        self.set_busy(False)
        self._on_action_changed()