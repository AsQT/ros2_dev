import math
import os
import re
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

from qtpy import QtCore, QtWidgets

from matplotlib.figure import Figure

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


# ============================================================
# CONFIG
# ============================================================

NUM_JOINTS = 6

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

JOINT_STATE_TOPIC = "/joint_states"
ARM_CONTROLLER_STATE_TOPIC = "/arm_controller/controller_state"

# Thời gian đồ thị mặc định.
# Trục X sẽ chạy từ từ: 0 -> 1 -> 2 -> ... -> PLOT_DURATION_SEC
PLOT_DURATION_SEC = 120.0

# Giới hạn lưu dữ liệu trong RAM.
# Nếu để GUI chạy lâu, chỉ giữ tối đa MAX_RECORD_SEC.
MAX_RECORD_SEC = 3600.0

GUI_REFRESH_MS = 100
PLOT_REFRESH_MS = 150

EXPORT_DPI = 250

# Màu của từng đường trong cùng 1 đồ thị
LINE_COLOR_ACTUAL_POS = "#1f77b4"   # xanh dương
LINE_COLOR_SET_POS = "#d62728"      # đỏ
LINE_COLOR_ACTUAL_VEL = "#2ca02c"   # xanh lá
LINE_COLOR_SET_VEL = "#ff7f0e"      # cam


# ============================================================
# DATA MODEL
# Nội bộ vẫn dùng rad/rad_s.
# GUI hiển thị deg/deg_s.
# ============================================================

@dataclass
class JointMonitorModel:
    actual_pos_rad: List[Optional[float]] = field(
        default_factory=lambda: [None] * NUM_JOINTS
    )
    actual_vel_rad_s: List[Optional[float]] = field(
        default_factory=lambda: [None] * NUM_JOINTS
    )

    set_pos_rad: List[Optional[float]] = field(
        default_factory=lambda: [None] * NUM_JOINTS
    )
    set_vel_rad_s: List[Optional[float]] = field(
        default_factory=lambda: [None] * NUM_JOINTS
    )

    last_joint_state_time: Optional[float] = None
    last_controller_state_time: Optional[float] = None

    # Mốc thời gian bắt đầu vẽ.
    # Khi bấm Clear graph thì reset về None.
    # Sample tiếp theo sẽ bắt đầu từ t = 0.
    plot_start_time: Optional[float] = None

    # Có thể chỉnh trên GUI bằng spinbox.
    plot_duration_sec: float = PLOT_DURATION_SEC

    # ts lưu thời gian đã chạy, đơn vị giây, bắt đầu từ 0.
    ts: List[deque] = field(default_factory=lambda: [deque() for _ in range(NUM_JOINTS)])

    actual_pos_deg_hist: List[deque] = field(
        default_factory=lambda: [deque() for _ in range(NUM_JOINTS)]
    )
    set_pos_deg_hist: List[deque] = field(
        default_factory=lambda: [deque() for _ in range(NUM_JOINTS)]
    )

    actual_vel_deg_s_hist: List[deque] = field(
        default_factory=lambda: [deque() for _ in range(NUM_JOINTS)]
    )
    set_vel_deg_s_hist: List[deque] = field(
        default_factory=lambda: [deque() for _ in range(NUM_JOINTS)]
    )

    lock: threading.Lock = field(default_factory=threading.Lock)

    def clear_history(self) -> None:
        self.plot_start_time = None

        for i in range(NUM_JOINTS):
            self.ts[i].clear()
            self.actual_pos_deg_hist[i].clear()
            self.set_pos_deg_hist[i].clear()
            self.actual_vel_deg_s_hist[i].clear()
            self.set_vel_deg_s_hist[i].clear()


# ============================================================
# ROS NODE
# ============================================================

class JointMonitorNode(Node):
    def __init__(self, model: JointMonitorModel):
        super().__init__("joint_monitor_gui_node")

        self.model = model

        self.sub_joint_state = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.cb_joint_state,
            10,
        )

        self.sub_arm_controller = self.create_subscription(
            JointTrajectoryControllerState,
            ARM_CONTROLLER_STATE_TOPIC,
            self.cb_controller_state,
            10,
        )

        self.get_logger().info(f"Subscribed actual: {JOINT_STATE_TOPIC}")
        self.get_logger().info(f"Subscribed setpoint: {ARM_CONTROLLER_STATE_TOPIC}")

    # ========================================================
    # /joint_states
    # actual position / actual velocity
    # ========================================================

    def cb_joint_state(self, msg: JointState):
        now = time.time()

        with self.model.lock:
            if self.model.plot_start_time is None:
                self.model.plot_start_time = now

            elapsed = now - self.model.plot_start_time

            for i, joint_name in enumerate(JOINT_NAMES):
                if joint_name not in msg.name:
                    continue

                idx = msg.name.index(joint_name)

                if idx < len(msg.position):
                    self.model.actual_pos_rad[i] = float(msg.position[idx])

                if idx < len(msg.velocity):
                    self.model.actual_vel_rad_s[i] = float(msg.velocity[idx])

                actual_pos_deg = self._rad_to_deg_or_nan(
                    self.model.actual_pos_rad[i]
                )
                actual_vel_deg_s = self._rad_to_deg_or_nan(
                    self.model.actual_vel_rad_s[i]
                )

                set_pos_deg = self._rad_to_deg_or_nan(
                    self.model.set_pos_rad[i]
                )
                set_vel_deg_s = self._rad_to_deg_or_nan(
                    self.model.set_vel_rad_s[i]
                )

                self.model.ts[i].append(elapsed)

                self.model.actual_pos_deg_hist[i].append(actual_pos_deg)
                self.model.set_pos_deg_hist[i].append(set_pos_deg)

                self.model.actual_vel_deg_s_hist[i].append(actual_vel_deg_s)
                self.model.set_vel_deg_s_hist[i].append(set_vel_deg_s)

                # Chỉ giới hạn RAM.
                # Trục X do plot_duration_sec quyết định.
                while (
                    self.model.ts[i]
                    and elapsed - self.model.ts[i][0] > MAX_RECORD_SEC
                ):
                    self.model.ts[i].popleft()
                    self.model.actual_pos_deg_hist[i].popleft()
                    self.model.set_pos_deg_hist[i].popleft()
                    self.model.actual_vel_deg_s_hist[i].popleft()
                    self.model.set_vel_deg_s_hist[i].popleft()

            self.model.last_joint_state_time = now

    # ========================================================
    # /arm_controller/controller_state
    # setpoint/reference position / velocity
    # ========================================================

    def cb_controller_state(self, msg: JointTrajectoryControllerState):
        now = time.time()

        joint_names = list(msg.joint_names)

        ref = self._get_reference_point(msg)

        if ref is None:
            return

        positions = list(ref.positions) if hasattr(ref, "positions") else []
        velocities = list(ref.velocities) if hasattr(ref, "velocities") else []

        with self.model.lock:
            for i, joint_name in enumerate(JOINT_NAMES):
                if joint_name not in joint_names:
                    continue

                idx = joint_names.index(joint_name)

                if idx < len(positions):
                    self.model.set_pos_rad[i] = float(positions[idx])

                if idx < len(velocities):
                    self.model.set_vel_rad_s[i] = float(velocities[idx])

            self.model.last_controller_state_time = now

    def _get_reference_point(self, msg: JointTrajectoryControllerState):
        """
        Tùy version ros2_control:
        - Bản mới thường có msg.reference
        - Bản cũ có thể có msg.desired
        """

        if hasattr(msg, "reference"):
            return msg.reference

        if hasattr(msg, "desired"):
            return msg.desired

        return None

    @staticmethod
    def _rad_to_deg_or_nan(value: Optional[float]) -> float:
        if value is None:
            return float("nan")
        return math.degrees(float(value))


# ============================================================
# 1 KHUNG ĐỒ THỊ CHO 1 KHỚP
# ============================================================

class JointPlotPanel(QtWidgets.QGroupBox):
    def __init__(
        self,
        joint_index: int,
        joint_name: str,
        plot_duration_sec: float,
        parent=None,
    ):
        super().__init__(parent)

        self.joint_index = joint_index
        self.joint_name = joint_name
        self.plot_duration_sec = float(plot_duration_sec)

        self.setTitle(joint_name)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # =====================================================
        # INFO LABELS
        # =====================================================

        info_grid = QtWidgets.QGridLayout()
        info_grid.setHorizontalSpacing(8)
        info_grid.setVerticalSpacing(3)

        self.lb_set_pos = QtWidgets.QLabel("Set pos: -- deg")
        self.lb_actual_pos = QtWidgets.QLabel("Actual pos: -- deg")
        self.lb_set_vel = QtWidgets.QLabel("Set vel: -- deg/s")
        self.lb_actual_vel = QtWidgets.QLabel("Actual vel: -- deg/s")

        self.lb_set_pos.setStyleSheet(
            "font-size: 9pt; font-weight: 700; color:#0B1F35;"
        )
        self.lb_actual_pos.setStyleSheet(
            "font-size: 9pt; font-weight: 700; color:#0B1F35;"
        )
        self.lb_set_vel.setStyleSheet(
            "font-size: 8pt; color:#37474F;"
        )
        self.lb_actual_vel.setStyleSheet(
            "font-size: 8pt; color:#37474F;"
        )

        info_grid.addWidget(self.lb_set_pos, 0, 0)
        info_grid.addWidget(self.lb_actual_pos, 0, 1)
        info_grid.addWidget(self.lb_set_vel, 1, 0)
        info_grid.addWidget(self.lb_actual_vel, 1, 1)

        layout.addLayout(info_grid)

        # =====================================================
        # MATPLOTLIB FIGURE
        # =====================================================

        self.fig = Figure(figsize=(6.2, 3.8), dpi=100)

        self.ax_pos = self.fig.add_subplot(111)
        self.ax_vel = self.ax_pos.twinx()

        self.ax_pos.set_title(joint_name, fontsize=9, pad=4)
        self.ax_pos.set_xlabel("Time [s]", fontsize=8, labelpad=2)
        self.ax_pos.set_ylabel("Position [deg]", fontsize=8, labelpad=2)
        self.ax_vel.set_ylabel("Velocity [deg/s]", fontsize=8, labelpad=2)

        self.ax_pos.tick_params(axis="both", labelsize=8)
        self.ax_vel.tick_params(axis="y", labelsize=8)

        self.ax_pos.grid(True)

        # =====================================================
        # STYLE ĐƯỜNG
        #
        # Actual:
        #   - nét liền như ban đầu
        #   - màu riêng
        #
        # Setpoint:
        #   - nét đứt thưa
        #   - mảnh hơn actual
        #   - màu riêng
        # =====================================================

        self.line_actual_pos, = self.ax_pos.plot(
            [],
            [],
            color=LINE_COLOR_ACTUAL_POS,
            linestyle="-",
            linewidth=2,
            label="actual pos"
        )

        self.line_set_pos, = self.ax_pos.plot(
            [],
            [],
            color=LINE_COLOR_SET_POS,
            linestyle=":",
            linewidth=1,
            label="set pos"
        )

        self.line_actual_vel, = self.ax_vel.plot(
            [],
            [],
            color=LINE_COLOR_ACTUAL_VEL,
            linestyle="-.",
            linewidth=1.8,
            alpha=0.85,
            label="actual vel"
        )

        self.line_set_vel, = self.ax_vel.plot(
            [],
            [],
            color=LINE_COLOR_SET_VEL,
            linestyle=":",
            linewidth=1,
            alpha=0.85,
            label="set vel"
        )

        lines = [
            self.line_actual_pos,
            self.line_set_pos,
            self.line_actual_vel,
            self.line_set_vel,
        ]

        labels = [line.get_label() for line in lines]
        self.ax_pos.legend(lines, labels, loc="upper right", fontsize=7)

        # Không dùng tight_layout vì trong QWidget dễ bị cắt label.
        self.fig.subplots_adjust(
            left=0.12,
            right=0.86,
            top=0.84,
            bottom=0.22,
        )

        self.canvas = FigureCanvas(self.fig)

        # Tăng chiều cao để không bị mất phần dưới.
        self.canvas.setMinimumHeight(280)
        self.setMinimumHeight(365)

        layout.addWidget(self.canvas)

        self.set_duration(self.plot_duration_sec)

    # ========================================================
    # DURATION / AXIS
    # ========================================================

    def set_duration(self, duration_sec: float) -> None:
        self.plot_duration_sec = float(duration_sec)

        # Khi mới mở hoặc mới clear:
        # không hiện cứng 0 -> 120 ngay.
        # update_plot() sẽ kéo trục X tăng dần theo dữ liệu.
        self.ax_pos.set_xlim(0.0, 1.0)
        self.ax_vel.set_xlim(0.0, 1.0)

        self.canvas.draw_idle()

    # ========================================================
    # UPDATE VALUES
    # ========================================================

    def update_values(
        self,
        set_pos_deg: Optional[float],
        actual_pos_deg: Optional[float],
        set_vel_deg_s: Optional[float],
        actual_vel_deg_s: Optional[float],
    ) -> None:
        self.lb_set_pos.setText(f"Set pos: {self._fmt(set_pos_deg, 3)} deg")
        self.lb_actual_pos.setText(f"Actual pos: {self._fmt(actual_pos_deg, 3)} deg")
        self.lb_set_vel.setText(f"Set vel: {self._fmt(set_vel_deg_s, 3)} deg/s")
        self.lb_actual_vel.setText(f"Actual vel: {self._fmt(actual_vel_deg_s, 3)} deg/s")

    # ========================================================
    # UPDATE PLOT
    # ========================================================

    def update_plot(
        self,
        x: List[float],
        actual_pos: List[float],
        set_pos: List[float],
        actual_vel: List[float],
        set_vel: List[float],
    ) -> None:
        self.line_actual_pos.set_data(x, actual_pos)
        self.line_set_pos.set_data(x, set_pos)

        self.line_actual_vel.set_data(x, actual_vel)
        self.line_set_vel.set_data(x, set_vel)

        # =====================================================
        # Trục thời gian chạy từ từ đến plot_duration_sec.
        #
        # Ví dụ:
        #   mới chạy 1s    -> X: 0 -> 1
        #   chạy 20s       -> X: 0 -> 20
        #   chạy >=120s    -> X: 0 -> 120
        # =====================================================

        if x:
            current_xmax = max(x)
            xmax = min(max(current_xmax, 1.0), self.plot_duration_sec)
        else:
            xmax = 1.0

        self.ax_pos.set_xlim(0.0, xmax)
        self.ax_vel.set_xlim(0.0, xmax)

        self._auto_ylim(self.ax_pos, [actual_pos, set_pos], -180.0, 180.0)
        self._auto_ylim(self.ax_vel, [actual_vel, set_vel], -60.0, 60.0)

        self.canvas.draw_idle()

    def clear_plot(self) -> None:
        self.line_actual_pos.set_data([], [])
        self.line_set_pos.set_data([], [])
        self.line_actual_vel.set_data([], [])
        self.line_set_vel.set_data([], [])

        self.ax_pos.set_xlim(0.0, 1.0)
        self.ax_vel.set_xlim(0.0, 1.0)

        self.canvas.draw_idle()

    def save_png(self, path: str) -> None:
        self.fig.savefig(path, dpi=EXPORT_DPI, bbox_inches="tight")

    @staticmethod
    def _fmt(value: Optional[float], ndigits: int) -> str:
        if value is None:
            return "--"
        if isinstance(value, float) and math.isnan(value):
            return "--"
        return f"{value:.{ndigits}f}"

    @staticmethod
    def _auto_ylim(
        ax,
        data_lists,
        default_min: float,
        default_max: float,
    ) -> None:
        values = []

        for data in data_lists:
            for v in data:
                if isinstance(v, float) and not math.isnan(v):
                    values.append(v)

        if not values:
            ax.set_ylim(default_min, default_max)
            return

        y_min = min(values)
        y_max = max(values)

        if abs(y_max - y_min) < 1e-9:
            y_min -= 1.0
            y_max += 1.0
        else:
            margin = 0.12 * abs(y_max - y_min)
            y_min -= margin
            y_max += margin

        ax.set_ylim(y_min, y_max)


# ============================================================
# MAIN TAB
# ============================================================

class JointMonitorTab(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.model = JointMonitorModel()

        self.ros_node: Optional[JointMonitorNode] = None
        self.ros_thread: Optional[threading.Thread] = None
        self.ros_running = False
        self.owns_rclpy = False

        self.panels: List[JointPlotPanel] = []

        self._build_ui()
        self._start_ros()

        self.timer_table = QtCore.QTimer(self)
        self.timer_table.timeout.connect(self._refresh_values)
        self.timer_table.start(GUI_REFRESH_MS)

        self.timer_plot = QtCore.QTimer(self)
        self.timer_plot.timeout.connect(self._refresh_plots)
        self.timer_plot.start(PLOT_REFRESH_MS)

    # ========================================================
    # UI
    # ========================================================

    def _build_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # ---------------- TOP BAR ----------------
        top = QtWidgets.QHBoxLayout()

        self.lb_status = QtWidgets.QLabel(
            f"Actual: {JOINT_STATE_TOPIC} | Setpoint: {ARM_CONTROLLER_STATE_TOPIC}"
        )
        self.lb_status.setStyleSheet("font-weight: 700; color:#0B1F35;")

        self.duration_spin = QtWidgets.QDoubleSpinBox()
        self.duration_spin.setRange(1.0, 3600.0)
        self.duration_spin.setDecimals(1)
        self.duration_spin.setSingleStep(10.0)
        self.duration_spin.setValue(PLOT_DURATION_SEC)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setMaximumWidth(120)

        self.btn_clear = QtWidgets.QPushButton("Clear graph")
        self.btn_capture = QtWidgets.QPushButton("Chụp / lưu 6 ảnh PNG")

        top.addWidget(self.lb_status, 1)
        top.addWidget(QtWidgets.QLabel("Thời gian đồ thị:"))
        top.addWidget(self.duration_spin)
        top.addWidget(self.btn_clear)
        top.addWidget(self.btn_capture)

        root.addLayout(top)

        # ---------------- EXPORT PATH ----------------
        export_box = QtWidgets.QGroupBox("Xuất ảnh")
        export_grid = QtWidgets.QGridLayout(export_box)

        default_dir = os.path.expanduser("~/robot_gui_plots")

        self.export_dir_edit = QtWidgets.QLineEdit(default_dir)
        self.btn_browse = QtWidgets.QPushButton("Chọn thư mục")
        self.lb_last_save = QtWidgets.QLabel("Chưa lưu ảnh.")
        self.lb_last_save.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        export_grid.addWidget(QtWidgets.QLabel("Thư mục lưu:"), 0, 0)
        export_grid.addWidget(self.export_dir_edit, 0, 1)
        export_grid.addWidget(self.btn_browse, 0, 2)
        export_grid.addWidget(QtWidgets.QLabel("Lần lưu gần nhất:"), 1, 0)
        export_grid.addWidget(self.lb_last_save, 1, 1, 1, 2)

        root.addWidget(export_box)

        # ---------------- PLOTS ----------------
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)

        content = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(content)
        grid.setContentsMargins(4, 4, 4, 4)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)

        for i, joint_name in enumerate(JOINT_NAMES):
            panel = JointPlotPanel(
                joint_index=i,
                joint_name=joint_name,
                plot_duration_sec=PLOT_DURATION_SEC,
            )
            self.panels.append(panel)

            r = i // 2
            c = i % 2
            grid.addWidget(panel, r, c)

        scroll.setWidget(content)
        root.addWidget(scroll, 1)

        self.duration_spin.valueChanged.connect(self._on_duration_changed)
        self.btn_clear.clicked.connect(self.clear_graph)
        self.btn_capture.clicked.connect(self.capture_all_plots)
        self.btn_browse.clicked.connect(self._browse_export_dir)

    def _on_duration_changed(self, value: float) -> None:
        duration = float(value)

        with self.model.lock:
            self.model.plot_duration_sec = duration

        for panel in self.panels:
            panel.plot_duration_sec = duration

    # ========================================================
    # ROS
    # ========================================================

    def _start_ros(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self.owns_rclpy = True

            self.ros_node = JointMonitorNode(self.model)
            self.ros_running = True

            self.ros_thread = threading.Thread(
                target=self._ros_spin_thread,
                daemon=True,
            )
            self.ros_thread.start()

        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self,
                "Lỗi ROS",
                str(e),
            )

    def _ros_spin_thread(self) -> None:
        while self.ros_running and rclpy.ok():
            try:
                if self.ros_node is not None:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.05)
            except Exception:
                pass

    # ========================================================
    # REFRESH VALUES
    # ========================================================

    def _refresh_values(self) -> None:
        now = time.time()

        with self.model.lock:
            actual_pos_rad = list(self.model.actual_pos_rad)
            actual_vel_rad_s = list(self.model.actual_vel_rad_s)

            set_pos_rad = list(self.model.set_pos_rad)
            set_vel_rad_s = list(self.model.set_vel_rad_s)

            last_js = self.model.last_joint_state_time
            last_ctrl = self.model.last_controller_state_time

            plot_start_time = self.model.plot_start_time
            plot_duration_sec = self.model.plot_duration_sec

        for i, panel in enumerate(self.panels):
            actual_pos_deg = self._rad_to_deg(actual_pos_rad[i])
            actual_vel_deg_s = self._rad_to_deg(actual_vel_rad_s[i])

            set_pos_deg = self._rad_to_deg(set_pos_rad[i])
            set_vel_deg_s = self._rad_to_deg(set_vel_rad_s[i])

            panel.update_values(
                set_pos_deg=set_pos_deg,
                actual_pos_deg=actual_pos_deg,
                set_vel_deg_s=set_vel_deg_s,
                actual_vel_deg_s=actual_vel_deg_s,
            )

        js_age = "--"
        ctrl_age = "--"
        elapsed_text = "--"

        if last_js is not None:
            js_age = f"{now - last_js:.2f}s"

        if last_ctrl is not None:
            ctrl_age = f"{now - last_ctrl:.2f}s"

        if plot_start_time is not None:
            elapsed = now - plot_start_time
            elapsed_show = min(elapsed, plot_duration_sec)
            elapsed_text = f"{elapsed_show:.1f}/{plot_duration_sec:.1f}s"

        self.lb_status.setText(
            f"Actual: {JOINT_STATE_TOPIC} age={js_age} | "
            f"Setpoint: {ARM_CONTROLLER_STATE_TOPIC} age={ctrl_age} | "
            f"Time: {elapsed_text}"
        )

    # ========================================================
    # REFRESH PLOTS
    # ========================================================

    def _refresh_plots(self) -> None:
        with self.model.lock:
            duration = float(self.model.plot_duration_sec)

            all_ts = [list(q) for q in self.model.ts]

            all_actual_pos = [list(q) for q in self.model.actual_pos_deg_hist]
            all_set_pos = [list(q) for q in self.model.set_pos_deg_hist]

            all_actual_vel = [list(q) for q in self.model.actual_vel_deg_s_hist]
            all_set_vel = [list(q) for q in self.model.set_vel_deg_s_hist]

        for i, panel in enumerate(self.panels):
            ts = all_ts[i]

            if not ts:
                panel.clear_plot()
                continue

            x = []
            actual_pos = []
            set_pos = []
            actual_vel = []
            set_vel = []

            for k, t in enumerate(ts):
                if 0.0 <= t <= duration:
                    x.append(t)
                    actual_pos.append(all_actual_pos[i][k])
                    set_pos.append(all_set_pos[i][k])
                    actual_vel.append(all_actual_vel[i][k])
                    set_vel.append(all_set_vel[i][k])

            if not x:
                panel.clear_plot()
                continue

            panel.plot_duration_sec = duration

            panel.update_plot(
                x=x,
                actual_pos=actual_pos,
                set_pos=set_pos,
                actual_vel=actual_vel,
                set_vel=set_vel,
            )

    # ========================================================
    # BUTTONS
    # ========================================================

    def clear_graph(self) -> None:
        duration = float(self.duration_spin.value())

        with self.model.lock:
            self.model.clear_history()
            self.model.plot_duration_sec = duration

        for panel in self.panels:
            panel.set_duration(duration)
            panel.clear_plot()

    def capture_all_plots(self) -> None:
        export_dir = os.path.expanduser(self.export_dir_edit.text().strip())

        if not export_dir:
            QtWidgets.QMessageBox.warning(
                self,
                "Lỗi",
                "Chưa chọn thư mục lưu ảnh.",
            )
            return

        os.makedirs(export_dir, exist_ok=True)

        index = self._next_capture_index(export_dir)

        saved_paths = []

        for i, panel in enumerate(self.panels):
            safe_joint = self._safe_filename(JOINT_NAMES[i])
            filename = f"joint_plot_{index:04d}_{safe_joint}.png"
            path = os.path.join(export_dir, filename)

            panel.save_png(path)
            saved_paths.append(path)

        self.lb_last_save.setText(
            f"Đã lưu bộ ảnh #{index:04d} tại: {export_dir}"
        )

        QtWidgets.QMessageBox.information(
            self,
            "Đã lưu ảnh",
            "Đã lưu 6 ảnh PNG:\n\n" + "\n".join(saved_paths),
        )

    def _browse_export_dir(self) -> None:
        current_dir = os.path.expanduser(self.export_dir_edit.text().strip())

        folder = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Chọn thư mục lưu ảnh",
            current_dir,
        )

        if folder:
            self.export_dir_edit.setText(folder)

    # ========================================================
    # EXPORT HELPERS
    # ========================================================

    def _next_capture_index(self, export_dir: str) -> int:
        max_idx = 0
        pattern = re.compile(r"joint_plot_(\d{4})_.*\.png$")

        for name in os.listdir(export_dir):
            m = pattern.match(name)
            if not m:
                continue

            try:
                idx = int(m.group(1))
                max_idx = max(max_idx, idx)
            except ValueError:
                pass

        return max_idx + 1

    @staticmethod
    def _safe_filename(name: str) -> str:
        return re.sub(r"[^a-zA-Z0-9_]+", "_", name)

    # ========================================================
    # UTILS
    # ========================================================

    @staticmethod
    def _rad_to_deg(value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        return math.degrees(float(value))

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def shutdown(self) -> None:
        try:
            self.timer_table.stop()
            self.timer_plot.stop()
        except Exception:
            pass

        try:
            self.ros_running = False

            if self.ros_node is not None:
                self.ros_node.destroy_node()
                self.ros_node = None

            if self.owns_rclpy and rclpy.ok():
                rclpy.shutdown()

        except Exception:
            pass