import math
import os
import time
from typing import Dict, Iterable, List, Optional

try:
    from PyQt5 import uic
    from PyQt5.QtCore import QTimer, pyqtSignal
    from PyQt5.QtWidgets import QLabel, QLineEdit, QMainWindow, QPushButton, QTextEdit
except ImportError:  # pragma: no cover - convenience for Windows/dev machines.
    from PyQt6 import uic
    from PyQt6.QtCore import QTimer, pyqtSignal
    from PyQt6.QtWidgets import QLabel, QLineEdit, QMainWindow, QPushButton, QTextEdit

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - direct Python run outside ROS 2.
    get_package_share_directory = None

try:
    from std_srvs.srv import SetBool
except ImportError:  # pragma: no cover - direct GUI test without ROS 2.
    SetBool = None

try:
    from robot_hardware_interface.srv import Home, Jog, RunAxis, StopAll, StopAxis
except ImportError:  # pragma: no cover - direct GUI test without hardware msgs.
    Home = Jog = RunAxis = StopAll = StopAxis = None

from .rviz_embedder import RvizEmbedder
from .robot_client import RobotClient, RobotClientError


JOINT_COUNT = 6

ERROR_ALL = 0x00000001
SOF_LIMIT_P = 0x00000008
SOF_LIMIT_M = 0x00000010
EMG = 0x00010000
SERVO_ON = 0x00100000
ALARM = 0x00200000
ORG_SET_OK = 0x02000000
RUNNING = 0x08000000

NORMAL_LED_INACTIVE = "#808080"
NORMAL_LED_ACTIVE = "#00cc66"
FAULT_LED_NORMAL = "#95c7ea"
FAULT_LED_ACTIVE = "#ff3333"
STATUS_LED_INACTIVE = "#d50000"
STATUS_LED_ACTIVE = "#00c853"

NORMAL_AXIS_LED_MASKS = {
    "ServoOn": SERVO_ON,
    "Running": RUNNING,
    "OrgOK": ORG_SET_OK,
    "LimitPositive": SOF_LIMIT_P,
    "LimitNegative": SOF_LIMIT_M,
}
FAULT_AXIS_LED_MASKS = {
    "Alarm": ALARM,
    "EMG": EMG,
    "ErrorAll": ERROR_ALL,
}
AXIS_LED_WIDGETS = {
    "servo_on": ("Servo On", "ServoOn", SERVO_ON, "normal"),
    "running": ("Running", "Running", RUNNING, "normal"),
    "origin_ok": ("Origin OK", "OrgOK", ORG_SET_OK, "normal"),
    "limit_p": ("Limit +", "LimitPositive", SOF_LIMIT_P, "normal"),
    "limit_m": ("Limit -", "LimitNegative", SOF_LIMIT_M, "normal"),
    "alarm": ("Alarm", "Alarm", ALARM, "fault"),
    "emg": ("EMG", "EMG", EMG, "fault"),
    "error_all": ("Error All", "ErrorAll", ERROR_ALL, "fault"),
}
NAVIGATION_PAGES = (
    ("btnHome", 0),
    ("btnMain", 1),
    ("btnRobot", 2),
    ("btnVision", 3),
    ("btnSetting", 4),
    ("btnLog", 5),
)
ROBOT_SERVO_ALL_SERVICE = "/robot_hw/servo_all"
ROBOT_JOG_SERVICE = "/robot_hw/jog"
ROBOT_HOME_SERVICE = "/robot_hw/home"
ROBOT_RUN_AXIS_SERVICE = "/robot_hw/run_axis"
ROBOT_STOP_AXIS_SERVICE = "/robot_hw/stop_axis"
ROBOT_STOP_ALL_SERVICE = "/robot_hw/stop_all"


def resolve_ui_path() -> str:
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory("robot_gui")
            candidate = os.path.join(share_dir, "ui", "robot_gui.ui")
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass

    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate = os.path.join(package_dir, "ui", "robot_gui.ui")
    if os.path.exists(candidate):
        return candidate

    cwd_candidate = os.path.abspath(os.path.join("robot_gui", "ui", "robot_gui.ui"))
    if os.path.exists(cwd_candidate):
        return cwd_candidate

    raise FileNotFoundError("Cannot find robot_gui.ui")


def set_led_color(widget: QLabel, color: str, border: str = "#3a3a3a") -> None:
    if hasattr(widget, "setVisible"):
        widget.setVisible(True)
    if hasattr(widget, "setMinimumSize"):
        widget.setMinimumSize(14, 14)
    if hasattr(widget, "setMaximumSize"):
        widget.setMaximumSize(18, 18)
    widget.setStyleSheet(
        f"background-color: {color}; border: 1px solid {border}; border-radius: 7px;"
    )


def set_led_state(
    widget: QLabel,
    active: bool,
    normal_color: str,
    active_color: str,
    normal_border: str,
    active_border: str,
) -> None:
    color = active_color if active else normal_color
    border = active_border if active else normal_border
    set_led_color(widget, color, border)


class RobotMainWindow(QMainWindow):
    flags_received = pyqtSignal(list)

    def __init__(self, ros_node=None, ui_path: Optional[str] = None):
        super().__init__()
        self.ros_node = ros_node
        self.client = RobotClient()
        self.ui_path = ui_path or resolve_ui_path()
        self.axis_to_robot_id: Dict[int, int] = {axis: axis - 1 for axis in range(1, JOINT_COUNT + 1)}
        self.joint_names = [f"joint_{i}" for i in range(1, JOINT_COUNT + 1)]
        self.commanded_deg = [0.0] * JOINT_COUNT
        self._last_jog_ms: Dict[int, int] = {}
        self._arm_publisher = None
        self._gripper_publisher = None
        self._servo_all_client = None
        self._jog_client = None
        self._home_client = None
        self._run_axis_client = None
        self._stop_axis_client = None
        self._stop_all_client = None
        self._robot_servo_on = False
        self._robot_enable_style = ""
        self._robot_disable_style = ""
        self.rviz_embedder = None
        self.embed_rviz = True
        self.initial_page = -1
        self._rviz_start_requested = False
        self.axis_leds: Dict[int, Dict[str, QLabel]] = {}

        uic.loadUi(self.ui_path, self)
        self.flags_received.connect(self.update_all_axis_leds_from_flags)
        self._read_ros_parameters()
        self._setup_ros_publishers()
        self._setup_ros_clients()
        self._setup_axis_leds()
        self._setup_defaults()
        self._connect_navigation()
        self._connect_robot_controls()
        self._connect_axis_controls()
        self._connect_log_controls()
        self._setup_rviz_embedder()
        self.update_connection_state(False)
        self.log_system(f"Loaded UI: {self.ui_path}")

    def _read_ros_parameters(self) -> None:
        if self.ros_node is None:
            return
        for name, default in (
            ("joint_names", self.joint_names),
            ("axis_ids", list(range(JOINT_COUNT))),
            ("robot_ip", "192.168.2.50"),
            ("robot_port", 5000),
            ("ping_timeout_ms", 2),
            ("embed_rviz", True),
            ("initial_page", -1),
        ):
            try:
                self.ros_node.declare_parameter(name, default)
            except Exception:
                pass

        try:
            joint_names = list(self.ros_node.get_parameter("joint_names").value)
            axis_ids = list(self.ros_node.get_parameter("axis_ids").value)
            self.joint_names = joint_names[:JOINT_COUNT]
            self.axis_to_robot_id = {axis + 1: int(axis_ids[axis]) for axis in range(min(JOINT_COUNT, len(axis_ids)))}
            self._set_text("txtRobotIP", str(self.ros_node.get_parameter("robot_ip").value))
            self._set_text("txtRobotPort", str(self.ros_node.get_parameter("robot_port").value))
            self._set_text("txtTcpTimeoutMs", str(self.ros_node.get_parameter("ping_timeout_ms").value))
            self.embed_rviz = self._to_bool(self.ros_node.get_parameter("embed_rviz").value, True)
            self.initial_page = self._to_int(self.ros_node.get_parameter("initial_page").value, -1)
        except Exception as exc:
            self.log_ros2(f"Parameter read warning: {exc}")

    def _setup_ros_publishers(self) -> None:
        if self.ros_node is None:
            return
        try:
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            self._joint_trajectory_msg = JointTrajectory
            self._joint_trajectory_point_msg = JointTrajectoryPoint
            self._arm_publisher = self.ros_node.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
            self._gripper_publisher = self.ros_node.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        except Exception as exc:
            self.log_ros2(f"ROS publisher disabled: {exc}")

    def _setup_ros_clients(self) -> None:
        if self.ros_node is None:
            return
        client_specs = (
            ("_servo_all_client", SetBool, ROBOT_SERVO_ALL_SERVICE),
            ("_jog_client", Jog, ROBOT_JOG_SERVICE),
            ("_home_client", Home, ROBOT_HOME_SERVICE),
            ("_run_axis_client", RunAxis, ROBOT_RUN_AXIS_SERVICE),
            ("_stop_axis_client", StopAxis, ROBOT_STOP_AXIS_SERVICE),
            ("_stop_all_client", StopAll, ROBOT_STOP_ALL_SERVICE),
        )
        for attr_name, srv_type, service_name in client_specs:
            if srv_type is None:
                self.log_ros2(f"{service_name} type is not available")
                continue
            try:
                setattr(self, attr_name, self.ros_node.create_client(srv_type, service_name))
            except Exception as exc:
                self.log_ros2(f"{service_name} client disabled: {exc}")

    def _setup_defaults(self) -> None:
        self._capture_robot_enable_styles()
        self._set_text("txtRobotIP", self._text("txtRobotIP", "192.168.2.50") or "192.168.2.50")
        self._set_text("txtRobotPort", self._text("txtRobotPort", "5000") or "5000")
        self._set_text("txtTcpTimeoutMs", self._text("txtTcpTimeoutMs", "2") or "2")
        self._set_text("PingStatus_value", "-- ms")
        self._set_text("PlannerStatus_value", "RL|Cartesian")

        for axis in range(1, JOINT_COUNT + 1):
            self._set_text(f"lblAxis{axis}Name", f"Joint {axis}")
            self._set_text(f"lblAxis{axis}ActualPosUnit", "deg")
            self._set_text(f"lblAxis{axis}CommandPosUnit", "deg")
            self._set_text(f"txtAxis{axis}ActualPos", "0.000")
            self._set_text(f"txtAxis{axis}ActualVel", "0.000")
            self._set_text(f"txtAxis{axis}CommandPos", "0.000")
            self._set_text(f"txtAxis{axis}CommandVel", "1.000")
            actual_pos = self._line_edit(f"txtAxis{axis}ActualPos")
            actual_vel = self._line_edit(f"txtAxis{axis}ActualVel")
            if actual_pos is not None:
                actual_pos.setReadOnly(True)
            if actual_vel is not None:
                actual_vel.setReadOnly(True)
            self.update_axis_status_leds(axis, 0)
        self.update_robot_enable_button(False)

    def _setup_axis_leds(self) -> None:
        self.axis_leds = {}
        for axis in range(1, JOINT_COUNT + 1):
            self.axis_leds[axis] = {}
            for led_key, (label, suffix, _mask, _kind) in AXIS_LED_WIDGETS.items():
                object_name = f"ledAxis{axis}{suffix}"
                widget = self.findChild(QLabel, object_name)
                if widget is None:
                    self.log_ros2(f"Missing LED widget: Axis {axis} {label} ({object_name})")
                    continue
                widget.setVisible(True)
                widget.setMinimumSize(14, 14)
                widget.setMaximumSize(18, 18)
                widget.setText("")
                self.axis_leds[axis][led_key] = widget
        self.log_ros2(f"Mapped {sum(len(leds) for leds in self.axis_leds.values())} axis LED widgets")

    def _connect_navigation(self) -> None:
        stacked = getattr(self, "stackedWidget_MainPages", None)
        if stacked is None:
            return
        for button_name, page_index in NAVIGATION_PAGES:
            button = self._button(button_name)
            if button is not None:
                button.setCheckable(True)
                button.clicked.connect(lambda checked=False, index=page_index: stacked.setCurrentIndex(index))
        if 0 <= self.initial_page < stacked.count():
            stacked.setCurrentIndex(self.initial_page)
        stacked.currentChanged.connect(self.update_tab_buttons)
        self.update_tab_buttons(stacked.currentIndex())

    def update_tab_buttons(self, current_index: int) -> None:
        for button_name, page_index in NAVIGATION_PAGES:
            button = self._button(button_name)
            if button is not None:
                button.setChecked(page_index == current_index)
        if current_index == 1 and self.rviz_embedder is not None:
            QTimer.singleShot(0, self._maybe_start_rviz_embedder)
            QTimer.singleShot(250, self.rviz_embedder.refresh_embedded_rviz)

    def _connect_robot_controls(self) -> None:
        self._connect_clicked("btnConnectRobot", self.connect_robot)
        self._connect_clicked("btnDisconnectRobot", self.disconnect_robot)
        self._connect_clicked("btnPingRobot", self.ping_robot)
        self._connect_clicked("btnRobotEnable", self.toggle_robot_servo)
        self._connect_clicked("btnRobotDisable", self.disable_robot)
        self._connect_clicked("btnGripperOpen", self.gripper_open)
        self._connect_clicked("btnGripperClose", self.gripper_close)
        self._connect_clicked("btnStartTask", lambda: self.log_system("Start task requested; task backend is not connected yet."))
        self._connect_clicked("btnStopTask", self.emergency_stop)
        self._connect_clicked("btnResetTask", lambda: self.log_ros2("Reset task requested; reset service is not wired in this GUI"))

    def _connect_axis_controls(self) -> None:
        for axis in range(1, JOINT_COUNT + 1):
            self._connect_clicked(f"btnAxis{axis}Enable", lambda checked=False, a=axis: self.enable_axis(a))
            self._connect_clicked(f"btnAxis{axis}Home", lambda checked=False, a=axis: self.home_axis(a))
            self._connect_clicked(f"btnAxis{axis}AlarmReset", lambda checked=False, a=axis: self.reset_axis(a))
            self._connect_clicked(f"btnAxis{axis}Run", lambda checked=False, a=axis: self.run_absolute(a))
            self._connect_clicked(f"btnAxis{axis}Stop", lambda checked=False, a=axis: self.stop_axis(a))

            positive = self._button(f"btnAxis{axis}JogPositive")
            negative = self._button(f"btnAxis{axis}JogNegative")
            if positive is not None:
                positive.pressed.connect(lambda a=axis: self.start_jog(a, 1))
                positive.released.connect(lambda a=axis: self.stop_jog(a))
            if negative is not None:
                negative.pressed.connect(lambda a=axis: self.start_jog(a, -1))
                negative.released.connect(lambda a=axis: self.stop_jog(a))

    def _connect_log_controls(self) -> None:
        self._connect_clicked("btnClearLog", self.clear_logs)
        self._connect_clicked("btnSaveLog", lambda: self.log_system("Save log requested; file save dialog is not implemented yet."))
        self._connect_clicked("btnExportLog", lambda: self.log_system("Export log requested; exporter is not implemented yet."))
        self._connect_clicked("btnLoadConfig", lambda: self.log_system("Load config requested; config backend is not implemented yet."))
        self._connect_clicked("btnSaveConfig", lambda: self.log_system("Save config requested; config backend is not implemented yet."))

    def _setup_rviz_embedder(self) -> None:
        parent_widget = getattr(self, "embeddedRvizWidget", None)
        if parent_widget is None:
            self.log_ros2("embeddedRvizWidget not found. RViz embedding disabled.")
            return

        placeholder_label = getattr(self, "labelRvizPlaceholder", None)
        if not self.embed_rviz:
            self._set_rviz_placeholder("RViz disabled")
            self.log_ros2("Embedded RViz disabled by parameter embed_rviz=false")
            return

        rviz_config_path = self._resolve_rviz_config_path()
        self.rviz_embedder = RvizEmbedder(
            parent_widget=parent_widget,
            placeholder_label=placeholder_label,
            rviz_config_path=rviz_config_path,
            log_callback=self.log_ros2,
            parent=self,
        )
        if rviz_config_path:
            self.log_ros2(f"Embedded RViz config: {rviz_config_path}")
        else:
            self.log_ros2("Embedded RViz config will be resolved by RvizEmbedder")
        self._set_rviz_placeholder("RViz waiting for Main page")
        QTimer.singleShot(500, self._maybe_start_rviz_embedder)

    def _maybe_start_rviz_embedder(self) -> None:
        if self.rviz_embedder is None or self._rviz_start_requested:
            return
        parent_widget = getattr(self, "embeddedRvizWidget", None)
        stacked = getattr(self, "stackedWidget_MainPages", None)
        if parent_widget is None:
            return
        main_page_active = stacked is None or stacked.currentIndex() == 1
        widget_ready = (
            main_page_active
            and parent_widget.isVisible()
            and parent_widget.window().isVisible()
            and parent_widget.width() > 10
            and parent_widget.height() > 10
        )
        self.log_ros2(
            "RViz start check: "
            f"main_active={main_page_active}, "
            f"widget_visible={parent_widget.isVisible()}, "
            f"window_visible={parent_widget.window().isVisible()}, "
            f"size={parent_widget.width()}x{parent_widget.height()}"
        )
        if not widget_ready:
            self._set_rviz_placeholder("RViz waiting for visible widget")
            QTimer.singleShot(500, self._maybe_start_rviz_embedder)
            return
        self._rviz_start_requested = True
        QTimer.singleShot(500, self.rviz_embedder.start)

    def _set_rviz_placeholder(self, text: str) -> None:
        placeholder_label = getattr(self, "labelRvizPlaceholder", None)
        if placeholder_label is not None:
            placeholder_label.setText(text)
            placeholder_label.show()

    def _resolve_rviz_config_path(self) -> Optional[str]:
        return RvizEmbedder.resolve_moveit_rviz_config_path(self.log_ros2)

    def connect_robot(self) -> None:
        ip = self._text("txtRobotIP", "192.168.2.50")
        port = self._to_int(self._text("txtRobotPort", "5000"), 5000)
        timeout_ms = self._to_int(self._text("txtTcpTimeoutMs", "2"), 2)
        try:
            self.client.connect_robot(ip, port, timeout_ms)
        except RobotClientError as exc:
            self.update_connection_state(False)
            self.log_hardware(str(exc))
            return
        self.update_connection_state(True)
        self.log_hardware(f"Connected to {ip}:{port}")

    def disconnect_robot(self) -> None:
        self.client.disconnect_robot()
        self.update_connection_state(False)
        self.log_hardware("Disconnected")

    def ping_robot(self) -> None:
        try:
            elapsed_ms = self.client.ping()
        except RobotClientError as exc:
            self.update_connection_state(False)
            self.log_hardware(str(exc))
            return
        self._set_text("PingStatus_value", f"{elapsed_ms:.1f} ms")
        self.log_hardware(f"Ping: {elapsed_ms:.1f} ms")

    def enable_robot(self) -> None:
        self.set_robot_servo_all(True)

    def disable_robot(self) -> None:
        self.set_robot_servo_all(False)

    def toggle_robot_servo(self) -> None:
        self.set_robot_servo_all(not self._robot_servo_on)

    def set_robot_servo_all(self, enabled: bool) -> None:
        if self._servo_all_client is None or SetBool is None:
            self.log_ros2(f"{ROBOT_SERVO_ALL_SERVICE} client is not available")
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        self._call_ros_service(
            ROBOT_SERVO_ALL_SERVICE,
            self._servo_all_client,
            request,
            lambda future: self._on_servo_all_response(future, bool(enabled)),
        )

    def _on_servo_all_response(self, future, requested_enabled: bool) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.log_ros2(f"{ROBOT_SERVO_ALL_SERVICE} response failed: {exc}")
            return
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        if success:
            self.log_ros2(f"{ROBOT_SERVO_ALL_SERVICE} OK: {message}")
        else:
            self.log_ros2(f"{ROBOT_SERVO_ALL_SERVICE} rejected data={requested_enabled}: {message}")

    def emergency_stop(self) -> None:
        if StopAll is None:
            self.log_ros2(f"{ROBOT_STOP_ALL_SERVICE} type is not available")
            return
        self._call_ros_service(ROBOT_STOP_ALL_SERVICE, self._stop_all_client, StopAll.Request())

    def enable_axis(self, axis: int) -> None:
        self.log_ros2("Axis enable uses robot-level /robot_hw/servo_all in this GUI")

    def home_axis(self, axis: int) -> None:
        robot_id = self.axis_to_robot_id[axis]
        if Home is None:
            self.log_ros2(f"{ROBOT_HOME_SERVICE} type is not available")
            return
        request = Home.Request()
        request.id = int(robot_id)
        self._call_ros_service(ROBOT_HOME_SERVICE, self._home_client, request)

    def reset_axis(self, axis: int) -> None:
        self.log_ros2(f"Alarm reset for Joint {axis} is not wired to a ROS service")
        self._set_fault_led(f"ledAxis{axis}Alarm", False)
        self._set_fault_led(f"ledAxis{axis}ErrorAll", False)

    def stop_axis(self, axis: int) -> None:
        robot_id = self.axis_to_robot_id[axis]
        if StopAxis is None:
            self.log_ros2(f"{ROBOT_STOP_AXIS_SERVICE} type is not available")
            return
        request = StopAxis.Request()
        request.id = int(robot_id)
        self._call_ros_service(ROBOT_STOP_AXIS_SERVICE, self._stop_axis_client, request)
        self._set_normal_led(f"ledAxis{axis}Running", False)

    def start_jog(self, axis: int, direction: int) -> None:
        now_ms = int(time.monotonic() * 1000.0)
        last_ms = self._last_jog_ms.get(axis)
        if last_ms is not None and now_ms - last_ms < 50:
            return
        self._last_jog_ms[axis] = now_ms
        robot_id = self.axis_to_robot_id[axis]
        if Jog is None:
            self.log_ros2(f"{ROBOT_JOG_SERVICE} type is not available")
            return
        request = Jog.Request()
        request.id = int(robot_id)
        request.vel = math.radians(self._velocity_deg_s(axis))
        request.dir = 1 if direction >= 0 else 0
        self._call_ros_service(ROBOT_JOG_SERVICE, self._jog_client, request)
        self._set_normal_led(f"ledAxis{axis}Running", True)

    def stop_jog(self, axis: int) -> None:
        self.stop_axis(axis)

    def run_absolute(self, axis: int) -> None:
        robot_id = self.axis_to_robot_id[axis]
        deg = self._to_float(self._text(f"txtAxis{axis}CommandPos", "0"), 0.0)
        self.commanded_deg[axis - 1] = deg
        if RunAxis is None:
            self.log_ros2(f"{ROBOT_RUN_AXIS_SERVICE} type is not available")
            return
        request = RunAxis.Request()
        request.id = int(robot_id)
        request.pos = math.radians(deg)
        request.vel = math.radians(self._velocity_deg_s(axis))
        self._call_ros_service(ROBOT_RUN_AXIS_SERVICE, self._run_axis_client, request)
        self.publish_joint_trajectory(axis)
        self._set_normal_led(f"ledAxis{axis}Running", True)
        self.log_hardware(f"Joint {axis} -> robot id {robot_id}: {deg:.3f} deg")

    def gripper_open(self) -> None:
        self.publish_gripper(0.04)
        self.log_ros2("Requested gripper open trajectory")

    def gripper_close(self) -> None:
        width = self._to_float(self._text("txtGripperPosition", "0.0"), 0.0)
        self.publish_gripper(max(width, 0.0))
        self.log_ros2("Requested gripper close trajectory")

    def publish_joint_trajectory(self, axis: int) -> None:
        if self._arm_publisher is None:
            return
        try:
            msg = self._joint_trajectory_msg()
            point = self._joint_trajectory_point_msg()
            msg.joint_names = list(self.joint_names)
            point.positions = [math.radians(value) for value in self.commanded_deg]
            point.time_from_start.sec = 2
            msg.points = [point]
            self._arm_publisher.publish(msg)
            self.log_ros2(f"Published /arm_controller/joint_trajectory for Joint {axis}")
        except Exception as exc:
            self.log_ros2(f"Publish joint trajectory failed: {exc}")

    def publish_gripper(self, width: float) -> None:
        if self._gripper_publisher is None:
            return
        try:
            msg = self._joint_trajectory_msg()
            point = self._joint_trajectory_point_msg()
            msg.joint_names = ["joint_gl", "joint_gr"]
            point.positions = [width / 2.0, width / 2.0]
            point.time_from_start.sec = 1
            msg.points = [point]
            self._gripper_publisher.publish(msg)
        except Exception as exc:
            self.log_ros2(f"Publish gripper trajectory failed: {exc}")

    def update_joint_state(self, names: Iterable[str], positions_rad: Iterable[float], velocities_rad_s: Iterable[float] = ()) -> None:
        position_by_name = dict(zip(names, positions_rad))
        velocity_by_name = dict(zip(names, velocities_rad_s))
        for axis, joint_name in enumerate(self.joint_names, start=1):
            if joint_name not in position_by_name:
                continue
            deg = math.degrees(float(position_by_name[joint_name]))
            vel_deg = math.degrees(float(velocity_by_name.get(joint_name, 0.0)))
            self._set_text(f"txtAxis{axis}ActualPos", f"{deg:.3f}")
            self._set_text(f"txtAxis{axis}ActualVel", f"{vel_deg:.3f}")
            self.commanded_deg[axis - 1] = deg

    def update_joint_state_mdeg(self, values_mdeg: Iterable[float]) -> None:
        for axis, mdeg in enumerate(list(values_mdeg)[:JOINT_COUNT], start=1):
            deg = float(mdeg) / 1000.0
            self._set_text(f"txtAxis{axis}ActualPos", f"{deg:.3f}")
            self.commanded_deg[axis - 1] = deg

    def update_connection_state(self, connected: bool) -> None:
        self._set_status_bar_led("ConnectStatus_led", connected)
        self._set_text("ConnectStatus_label", "Connected" if connected else "Disconnected")

    def on_robot_flags_msg(self, msg) -> None:
        self.update_all_axis_leds_from_flags(self.parse_robot_flags_msg(msg))

    def update_flag_status(self, msg) -> None:
        self.on_robot_flags_msg(msg)

    def receive_robot_flags(self, flags: Iterable[int]) -> None:
        axis_flags = [int(status) for status in flags]
        self.log_ros2(f"Received /robot_hw/flags: {[f'0x{status:08X}' for status in axis_flags]}")
        self.flags_received.emit(axis_flags)

    def parse_flags_msg(self, msg) -> List[int]:
        return self.parse_robot_flags_msg(msg)

    def parse_robot_flags_msg(self, msg) -> List[int]:
        axes = getattr(msg, "axes", None)
        if axes is not None:
            return [int(getattr(axis_flags, "status_f", 0)) for axis_flags in axes]

        self.log_ros2("/robot_hw/flags message has no axes field")
        return []

    def update_all_axis_leds_from_flags(self, flags: Iterable[int]) -> None:
        axis_flags = [int(status) for status in list(flags)[:JOINT_COUNT]]
        if len(axis_flags) < JOINT_COUNT:
            self.log_ros2(
                f"/robot_hw/flags has {len(axis_flags)} axes; expected {JOINT_COUNT}. Missing axes set inactive."
            )
            axis_flags.extend([0] * (JOINT_COUNT - len(axis_flags)))

        for axis_index, status in enumerate(axis_flags, start=1):
            self.update_axis_leds(axis_index, status)
        self.update_robot_enable_button(all((status & SERVO_ON) != 0 for status in axis_flags))

    def update_axis_status_from_robot_id(self, robot_id: int, status: int) -> None:
        for axis, mapped_robot_id in self.axis_to_robot_id.items():
            if int(mapped_robot_id) == int(robot_id):
                self.update_axis_status_leds(axis, status)
                return

    def update_robot_enable_from_status(self, status: int) -> None:
        self.update_robot_enable_button((int(status) & SERVO_ON) != 0)

    def update_robot_enable_button(self, servo_on: bool) -> None:
        self._robot_servo_on = bool(servo_on)
        self._set_status_bar_led("RobotEnableStatus_led", self._robot_servo_on)
        button = self._button("btnRobotEnable")
        if button is None:
            return
        if self._robot_servo_on:
            button.setText("Disable")
            button.setStyleSheet(self._robot_disable_style)
        else:
            button.setText("Enable")
            button.setStyleSheet(self._robot_enable_style)

    def update_axis_status_leds(self, axis: int, status: int) -> None:
        self.update_axis_leds(axis, status)

    def update_axis_leds(self, axis: int, status: int) -> None:
        if axis < 1 or axis > JOINT_COUNT:
            return
        status = int(status)
        axis_leds = getattr(self, "axis_leds", {}).get(axis, {})
        for led_key, (label, suffix, mask, kind) in AXIS_LED_WIDGETS.items():
            widget = axis_leds.get(led_key)
            if widget is None:
                widget = getattr(self, f"ledAxis{axis}{suffix}", None)
            if widget is None:
                self.log_ros2(f"Missing LED widget: Axis {axis} {label}")
                continue
            active = (status & mask) != 0
            if kind == "fault":
                if hasattr(self, "set_fault_led"):
                    self.set_fault_led(widget, active)
                else:
                    self._set_fault_led(f"ledAxis{axis}{suffix}", active)
            else:
                if hasattr(self, "set_normal_led"):
                    self.set_normal_led(widget, active)
                else:
                    self._set_normal_led(f"ledAxis{axis}{suffix}", active)

    def clear_logs(self) -> None:
        for name in ("txtSystemLog", "txtHardwareLog", "txtROS2Log"):
            widget = self._text_edit(name)
            if widget is not None:
                widget.clear()

    def log_system(self, message: str) -> None:
        self._append_log("txtSystemLog", message)
        self._set_text("txtMainLog", message)

    def log_hardware(self, message: str) -> None:
        self._append_log("txtHardwareLog", message)
        self._set_text("txtRobotHardwareLog", message)

    def log_ros2(self, message: str) -> None:
        self._append_log("txtROS2Log", message)
        if self.ros_node is not None:
            try:
                self.ros_node.get_logger().info(message)
            except Exception:
                pass

    def _safe_client_call(self, label: str, callback) -> bool:
        try:
            callback()
        except RobotClientError as exc:
            self.log_hardware(f"{label}: {exc}")
            return False
        except Exception as exc:
            self.log_hardware(f"{label}: unexpected error: {exc}")
            return False
        return True

    def _velocity_mdeg_s(self, axis: int) -> int:
        return max(1, int(round(abs(self._velocity_deg_s(axis)) * 1000.0)))

    def _velocity_deg_s(self, axis: int) -> float:
        return abs(self._to_float(self._text(f"txtAxis{axis}CommandVel", "1.0"), 1.0))

    def _call_ros_service(self, service_name: str, client, request, done_callback=None) -> bool:
        if client is None:
            self.log_ros2(f"Service {service_name} is not available")
            return False
        try:
            if not client.service_is_ready():
                self.log_ros2(f"Service {service_name} is not available")
                return False
            future = client.call_async(request)
            if hasattr(future, "add_done_callback"):
                future.add_done_callback(done_callback or (lambda fut, name=service_name: self._log_ros_service_response(name, fut)))
            self.log_ros2(f"Requested {service_name}")
        except Exception as exc:
            self.log_ros2(f"Service {service_name} request failed: {exc}")
            return False
        return True

    def _log_ros_service_response(self, service_name: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.log_ros2(f"Service {service_name} response failed: {exc}")
            return
        ok = bool(getattr(response, "ok", getattr(response, "success", False)))
        code = getattr(response, "error_code", 0)
        message = str(getattr(response, "message", ""))
        status = "OK" if ok else "ERR"
        self.log_ros2(f"{service_name} {status} (code={code}): {message}")

    def _connect_clicked(self, name: str, callback) -> None:
        button = self._button(name)
        if button is not None:
            button.clicked.connect(callback)

    def _button(self, name: str):
        return self.findChild(QPushButton, name)

    def _line_edit(self, name: str):
        return self.findChild(QLineEdit, name)

    def _text_edit(self, name: str):
        return self.findChild(QTextEdit, name)

    def _text(self, name: str, default: str = "") -> str:
        widget = getattr(self, name, None)
        if widget is None:
            return default
        if hasattr(widget, "text"):
            return widget.text()
        if hasattr(widget, "toPlainText"):
            return widget.toPlainText()
        return default

    def _set_text(self, name: str, value: str) -> None:
        widget = getattr(self, name, None)
        if widget is None:
            return
        if hasattr(widget, "setText"):
            widget.setText(value)
        elif hasattr(widget, "setPlainText"):
            widget.setPlainText(value)

    def _append_log(self, name: str, message: str) -> None:
        widget = self._text_edit(name)
        if widget is not None:
            widget.append(message)

    def _capture_robot_enable_styles(self) -> None:
        enable_button = self._button("btnRobotEnable")
        disable_button = self._button("btnRobotDisable")
        self._robot_enable_style = enable_button.styleSheet() if enable_button is not None else ""
        self._robot_disable_style = disable_button.styleSheet() if disable_button is not None else self._robot_enable_style

    def _set_led_state(self, name: str, active: bool, normal_color: str, active_color: str, normal_border: str, active_border: str) -> None:
        widget = getattr(self, name, None)
        if widget is None:
            self.log_ros2(f"Missing LED widget: {name}")
            return
        set_led_state(widget, active, normal_color, active_color, normal_border, active_border)

    def set_normal_led(self, widget, active: bool) -> None:
        set_led_state(widget, active, NORMAL_LED_INACTIVE, NORMAL_LED_ACTIVE, "#4b5563", "#006b2d")

    def _set_normal_led(self, name: str, active: bool) -> None:
        self._set_led_state(name, active, NORMAL_LED_INACTIVE, NORMAL_LED_ACTIVE, "#4b5563", "#006b2d")

    def set_fault_led(self, widget, active: bool) -> None:
        set_led_state(widget, active, FAULT_LED_NORMAL, FAULT_LED_ACTIVE, "#4b8db8", "#a00018")

    def _set_fault_led(self, name: str, active: bool) -> None:
        self._set_led_state(name, active, FAULT_LED_NORMAL, FAULT_LED_ACTIVE, "#4b8db8", "#a00018")

    def _set_status_bar_led(self, name: str, active: bool) -> None:
        self._set_led_state(name, active, STATUS_LED_INACTIVE, STATUS_LED_ACTIVE, "#8b0000", "#006b2d")

    def closeEvent(self, event) -> None:
        if self.rviz_embedder is not None:
            self.rviz_embedder.stop()
        super().closeEvent(event)

    @staticmethod
    def _to_int(value: str, default: int) -> int:
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _to_bool(value, default: bool) -> bool:
        if isinstance(value, bool):
            return value
        if value is None:
            return default
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def _to_float(value: str, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default
