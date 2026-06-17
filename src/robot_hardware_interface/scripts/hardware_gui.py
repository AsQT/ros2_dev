#!/usr/bin/env python3
"""robot_hw_gui.py

GUI kiểm tra phần cứng Robot TCP (Qt) – tối ưu để nhìn rõ 6 trục.

Nguyên tắc:
  - STATE: subscribe topics (/joint_states, /robot_hw/connected, /robot_hw/status_*)
  - COMMAND: gọi service trong robot_hardware_interface:
      /robot_hw/servo_on_axis (ServoOnAxis)
      /robot_hw/servo_on_all  (ServoOnAll)
      /robot_hw/jog           (Jog)
      /robot_hw/run_axis      (RunAxis)
      /robot_hw/home          (Home)
      /robot_hw/stop_axis     (StopAxis)
      /robot_hw/stop_all      (StopAll)

Mặc định dùng đơn vị ROS:
  - pos: rad, vel: rad/s

Tip: Nếu không thấy interface custom trong rqt/CLI, hãy source workspace:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_arm_ws/install/setup.bash
"""

import math
import sys
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt16MultiArray, String
#from robot_hardware_interface.msg import StatusFlags6

from qtpy import QtCore, QtGui, QtWidgets


# ---- Custom services (được generate trong pkg robot_hardware_interface) ----
try:
    from robot_hardware_interface.srv import (
        ServoOnAxis,
        ServoOnAll,
        Jog,
        RunAxis,
        Home,
        StopAxis,
        StopAll,
    )
except Exception as e:  # pragma: no cover
    ServoOnAxis = ServoOnAll = Jog = RunAxis = Home = StopAxis = StopAll = None
    _IMPORT_ERR = str(e)


@dataclass
class JointSnapshot:
    pos_rad: float = float("nan")
    vel_rad_s: float = float("nan")
    stamp_ros: float = 0.0
    wall_time: float = 0.0


def _rad_to_deg(x: float) -> float:
    return float(x) * 180.0 / math.pi


def _fmt(x: float, digits: int = 3) -> str:
    if x is None or math.isnan(x):
        return "--"
    fmt = f"{{:.{digits}f}}"
    return fmt.format(float(x))


class Backend(Node):
    """ROS backend chạy trong thread riêng."""

    def __init__(self):
        super().__init__("robot_hw_gui_backend")

        # --- params ---
        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("axis_ids", [0, 1, 2, 3, 4, 5])

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.axis_ids: List[int] = [int(x) for x in self.get_parameter("axis_ids").value]

        # --- state ---
        self._lock = threading.Lock()
        self.connected: bool = False
        self.status_flags: List[int] = []
        self.status_text: str = ""
        self.joints: Dict[str, JointSnapshot] = {}

        # --- subscribers ---
        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        self.create_subscription(Bool, "/robot_hw/connected", self._on_connected, 10)
        self.create_subscription(String, "/robot_hw/status_text", self._on_text, 10)

        # --- services ---
        # custom
        self.cli_servo_on_axis = self.create_client(ServoOnAxis, "/robot_hw/servo_on_axis") if ServoOnAxis else None
        self.cli_servo_on_all = self.create_client(ServoOnAll, "/robot_hw/servo_on_all") if ServoOnAll else None
        self.cli_jog = self.create_client(Jog, "/robot_hw/jog") if Jog else None
        self.cli_run_axis = self.create_client(RunAxis, "/robot_hw/run_axis") if RunAxis else None
        self.cli_home = self.create_client(Home, "/robot_hw/home") if Home else None
        self.cli_stop_axis = self.create_client(StopAxis, "/robot_hw/stop_axis") if StopAxis else None
        self.cli_stop_all = self.create_client(StopAll, "/robot_hw/stop_all") if StopAll else None

    # ---- subscribers ----
    def _on_connected(self, msg: Bool) -> None:
        with self._lock:
            self.connected = bool(msg.data)

    def _on_text(self, msg: String) -> None:
        with self._lock:
            self.status_text = msg.data

    def _on_js(self, msg: JointState) -> None:
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        now_wall = time.time()
        with self._lock:
            for idx, name in enumerate(msg.name):
                pos = msg.position[idx] if idx < len(msg.position) else float("nan")
                vel = msg.velocity[idx] if idx < len(msg.velocity) else float("nan")
                self.joints[name] = JointSnapshot(
                    pos_rad=float(pos),
                    vel_rad_s=float(vel),
                    stamp_ros=now_ros,
                    wall_time=now_wall,
                )

    # ---- snapshot ----
    def snapshot(self) -> Tuple[bool, List[int], str, Dict[str, JointSnapshot]]:
        with self._lock:
            return (self.connected, list(self.status_flags), str(self.status_text), dict(self.joints))

    # ---- sync service helpers (được gọi trong worker thread, không block UI) ----
    def _wait_service(self, cli, timeout_s: float) -> bool:
        if cli is None:
            return False
        if cli.service_is_ready():
            return True
        return bool(cli.wait_for_service(timeout_sec=timeout_s))

    def servo_on_axis(self, axis_id: int, state: int, timeout_s: float = 1.5) -> Tuple[bool, int, str]:
        if self.cli_servo_on_axis is None:
            return (False, 10, f"ServoOnAxis not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_servo_on_axis, timeout_s):
            return (False, 1, "service not available")
        req = ServoOnAxis.Request()
        req.id = int(axis_id)
        req.state = int(state) & 0x01
        fut = self.cli_servo_on_axis.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))

    def servo_on_all(self, state: int, timeout_s: float = 1.5) -> Tuple[bool, int, str]:
        if self.cli_servo_on_all is None:
            return (False, 10, f"ServoOnAll not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_servo_on_all, timeout_s):
            return (False, 1, "service not available")
        req = ServoOnAll.Request()
        req.state = int(state) & 0x01
        fut = self.cli_servo_on_all.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))

    def jog(self, axis_id: int, vel_rad_s: float, direction01: int, timeout_s: float = 1.5) -> Tuple[bool, int, str]:
        if self.cli_jog is None:
            return (False, 10, f"Jog not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_jog, timeout_s):
            return (False, 1, "service not available")
        req = Jog.Request()
        req.id = int(axis_id)
        req.vel = float(vel_rad_s)
        req.dir = int(direction01) & 0x01
        fut = self.cli_jog.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))

    def run_axis(self, axis_id: int, pos_rad: float, vel_rad_s: float, timeout_s: float = 1.5) -> Tuple[bool, int, str]:
        if self.cli_run_axis is None:
            return (False, 10, f"RunAxis not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_run_axis, timeout_s):
            return (False, 1, "service not available")
        req = RunAxis.Request()
        req.id = int(axis_id)
        req.pos = float(pos_rad)
        req.vel = float(vel_rad_s)
        fut = self.cli_run_axis.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))

    def home(self, axis_id: int, timeout_s: float = 2.0) -> Tuple[bool, int, str]:
        if self.cli_home is None:
            return (False, 10, f"Home not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_home, timeout_s):
            return (False, 1, "service not available")
        req = Home.Request()
        req.id = int(axis_id)
        fut = self.cli_home.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))

    def stop_axis(self, axis_id: int, timeout_s: float = 1.5) -> Tuple[bool, int, str]:
        if self.cli_stop_axis is None:
            return (False, 10, f"StopAxis not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_stop_axis, timeout_s):
            return (False, 1, "service not available")
        req = StopAxis.Request()
        req.id = int(axis_id)
        fut = self.cli_stop_axis.call_async(req)
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))

    def stop_all(self, timeout_s: float = 1.5) -> Tuple[bool, int, str]:
        if self.cli_stop_all is None:
            return (False, 10, f"StopAll not available: {_IMPORT_ERR}")
        if not self._wait_service(self.cli_stop_all, timeout_s):
            return (False, 1, "service not available")
        fut = self.cli_stop_all.call_async(StopAll.Request())
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.ok), int(res.error_code), str(res.message))


class _WorkerSignals(QtCore.QObject):
    done = QtCore.Signal(bool, int, str)


class _CallWorker(QtCore.QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = _WorkerSignals()

    @QtCore.Slot()
    def run(self):
        try:
            ok, code, msg = self.fn(*self.args, **self.kwargs)
        except Exception as e:  # pragma: no cover
            ok, code, msg = False, 255, f"exception: {e}"
        self.signals.done.emit(bool(ok), int(code), str(msg))


class HoldButton(QtWidgets.QPushButton):
    """Button cho Jog: nhấn giữ -> start, nhả -> stop."""

    pressed_hold = QtCore.Signal()
    released_hold = QtCore.Signal()

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        super().mousePressEvent(e)
        if e.button() == QtCore.Qt.LeftButton:
            self.pressed_hold.emit()

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent) -> None:
        super().mouseReleaseEvent(e)
        if e.button() == QtCore.Qt.LeftButton:
            self.released_hold.emit()


class AxisPanel(QtWidgets.QGroupBox):
    def __init__(self, axis_id: int, joint_name: str, parent=None):
        super().__init__(parent)
        self.axis_id = axis_id
        self.joint_name = joint_name
        self.setTitle(f"Joint {axis_id} ")

        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(10, 10, 10, 10)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(6)

        # big readouts
        self.lb_pos_deg = QtWidgets.QLabel("Positon: -- deg")
        self.lb_pos_deg.setStyleSheet("font-size: 16pt; font-weight: 700; color: #0B1F35;")
        self.lb_pos_rad = QtWidgets.QLabel("-- rad")
        self.lb_pos_rad.setStyleSheet("font-size: 11pt; color: #244B6B;")
        self.lb_vel = QtWidgets.QLabel("vel: -- rad/s")
        self.lb_vel.setStyleSheet("font-size: 11pt; color: #244B6B;")
        self.lb_age = QtWidgets.QLabel("age: -- ms")
        self.lb_age.setStyleSheet("font-size: 10pt; color: #6B7C8A;")

        g.addWidget(self.lb_pos_deg, 0, 0, 1, 3)
        g.addWidget(self.lb_pos_rad, 0, 1, 1, 1)
        g.addWidget(self.lb_vel, 0, 2, 1, 1)
        g.addWidget(self.lb_age, 0, 3, 1, 1)

        # targets
        self.ed_pos = QtWidgets.QDoubleSpinBox()
        self.ed_pos.setRange(-1000.0, 1000.0)
        self.ed_pos.setDecimals(3)
        self.ed_pos.setSingleStep(0.05)
        self.ed_pos.setSuffix(" rad")

        self.ed_vel = QtWidgets.QDoubleSpinBox()
        self.ed_vel.setRange(0.0, 1000.0)
        self.ed_vel.setDecimals(3)
        self.ed_vel.setSingleStep(0.05)
        self.ed_vel.setValue(0.5)
        self.ed_vel.setSuffix(" rad/s")

        g.addWidget(QtWidgets.QLabel("Target pos"), 2, 0)
        g.addWidget(self.ed_pos, 2, 1)
        g.addWidget(QtWidgets.QLabel("Vel"), 2, 2)
        g.addWidget(self.ed_vel, 2, 3)

        # buttons row
        self.btn_servo_on = QtWidgets.QPushButton("Servo ON")
        self.btn_servo_off = QtWidgets.QPushButton("Servo OFF")
        self.btn_home = QtWidgets.QPushButton("Home")
        self.btn_run = QtWidgets.QPushButton("Run")
        self.btn_stop = QtWidgets.QPushButton("Stop")

        self.btn_run.setStyleSheet("font-weight: 700;")
        self.btn_stop.setStyleSheet("font-weight: 700; color: white; background: #C62828;")

        g.addWidget(self.btn_servo_on, 3, 0)
        g.addWidget(self.btn_servo_off, 3, 1)
        g.addWidget(self.btn_home, 3, 2)
        g.addWidget(self.btn_run, 2, 4)
        g.addWidget(self.btn_stop, 3, 4)

        # jog row
        self.ed_jog_vel = QtWidgets.QDoubleSpinBox()
        self.ed_jog_vel.setRange(0.0, 1000.0)
        self.ed_jog_vel.setDecimals(3)
        self.ed_jog_vel.setSingleStep(0.05)
        self.ed_jog_vel.setValue(0.3)
        self.ed_jog_vel.setSuffix(" rad/s")

        self.btn_jog_neg = HoldButton("◀ Jog-")
        self.btn_jog_pos = HoldButton("Jog+ ▶")
        self.btn_jog_neg.setAutoRepeat(False)
        self.btn_jog_pos.setAutoRepeat(False)

        g.addWidget(QtWidgets.QLabel("Jog vel"), 4, 0)
        g.addWidget(self.ed_jog_vel, 4, 1)
        g.addWidget(self.btn_jog_neg, 4, 2)
        g.addWidget(self.btn_jog_pos, 4, 3)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, backend: Backend):
        super().__init__()
        self.backend = backend
        self.pool = QtCore.QThreadPool.globalInstance()

        self.setWindowTitle("Robot TCP HW GUI (Services)")
        self.resize(1200, 720)

        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        v = QtWidgets.QVBoxLayout(cw)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(10)

        # top bar
        top = QtWidgets.QHBoxLayout()
        self.lb_conn = QtWidgets.QLabel("DISCONNECTED")
        self.lb_conn.setStyleSheet("font-size: 12pt; font-weight: 800; color: white; background:#455A64; padding:6px 10px; border-radius:10px;")
        self.lb_status = QtWidgets.QLabel("status: --")
        self.lb_status.setStyleSheet("font-size: 11pt; color:#0B1F35;")

        self.btn_servo_all_on = QtWidgets.QPushButton("Servo ALL ON")
        self.btn_servo_all_off = QtWidgets.QPushButton("Servo ALL OFF")
        self.btn_stop_all = QtWidgets.QPushButton("STOP ALL")
        self.btn_stop_all.setStyleSheet("font-weight: 900; color: white; background:#B71C1C; padding:6px 14px;")

        top.addWidget(self.lb_conn)
        top.addSpacing(10)
        top.addWidget(self.lb_status, 1)
        top.addWidget(self.btn_servo_all_on)
        top.addWidget(self.btn_servo_all_off)
        top.addWidget(self.btn_stop_all)
        v.addLayout(top)

        # axis grid
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        v.addLayout(grid, 1)

        self.axis_panels: List[AxisPanel] = []
        for i in range(min(6, len(self.backend.axis_ids), len(self.backend.joint_names))):
            axis_id = self.backend.axis_ids[i]
            joint = self.backend.joint_names[i]
            p = AxisPanel(axis_id, joint)
            self.axis_panels.append(p)
            r = i // 2
            c = (i % 2) * 1
            grid.addWidget(p, r, c)

            # wire buttons
            p.btn_servo_on.clicked.connect(lambda _=False, a=axis_id: self._call(self.backend.servo_on_axis, a, 1))
            p.btn_servo_off.clicked.connect(lambda _=False, a=axis_id: self._call(self.backend.servo_on_axis, a, 0))
            p.btn_home.clicked.connect(lambda _=False, a=axis_id: self._call(self.backend.home, a))
            p.btn_run.clicked.connect(lambda _=False, pp=p: self._run_axis(pp))
            p.btn_stop.clicked.connect(lambda _=False, a=axis_id: self._call(self.backend.stop_axis, a))

            p.btn_jog_neg.pressed_hold.connect(lambda a=axis_id, pp=p: self._jog_start(pp, a, 0))
            p.btn_jog_pos.pressed_hold.connect(lambda a=axis_id, pp=p: self._jog_start(pp, a, 1))
            p.btn_jog_neg.released_hold.connect(lambda a=axis_id: self._call(self.backend.stop_axis, a))
            p.btn_jog_pos.released_hold.connect(lambda a=axis_id: self._call(self.backend.stop_axis, a))

        # timers
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)  # 10 Hz

        # global buttons
        self.btn_servo_all_on.clicked.connect(lambda: self._call(self.backend.servo_on_all, 1))
        self.btn_servo_all_off.clicked.connect(lambda: self._call(self.backend.servo_on_all, 0))
        self.btn_stop_all.clicked.connect(lambda: self._call(self.backend.stop_all))

        # bottom log
        self.txt_log = QtWidgets.QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(500)
        v.addWidget(self.txt_log, 0)

    def _log(self, ok: bool, code: int, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        s = f"[{ts}] {'OK' if ok else 'ERR'} (code={code}): {msg}"
        self.txt_log.appendPlainText(s)

    def _call(self, fn, *args, **kwargs) -> None:
        w = _CallWorker(fn, *args, **kwargs)
        w.signals.done.connect(self._log)
        self.pool.start(w)

    def _run_axis(self, panel: AxisPanel) -> None:
        pos = float(panel.ed_pos.value())
        vel = float(panel.ed_vel.value())
        self._call(self.backend.run_axis, panel.axis_id, pos, vel)

    def _jog_start(self, panel: AxisPanel, axis_id: int, dir01: int) -> None:
        vel = float(panel.ed_jog_vel.value())
        self._call(self.backend.jog, axis_id, vel, dir01)

    def _refresh(self) -> None:
        connected, _flags, text, joints = self.backend.snapshot()

        if connected:
            self.lb_conn.setText("CONNECTED")
            self.lb_conn.setStyleSheet("font-size: 12pt; font-weight: 800; color: white; background:#2E7D32; padding:6px 10px; border-radius:10px;")
        else:
            self.lb_conn.setText("DISCONNECTED")
            self.lb_conn.setStyleSheet("font-size: 12pt; font-weight: 800; color: white; background:#455A64; padding:6px 10px; border-radius:10px;")

        self.lb_status.setText(f"status: {text}")

        now = time.time()
        for p in self.axis_panels:
            js = joints.get(p.joint_name)
            if js is None:
                p.lb_pos_rad.setText("pos: -- rad")
                p.lb_pos_deg.setText("-- deg")
                p.lb_vel.setText("vel: -- rad/s")
                p.lb_age.setText("age: -- ms")
                continue

            p.lb_pos_rad.setText(f"pos: {_fmt(js.pos_rad, 3)} rad")
            p.lb_pos_deg.setText(f"{_fmt(_rad_to_deg(js.pos_rad), 2)} deg")
            p.lb_vel.setText(f"vel: {_fmt(js.vel_rad_s, 3)} rad/s")
            age_ms = (now - js.wall_time) * 1000.0
            p.lb_age.setText(f"age: {age_ms:.0f} ms")


def main(argv=None):
    argv = argv if argv is not None else sys.argv
    rclpy.init(args=argv)

    backend = Backend()
    exec_ = MultiThreadedExecutor(num_threads=2)
    exec_.add_node(backend)

    th = threading.Thread(target=exec_.spin, daemon=True)
    th.start()

    app = QtWidgets.QApplication(argv)
    win = MainWindow(backend)
    win.show()
    rc = app.exec_()

    backend.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
