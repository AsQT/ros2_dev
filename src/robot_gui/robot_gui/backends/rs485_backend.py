import math
import re
import shutil
import subprocess
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

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
    from robot_hardware_interface.msg import FlagStatus
except Exception as e:  # pragma: no cover
    ServoOnAxis = ServoOnAll = Jog = RunAxis = Home = StopAxis = StopAll = None
    FlagStatus = None
    _IMPORT_ERR = str(e)

@dataclass
class JointSnapshot:
    pos_rad: float = float("nan")
    vel_rad_s: float = float("nan")
    stamp_ros: float = 0.0
    wall_time: float = 0.0

@dataclass
class AxisStatus:
    servo_on: bool = False
    error_all: bool = False
    org_ok: bool = False
    motionning: bool = False
    org_retunning: bool = False
    limit_pos: bool = False
    limit_neg: bool = False
    org_sensor: bool = False
    alarm_rst: bool = False
    emg: bool = False
    stop: bool = False
    communi_err: bool = False
    status_f: int = 0

def rad_to_deg(x: float) -> float:
    return float(x) * 180.0 / math.pi

def fmt(x: float, digits: int = 3) -> str:
    if x is None or math.isnan(x):
        return "--"
    return f"{float(x):.{digits}f}"

class Rs485Backend(Node):
    """ROS backend for the robot TCP hardware node and joint_states snapshot."""

    def __init__(self):
        super().__init__("robot_hw_gui_backend")

        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("axis_ids", [0, 1, 2, 3, 4, 5])
        self.declare_parameter("robot_ip", "192.168.2.50")
        self.declare_parameter("ping_timeout_ms", 1000)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.axis_ids: List[int] = [int(x) for x in self.get_parameter("axis_ids").value]

        self._lock = threading.Lock()
        self.connected: bool = False
        self.status_text: str = ""
        self.axis_status: Dict[int, AxisStatus] = {}
        self.joints: Dict[str, JointSnapshot] = {}

        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        self.create_subscription(Bool, "/hardware/connected", self._on_connected, 10)
        # Legacy topics are kept so the GUI still works with older robot_hw_node builds.
        self.create_subscription(Bool, "/robot_hw/connected", self._on_connected, 10)
        self.create_subscription(String, "/robot_hw/status_text", self._on_text, 10)
        if FlagStatus is not None:
            self.create_subscription(FlagStatus, "/hardware/flags", self._on_flags, 10)

        self.cli_poll_now = self.create_client(Trigger, "/robot_hw/poll_now")

        self.cli_servo_on_axis = self.create_client(ServoOnAxis, "/robot_hw/servo_on_axis") if ServoOnAxis else None
        self.cli_servo_on_all = self.create_client(ServoOnAll, "/robot_hw/servo_on_all") if ServoOnAll else None
        self.cli_jog = self.create_client(Jog, "/robot_hw/jog") if Jog else None
        self.cli_run_axis = self.create_client(RunAxis, "/robot_hw/run_axis") if RunAxis else None
        self.cli_home = self.create_client(Home, "/robot_hw/home") if Home else None
        self.cli_stop_axis = self.create_client(StopAxis, "/robot_hw/stop_axis") if StopAxis else None
        self.cli_stop_all = self.create_client(StopAll, "/robot_hw/stop_all") if StopAll else None

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

    def _on_flags(self, msg) -> None:
        with self._lock:
            self.axis_status.clear()
            for idx, axis in enumerate(msg.axes):
                axis_id = self.axis_ids[idx] if idx < len(self.axis_ids) else idx
                self.axis_status[int(axis_id)] = AxisStatus(
                    servo_on=bool(axis.servo_on),
                    error_all=bool(axis.error_all),
                    org_ok=bool(axis.org_ok),
                    motionning=bool(axis.motionning),
                    org_retunning=bool(axis.org_retunning),
                    limit_pos=bool(axis.limit_pos),
                    limit_neg=bool(axis.limit_neg),
                    org_sensor=bool(axis.org_sensor),
                    alarm_rst=bool(axis.alarm_rst),
                    emg=bool(axis.emg),
                    stop=bool(axis.stop),
                    communi_err=bool(axis.communi_err),
                    status_f=int(axis.status_f),
                )

    def snapshot(self) -> Tuple[bool, str, Dict[str, JointSnapshot], Dict[int, AxisStatus]]:
        with self._lock:
            return (
                self.connected,
                str(self.status_text),
                dict(self.joints),
                dict(self.axis_status),
            )

    def _wait_service(self, cli, timeout_s: float) -> bool:
        if cli is None:
            return False
        if cli.service_is_ready():
            return True
        return bool(cli.wait_for_service(timeout_sec=timeout_s))

    def call_trigger(self, cli, timeout_s: float = 1.5):
        if not self._wait_service(cli, timeout_s):
            return (False, 1, "service not available")
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while rclpy.ok() and (not fut.done()) and (time.time() - t0) < timeout_s:
            time.sleep(0.01)
        if (not fut.done()) or (fut.result() is None):
            return (False, 2, "no response (timeout)")
        res = fut.result()
        return (bool(res.success), 0 if res.success else 3, str(res.message))

    def ping_robot(self):
        ip = str(self.get_parameter("robot_ip").value).strip()
        timeout_ms = int(self.get_parameter("ping_timeout_ms").value)
        timeout_ms = max(100, min(timeout_ms, 10000))
        timeout_s = max(1, math.ceil(timeout_ms / 1000.0))

        if not ip:
            return (False, 10, "robot_ip param is empty")
        if shutil.which("ping") is None:
            return (False, 11, "ping command not found")

        cmd = ["ping", "-n", "-c", "1", "-W", str(timeout_s), ip]
        started = time.monotonic()
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_s + 1.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            elapsed_ms = (time.monotonic() - started) * 1000.0
            return (False, 2, f"Ping {ip} timeout after {elapsed_ms:.0f} ms")

        output = "\n".join(part for part in (proc.stdout, proc.stderr) if part)
        time_match = re.search(r"time[=<]([0-9.]+)\s*ms", output)

        if proc.returncode == 0:
            if time_match:
                return (True, 0, f"Ping {ip} OK: {time_match.group(1)} ms")
            elapsed_ms = (time.monotonic() - started) * 1000.0
            return (True, 0, f"Ping {ip} OK: {elapsed_ms:.0f} ms")

        summary = output.strip().splitlines()[-1] if output.strip() else "no reply"
        return (False, proc.returncode, f"Ping {ip} failed: {summary}")

    def servo_on_axis(self, axis_id: int, state: int, timeout_s: float = 1.5):
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

    def servo_on_all(self, state: int, timeout_s: float = 1.5):
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

    def jog(self, axis_id: int, vel_rad_s: float, direction01: int, timeout_s: float = 1.5):
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

    def run_axis(self, axis_id: int, pos_rad: float, vel_rad_s: float, timeout_s: float = 1.5):
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

    def home(self, axis_id: int, timeout_s: float = 2.0):
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

    def stop_axis(self, axis_id: int, timeout_s: float = 1.5):
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

    def stop_all(self, timeout_s: float = 1.5):
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
