import math
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
except Exception as e:  # pragma: no cover
    ServoOnAxis = ServoOnAll = Jog = RunAxis = Home = StopAxis = StopAll = None
    _IMPORT_ERR = str(e)

@dataclass
class JointSnapshot:
    pos_rad: float = float("nan")
    vel_rad_s: float = float("nan")
    stamp_ros: float = 0.0
    wall_time: float = 0.0

def rad_to_deg(x: float) -> float:
    return float(x) * 180.0 / math.pi

def fmt(x: float, digits: int = 3) -> str:
    if x is None or math.isnan(x):
        return "--"
    return f"{float(x):.{digits}f}"

class Rs485Backend(Node):
    """ROS backend for RS-485 hardware services and joint_states snapshot."""

    def __init__(self):
        super().__init__("rs485_hw_gui_backend")

        self.declare_parameter("joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("axis_ids", [0, 1, 2, 3, 4, 5])

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.axis_ids: List[int] = [int(x) for x in self.get_parameter("axis_ids").value]

        self._lock = threading.Lock()
        self.connected: bool = False
        self.status_text: str = ""
        self.joints: Dict[str, JointSnapshot] = {}

        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        self.create_subscription(Bool, "/rs485_hw/connected", self._on_connected, 10)
        self.create_subscription(String, "/rs485_hw/status_text", self._on_text, 10)

        self.cli_connect = self.create_client(Trigger, "/rs485_hw/connect")
        self.cli_disconnect = self.create_client(Trigger, "/rs485_hw/disconnect")

        self.cli_servo_on_axis = self.create_client(ServoOnAxis, "/rs485_hw/servo_on_axis") if ServoOnAxis else None
        self.cli_servo_on_all = self.create_client(ServoOnAll, "/rs485_hw/servo_on_all") if ServoOnAll else None
        self.cli_jog = self.create_client(Jog, "/rs485_hw/jog") if Jog else None
        self.cli_run_axis = self.create_client(RunAxis, "/rs485_hw/run_axis") if RunAxis else None
        self.cli_home = self.create_client(Home, "/rs485_hw/home") if Home else None
        self.cli_stop_axis = self.create_client(StopAxis, "/rs485_hw/stop_axis") if StopAxis else None
        self.cli_stop_all = self.create_client(StopAll, "/rs485_hw/stop_all") if StopAll else None

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

    def snapshot(self) -> Tuple[bool, str, Dict[str, JointSnapshot]]:
        with self._lock:
            return (self.connected, str(self.status_text), dict(self.joints))

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
