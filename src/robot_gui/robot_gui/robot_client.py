import socket
import time
from typing import Optional


class RobotClientError(RuntimeError):
    """Raised when a robot TCP command cannot be completed."""


class RobotClient:
    """Small TCP abstraction used by the GUI.

    The command strings are intentionally isolated here. If the robot firmware
    uses a binary or framed protocol, update this class without touching Qt UI
    code.
    """

    def __init__(self):
        self._sock: Optional[socket.socket] = None
        self.ip = "192.168.2.50"
        self.port = 5000
        self.timeout_ms = 2

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect_robot(self, ip: str, port: int, timeout_ms: int = 2) -> None:
        self.disconnect_robot()
        self.ip = ip
        self.port = int(port)
        self.timeout_ms = int(timeout_ms)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(max(self.timeout_ms / 1000.0, 0.001))
        try:
            sock.connect((self.ip, self.port))
        except OSError as exc:
            sock.close()
            raise RobotClientError(f"Connect failed: {exc}") from exc

        self._sock = sock

    def disconnect_robot(self) -> None:
        if self._sock is None:
            return
        try:
            self._sock.close()
        finally:
            self._sock = None

    def ping(self) -> float:
        start = time.monotonic()
        self._send_line("PING")
        try:
            self._recv_some()
        except RobotClientError:
            pass
        return (time.monotonic() - start) * 1000.0

    def enable_robot(self) -> None:
        self._send_line("ROBOT ENABLE")

    def disable_robot(self) -> None:
        self._send_line("ROBOT DISABLE")

    def emergency_stop(self) -> None:
        self._send_line("ROBOT ESTOP")

    def reset_error(self, joint_id: Optional[int] = None) -> None:
        if joint_id is None:
            self._send_line("ROBOT RESET_ERROR")
        else:
            self._send_line(f"AXIS {joint_id} RESET_ERROR")

    def enable_axis(self, joint_id: int) -> None:
        self._send_line(f"AXIS {joint_id} ENABLE")

    def home_axis(self, joint_id: int) -> None:
        self._send_line(f"AXIS {joint_id} HOME")

    def stop_axis(self, joint_id: int) -> None:
        self._send_line(f"AXIS {joint_id} STOP")

    def send_jog(self, joint_id: int, direction: int, speed_mdeg_s: int = 1000) -> None:
        sign = 1 if direction >= 0 else -1
        self._send_line(f"AXIS {joint_id} JOG {sign} {int(speed_mdeg_s)}")

    def stop_jog(self, joint_id: int) -> None:
        self._send_line(f"AXIS {joint_id} JOG_STOP")

    def send_abs_position(self, joint_id: int, position_mdeg: int, velocity_mdeg_s: int = 1000) -> None:
        self._send_line(f"AXIS {joint_id} RUN_ABS {int(position_mdeg)} {int(velocity_mdeg_s)}")

    def gripper_open(self) -> None:
        self._send_line("GRIPPER OPEN")

    def gripper_close(self) -> None:
        self._send_line("GRIPPER CLOSE")

    def set_gripper_width(self, width: float) -> None:
        self._send_line(f"GRIPPER WIDTH {width:.6f}")

    def _send_line(self, command: str) -> None:
        if self._sock is None:
            raise RobotClientError("Robot is not connected")
        try:
            self._sock.sendall((command + "\n").encode("ascii"))
        except OSError as exc:
            self.disconnect_robot()
            raise RobotClientError(f"TCP send failed: {exc}") from exc

    def _recv_some(self) -> bytes:
        if self._sock is None:
            raise RobotClientError("Robot is not connected")
        try:
            return self._sock.recv(1024)
        except socket.timeout:
            return b""
        except OSError as exc:
            self.disconnect_robot()
            raise RobotClientError(f"TCP receive failed: {exc}") from exc
