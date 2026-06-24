#!/usr/bin/env python3
import argparse
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from robot_hardware_interface.msg import FlagStatus


SERVO_ON_MASK = 0x00100000


class ServoAllFlagsTester(Node):
    def __init__(self, samples: int, timeout: float):
        super().__init__("servo_all_flags_tester")
        self.samples = max(1, int(samples))
        self.timeout = max(0.1, float(timeout))
        self.last_msg: Optional[FlagStatus] = None
        self.subscription = self.create_subscription(
            FlagStatus,
            "/robot_hw/flags",
            self._on_flags,
            10,
        )
        self.servo_client = self.create_client(SetBool, "/robot_hw/servo_all")

    def _on_flags(self, msg: FlagStatus) -> None:
        self.last_msg = msg

    def wait_for_flags(self, label: str) -> Optional[FlagStatus]:
        self.last_msg = None
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_msg is not None:
                break
        if self.last_msg is None:
            print(f"{label}: no /robot_hw/flags message within {self.timeout:.2f}s")
        return self.last_msg

    def collect_flags(self, label: str) -> List[FlagStatus]:
        messages: List[FlagStatus] = []
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline and len(messages) < self.samples:
            self.last_msg = None
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_msg is not None:
                messages.append(self.last_msg)
        if not messages:
            print(f"{label}: no /robot_hw/flags samples within {self.timeout:.2f}s")
        else:
            print(f"{label}: collected {len(messages)} /robot_hw/flags sample(s)")
        return messages

    def call_servo_all(self, enabled: bool) -> Tuple[bool, str]:
        if not self.servo_client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f"/robot_hw/servo_all service not available within {self.timeout:.2f}s")

        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.servo_client.call_async(request)
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"/robot_hw/servo_all data={enabled} timed out after {self.timeout:.2f}s")

        response = future.result()
        if response is None:
            raise RuntimeError(f"/robot_hw/servo_all data={enabled} returned no response")
        return bool(response.success), str(response.message)


def format_msg(msg: Optional[FlagStatus]) -> str:
    if msg is None:
        return "  <no message>"
    lines = []
    for index, axis in enumerate(msg.axes, start=1):
        status = int(axis.status_f)
        bit = (status & SERVO_ON_MASK) != 0
        lines.append(
            f"  Axis {index}: status_f=0x{status:08X}, "
            f"servo_on={bool(axis.servo_on)}, servo_on_bit={bit}"
        )
    return "\n".join(lines)


def print_latest(label: str, messages: List[FlagStatus]) -> None:
    print(label)
    print(format_msg(messages[-1] if messages else None))


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Test /robot_hw/servo_all against /robot_hw/flags")
    parser.add_argument("--samples", type=int, default=5, help="number of flag samples after ON/OFF")
    parser.add_argument("--timeout", type=float, default=3.0, help="timeout in seconds for service/topic waits")
    parser.add_argument("--settle", type=float, default=0.5, help="seconds to wait after service calls")
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = ServoAllFlagsTester(samples=args.samples, timeout=args.timeout)
    on_result: Optional[Tuple[bool, str]] = None
    off_result: Optional[Tuple[bool, str]] = None
    exit_code = 0

    try:
        before = node.collect_flags("before servo_all ON")
        print_latest("before_flags:", before)

        print("Calling /robot_hw/servo_all data=True")
        on_result = node.call_servo_all(True)
        print(f"servo_all ON response: success={on_result[0]}, message={on_result[1]!r}")
        time.sleep(max(0.0, args.settle))
        after_on = node.collect_flags("after servo_all ON")
        print_latest("after_enable_flags:", after_on)

    except Exception as exc:
        exit_code = 1
        print(f"ERROR during ON/read phase: {exc}", file=sys.stderr)
    finally:
        try:
            print("Calling /robot_hw/servo_all data=False")
            off_result = node.call_servo_all(False)
            print(f"servo_all OFF response: success={off_result[0]}, message={off_result[1]!r}")
            time.sleep(max(0.0, args.settle))
            after_off = node.collect_flags("after servo_all OFF")
            print_latest("after_disable_flags:", after_off)
        except Exception as exc:
            exit_code = 1
            print(f"ERROR while forcing servo_all OFF: {exc}", file=sys.stderr)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
