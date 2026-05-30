#!/usr/bin/env python3
"""Read-only environment checker for ROS 2 Jazzy + YOLO + RL."""

from __future__ import annotations

import importlib.util
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


def has_module(module: str) -> bool:
    return importlib.util.find_spec(module) is not None


def command_output(args: list[str]) -> tuple[bool, str]:
    exe = shutil.which(args[0])
    if exe is None:
        return False, "not found"
    try:
        completed = subprocess.run(
            [exe, *args[1:]],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=8,
        )
    except Exception as exc:  # pragma: no cover - diagnostic path
        return False, str(exc)
    text = completed.stdout.strip().splitlines()
    return completed.returncode == 0, text[0] if text else "available"


def module_check(module: str, label: str | None = None) -> Check:
    return Check(label or module, has_module(module), "importable" if has_module(module) else "missing")


def torch_check() -> Check:
    if not has_module("torch"):
        return Check("torch", False, "missing")
    import torch  # type: ignore

    version = getattr(torch, "__version__", "unknown")
    cuda = torch.cuda.is_available()
    cuda_version = getattr(torch.version, "cuda", None)
    return Check("torch", True, f"{version}, cuda_available={cuda}, cuda={cuda_version}")


def main() -> int:
    print("ROS 2 Jazzy / YOLO / RL environment check")
    print(f"Python     : {sys.version.split()[0]} ({sys.executable})")
    print(f"Platform   : {platform.platform()}")
    print(f"ROS_DISTRO : {os.environ.get('ROS_DISTRO', 'unset')}")
    print(f"VIRTUAL_ENV: {os.environ.get('VIRTUAL_ENV', 'unset')}")
    print()

    checks: list[Check] = [
        module_check("rclpy"),
        module_check("cv_bridge"),
        module_check("sensor_msgs"),
        module_check("tf2_ros"),
        module_check("cv2", "opencv-python/cv2"),
        module_check("numpy"),
        module_check("scipy"),
        module_check("ultralytics"),
        module_check("onnxruntime"),
        module_check("gymnasium"),
        module_check("stable_baselines3"),
        torch_check(),
    ]

    ros_ok, ros_detail = command_output(["ros2", "doctor", "--report"])
    checks.append(Check("ros2 doctor --report", ros_ok, ros_detail))

    colcon_ok, colcon_detail = command_output(["colcon", "list"])
    checks.append(Check("colcon list", colcon_ok, colcon_detail))

    width = max(len(check.name) for check in checks)
    for check in checks:
        status = "OK" if check.ok else "MISS"
        print(f"[{status}] {check.name:<{width}}  {check.detail}")

    missing = [check.name for check in checks if not check.ok]
    print()
    if missing:
        print("Missing/failed checks:", ", ".join(missing))
        return 1

    print("All checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
