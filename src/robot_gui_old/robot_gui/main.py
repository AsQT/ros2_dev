import sys
from typing import Optional

try:
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication
except ImportError:  # pragma: no cover - convenience for Windows/dev machines.
    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import JointState
except ImportError:  # pragma: no cover - direct GUI test without ROS 2.
    rclpy = None
    Node = object
    JointState = None

try:
    from robot_hardware_interface.msg import FlagStatus
except ImportError:  # pragma: no cover - GUI can still run without hardware msgs.
    FlagStatus = None

from .main_window import RobotMainWindow


class RobotGuiNode(Node):
    def __init__(self):
        super().__init__("robot_gui")
        self.window: Optional[RobotMainWindow] = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.flags_sub = None
        if FlagStatus is not None:
            self.flags_sub = self.create_subscription(
                FlagStatus,
                "/robot_hw/flags",
                self.on_robot_flags_msg,
                10,
            )

    def _on_joint_state(self, msg) -> None:
        if self.window is not None:
            self.window.update_joint_state(msg.name, msg.position, msg.velocity)

    def on_robot_flags_msg(self, msg) -> None:
        if self.window is None:
            return
        flags = [int(getattr(axis_flags, "status_f", 0)) for axis_flags in msg.axes]
        self.window.receive_robot_flags(flags)


def main(args=None) -> int:
    raw_args = list(sys.argv if args is None else args)
    ros_node = None
    qt_args = raw_args
    spin_timer = None

    if rclpy is not None:
        rclpy.init(args=raw_args)
        qt_args = remove_ros_args(args=raw_args)
        ros_node = RobotGuiNode()

    app = QApplication(qt_args)
    window = RobotMainWindow(ros_node=ros_node)
    if ros_node is not None:
        ros_node.window = window

        spin_timer = QTimer()
        spin_timer.timeout.connect(lambda: _spin_once_if_ok(ros_node))
        spin_timer.start(20)

    window.show()

    try:
        result = app.exec_() if hasattr(app, "exec_") else app.exec()
    except KeyboardInterrupt:
        result = 0
    finally:
        if spin_timer is not None:
            spin_timer.stop()
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    return int(result)


def _spin_once_if_ok(ros_node) -> None:
    if rclpy is None or not rclpy.ok():
        return
    try:
        rclpy.spin_once(ros_node, timeout_sec=0.0)
    except KeyboardInterrupt:
        app = QApplication.instance()
        if app is not None:
            app.quit()
    except Exception:
        if rclpy.ok():
            raise


if __name__ == "__main__":
    raise SystemExit(main())
