import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from qtpy import QtWidgets

from robot_gui.backends.rs485_backend import Rs485Backend
from robot_gui.backends.pee_backend import PeeBackend
from robot_gui.main_window import RobotMainWindow

def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    rclpy.init(args=argv)

    rs485 = Rs485Backend()
    pee = PeeBackend()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(rs485)
    executor.add_node(pee)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(argv)
    win = RobotMainWindow(rs485, pee)
    win.show()

    try:
        rc = app.exec()
    finally:
        executor.shutdown()
        rs485.destroy_node()
        pee.destroy_node()
        rclpy.shutdown()

    return int(rc)

if __name__ == "__main__":
    raise SystemExit(main())
