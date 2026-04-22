from qtpy import QtWidgets

from robot_gui.backends.rs485_backend import Rs485Backend
from robot_gui.backends.pee_backend import PeeBackend
from robot_gui.tabs.hardware_tab import HardwareTab
from robot_gui.tabs.tf_tab import TfTab
from robot_gui.tabs.planning_tab import PlanningTab

class RobotMainWindow(QtWidgets.QMainWindow):
    def __init__(self, rs485_backend: Rs485Backend, pee_backend: PeeBackend):
        super().__init__()
        self.setWindowTitle("Robot GUI (Hardware + TF + Planning)")
        self.resize(1200, 760)

        tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(tabs)

        self.tab_hw = HardwareTab(rs485_backend)
        self.tab_tf = TfTab(pee_backend)
        self.tab_plan = PlanningTab(pee_backend)

        self.tab_tf.btn_copy_to_target.clicked.connect(self._copy_tf_to_target)

        tabs.addTab(self.tab_hw, "Hardware")
        tabs.addTab(self.tab_tf, "TF")
        tabs.addTab(self.tab_plan, "Planning")

    def _copy_tf_to_target(self):
        self.tab_tf.update_live_pose()
        try:
            self.tab_plan.in_x.setValue(float(self.tab_tf.out_x.text()))
            self.tab_plan.in_y.setValue(float(self.tab_tf.out_y.text()))
            self.tab_plan.in_z.setValue(float(self.tab_tf.out_z.text()))
            self.tab_plan.in_r.setValue(float(self.tab_tf.out_r.text()))
            self.tab_plan.in_p.setValue(float(self.tab_tf.out_p.text()))
            self.tab_plan.in_yaw.setValue(float(self.tab_tf.out_yaw.text()))
            self.tab_plan.append_log("Đã nạp live pose (TF) sang target.")
        except ValueError:
            self.tab_plan.append_log("Chưa có TF hợp lệ để nạp sang target.")
