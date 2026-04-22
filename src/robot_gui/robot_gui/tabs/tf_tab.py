from qtpy import QtCore, QtWidgets
from robot_gui.backends.pee_backend import PeeBackend

class TfTab(QtWidgets.QWidget):
    def __init__(self, backend: PeeBackend, parent=None):
        super().__init__(parent)
        self.backend = backend

        layout = QtWidgets.QVBoxLayout(self)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_copy_to_target = QtWidgets.QPushButton("Nạp pose hiện tại vào target")
        btn_row.addWidget(self.btn_copy_to_target)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        out_box = QtWidgets.QGroupBox("")
        out_grid = QtWidgets.QGridLayout(out_box)

        self.out_x = QtWidgets.QLineEdit(); self.out_x.setReadOnly(True)
        self.out_y = QtWidgets.QLineEdit(); self.out_y.setReadOnly(True)
        self.out_z = QtWidgets.QLineEdit(); self.out_z.setReadOnly(True)
        self.out_r = QtWidgets.QLineEdit(); self.out_r.setReadOnly(True)
        self.out_p = QtWidgets.QLineEdit(); self.out_p.setReadOnly(True)
        self.out_yaw = QtWidgets.QLineEdit(); self.out_yaw.setReadOnly(True)

        out_grid.addWidget(QtWidgets.QLabel("X [m]"), 0, 0); out_grid.addWidget(self.out_x, 0, 1)
        out_grid.addWidget(QtWidgets.QLabel("Y [m]"), 1, 0); out_grid.addWidget(self.out_y, 1, 1)
        out_grid.addWidget(QtWidgets.QLabel("Z [m]"), 2, 0); out_grid.addWidget(self.out_z, 2, 1)
        out_grid.addWidget(QtWidgets.QLabel("Roll [deg]"), 0, 2); out_grid.addWidget(self.out_r, 0, 3)
        out_grid.addWidget(QtWidgets.QLabel("Pitch [deg]"), 1, 2); out_grid.addWidget(self.out_p, 1, 3)
        out_grid.addWidget(QtWidgets.QLabel("Yaw [deg]"), 2, 2); out_grid.addWidget(self.out_yaw, 2, 3)

        layout.addWidget(out_box)

        self.tf_status = QtWidgets.QLabel("Trạng thái TF: idle")
        layout.addWidget(self.tf_status)
        layout.addStretch()

        self._pose_timer = QtCore.QTimer(self)
        self._pose_timer.timeout.connect(self.update_live_pose)
        self._pose_timer.start(500)

    def update_live_pose(self) -> None:
        ok, pose, msg = self.backend.lookup_fixed_pose()
        self.tf_status.setText(f"Trạng thái TF: {msg}")
        if not ok:
            return
        self.out_x.setText(f"{pose['x']:.4f}")
        self.out_y.setText(f"{pose['y']:.4f}")
        self.out_z.setText(f"{pose['z']:.4f}")
        self.out_r.setText(f"{pose['roll']:.2f}")
        self.out_p.setText(f"{pose['pitch']:.2f}")
        self.out_yaw.setText(f"{pose['yaw']:.2f}")
