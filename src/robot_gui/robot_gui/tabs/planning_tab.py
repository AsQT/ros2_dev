import threading
from typing import Dict, List

from qtpy import QtCore, QtWidgets
from robot_gui.backends.pee_backend import PeeBackend

class PlanningTab(QtWidgets.QWidget):
    worker_finished = QtCore.Signal(bool, str)

    def __init__(self, backend: PeeBackend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self._planner_options: Dict[str, List[str]] = {}
        self._busy = False

        layout = QtWidgets.QVBoxLayout(self)

        setup_box = QtWidgets.QGroupBox("Thiết lập planner")
        setup_form = QtWidgets.QFormLayout(setup_box)

        self.group_edit = QtWidgets.QLineEdit(self.backend.default_group_name())
        self.pipeline_combo = QtWidgets.QComboBox()
        self.planner_combo = QtWidgets.QComboBox()
        self.btn_refresh_planners = QtWidgets.QPushButton("Refresh planner list")

        setup_form.addRow("Pipeline ID:", self.pipeline_combo)
        setup_form.addRow("Planner ID:", self.planner_combo)
        setup_form.addRow("", self.btn_refresh_planners)
        layout.addWidget(setup_box)

        pose_box = QtWidgets.QGroupBox("Nhập pose đích của P_EE")
        pose_grid = QtWidgets.QGridLayout(pose_box)

        self.in_x = self._make_spin(-10.0, 10.0, 0.001, 4, 0.30)
        self.in_y = self._make_spin(-10.0, 10.0, 0.001, 4, 0.00)
        self.in_z = self._make_spin(-10.0, 10.0, 0.001, 4, 0.30)
        self.in_r = self._make_spin(-360.0, 360.0, 1.0, 2, 180.0)
        self.in_p = self._make_spin(-360.0, 360.0, 1.0, 2, 0.0)
        self.in_yaw = self._make_spin(-360.0, 360.0, 1.0, 2, 0.0)

        pose_grid.addWidget(QtWidgets.QLabel("X [m]"), 0, 0); pose_grid.addWidget(self.in_x, 0, 1)
        pose_grid.addWidget(QtWidgets.QLabel("Y [m]"), 1, 0); pose_grid.addWidget(self.in_y, 1, 1)
        pose_grid.addWidget(QtWidgets.QLabel("Z [m]"), 2, 0); pose_grid.addWidget(self.in_z, 2, 1)
        pose_grid.addWidget(QtWidgets.QLabel("Roll [deg]"), 0, 2); pose_grid.addWidget(self.in_r, 0, 3)
        pose_grid.addWidget(QtWidgets.QLabel("Pitch [deg]"), 1, 2); pose_grid.addWidget(self.in_p, 1, 3)
        pose_grid.addWidget(QtWidgets.QLabel("Yaw [deg]"), 2, 2); pose_grid.addWidget(self.in_yaw, 2, 3)
        layout.addWidget(pose_box)

        tol_box = QtWidgets.QGroupBox("Tùy chọn")
        tol_grid = QtWidgets.QGridLayout(tol_box)

        self.pos_tol = self._make_spin(0.0001, 1.0, 0.001, 4, self.backend.default_pos_tol())
        self.ori_tol = self._make_spin(0.01, 180.0, 0.1, 2, self.backend.default_ori_tol_deg())
        self.vel_scale = self._make_spin(0.01, 1.0, 0.05, 2, self.backend.default_vel_scale())
        self.acc_scale = self._make_spin(0.01, 1.0, 0.05, 2, self.backend.default_acc_scale())

        tol_grid.addWidget(QtWidgets.QLabel("Pos tol [m]"), 0, 0); tol_grid.addWidget(self.pos_tol, 0, 1)
        tol_grid.addWidget(QtWidgets.QLabel("Ori tol [deg]"), 0, 2); tol_grid.addWidget(self.ori_tol, 0, 3)
        tol_grid.addWidget(QtWidgets.QLabel("Vel scale"), 1, 0); tol_grid.addWidget(self.vel_scale, 1, 1)
        tol_grid.addWidget(QtWidgets.QLabel("Acc scale"), 1, 2); tol_grid.addWidget(self.acc_scale, 1, 3)
        layout.addWidget(tol_box)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_plan = QtWidgets.QPushButton("Plan")
        self.btn_execute = QtWidgets.QPushButton("Execute")
        btn_row.addWidget(self.btn_plan)
        btn_row.addWidget(self.btn_execute)
        layout.addLayout(btn_row)

        self.log_box = QtWidgets.QTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box)

        self.worker_finished.connect(self._finish_worker)
        self.pipeline_combo.currentIndexChanged.connect(self._on_pipeline_changed)
        self.btn_refresh_planners.clicked.connect(self.refresh_planner_options)
        self.btn_plan.clicked.connect(self.on_plan_clicked)
        self.btn_execute.clicked.connect(self.on_execute_clicked)

        self.refresh_planner_options()

    def _make_spin(self, min_v: float, max_v: float, step: float, decimals: int, value: float) -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(min_v, max_v)
        box.setDecimals(decimals)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def append_log(self, text: str) -> None:
        self.log_box.append(text)

    def set_busy(self, busy: bool) -> None:
        self._busy = busy
        self.btn_plan.setEnabled(not busy)
        self.btn_execute.setEnabled(not busy)

    def refresh_planner_options(self) -> None:
        options, msg = self.backend.get_planner_options()
        self._planner_options = options

        self.pipeline_combo.blockSignals(True)
        self.pipeline_combo.clear()
        self.pipeline_combo.addItem("(default)", "")
        for pipeline_id in sorted(self._planner_options.keys()):
            self.pipeline_combo.addItem(pipeline_id, pipeline_id)
        self.pipeline_combo.blockSignals(False)

        self._on_pipeline_changed()
        self.append_log(msg)

    def _on_pipeline_changed(self) -> None:
        pipeline_id = self.pipeline_combo.currentData() or ""
        planners = self._planner_options.get(str(pipeline_id), [])

        self.planner_combo.blockSignals(True)
        self.planner_combo.clear()
        self.planner_combo.addItem("(default)", "")
        for planner_id in planners:
            self.planner_combo.addItem(planner_id, planner_id)
        self.planner_combo.blockSignals(False)

    def on_plan_clicked(self) -> None:
        if self._busy:
            return
        self.set_busy(True)
        self.append_log("Đang planning...")
        threading.Thread(target=self._plan_worker, daemon=True).start()

    def _plan_worker(self) -> None:
        pipeline_id = self.pipeline_combo.currentData() or ""
        planner_id = self.planner_combo.currentData() or ""
        ok, msg = self.backend.plan_to_pose(
            group_name=self.group_edit.text().strip(),
            x=self.in_x.value(),
            y=self.in_y.value(),
            z=self.in_z.value(),
            roll_deg=self.in_r.value(),
            pitch_deg=self.in_p.value(),
            yaw_deg=self.in_yaw.value(),
            pos_tol=self.pos_tol.value(),
            ori_tol_deg=self.ori_tol.value(),
            vel_scale=self.vel_scale.value(),
            acc_scale=self.acc_scale.value(),
            pipeline_id=str(pipeline_id),
            planner_id=str(planner_id),
        )
        self.worker_finished.emit(ok, msg)

    def on_execute_clicked(self) -> None:
        if self._busy:
            return
        self.set_busy(True)
        self.append_log("Đang execute...")
        threading.Thread(target=self._execute_worker, daemon=True).start()

    def _execute_worker(self) -> None:
        ok, msg = self.backend.execute_last_plan()
        self.worker_finished.emit(ok, msg)

    def _finish_worker(self, ok: bool, msg: str) -> None:
        self.append_log(("[OK] " if ok else "[ERR] ") + msg)
        self.set_busy(False)
