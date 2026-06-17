import time
import math
from typing import List, Optional

from qtpy import QtCore, QtGui, QtWidgets

from robot_gui.backends.rs485_backend import AxisStatus, Rs485Backend, rad_to_deg, fmt
from robot_gui.utils.workers import CallWorker


class HoldButton(QtWidgets.QPushButton):
    """Button cho Jog: nhấn giữ -> start, nhả -> stop."""

    pressed_hold = QtCore.Signal()
    released_hold = QtCore.Signal()

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        super().mousePressEvent(e)
        if e.button() == QtCore.Qt.LeftButton:
            self.pressed_hold.emit()

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent) -> None:
        super().mouseReleaseEvent(e)
        if e.button() == QtCore.Qt.LeftButton:
            self.released_hold.emit()


class AxisPanel(QtWidgets.QGroupBox):
    def __init__(self, axis_id: int, joint_name: str, parent=None):
        super().__init__(parent)

        self.axis_id = axis_id
        self.joint_name = joint_name

        self.setTitle(f"Joint {axis_id}")

        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(10, 10, 10, 10)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(6)

        # ============================================================
        # DISPLAY FEEDBACK
        # GUI hiển thị theo độ
        # Backend vẫn dùng radian
        # ============================================================

        self.lb_pos_deg = QtWidgets.QLabel("-- deg")
        self.lb_pos_deg.setStyleSheet(
            "font-size: 16pt; font-weight: 700; color: #0B1F35;"
        )

        self.lb_pos_rad = QtWidgets.QLabel("pos: -- deg")
        self.lb_pos_rad.setStyleSheet(
            "font-size: 11pt; color: #244B6B;"
        )

        self.lb_vel = QtWidgets.QLabel("vel: -- deg/s")
        self.lb_vel.setStyleSheet(
            "font-size: 11pt; color: #244B6B;"
        )

        self.lb_age = QtWidgets.QLabel("age: -- ms")
        self.lb_age.setStyleSheet(
            "font-size: 10pt; color: #6B7C8A;"
        )

        self.lb_flags = QtWidgets.QLabel("flags: --")
        self.lb_flags.setStyleSheet(
            "font-size: 10pt; color: #263238;"
        )
        self.lb_flags.setWordWrap(True)

        g.addWidget(self.lb_pos_deg, 0, 0, 1, 2)
        g.addWidget(self.lb_pos_rad, 0, 2, 1, 1)
        g.addWidget(self.lb_vel, 0, 3, 1, 1)
        g.addWidget(self.lb_age, 0, 4, 1, 1)
        g.addWidget(self.lb_flags, 1, 0, 1, 5)

        # ============================================================
        # TARGET POSITION
        # Người dùng nhập độ
        # Khi chạy sẽ đổi sang radian ở _run_axis()
        # ============================================================

        self.ed_pos = QtWidgets.QDoubleSpinBox()
        self.ed_pos.setRange(math.degrees(-1000.0), math.degrees(1000.0))
        self.ed_pos.setDecimals(3)
        self.ed_pos.setSingleStep(1.0)
        self.ed_pos.setValue(0.0)
        self.ed_pos.setSuffix(" deg")

        # ============================================================
        # TARGET VELOCITY
        # Người dùng nhập deg/s
        # Khi chạy sẽ đổi sang rad/s ở _run_axis()
        # 0.5 rad/s = 28.648 deg/s
        # ============================================================

        self.ed_vel = QtWidgets.QDoubleSpinBox()
        self.ed_vel.setRange(0.0, math.degrees(1000.0))
        self.ed_vel.setDecimals(3)
        self.ed_vel.setSingleStep(1.0)
        self.ed_vel.setValue(math.degrees(0.5))
        self.ed_vel.setSuffix(" deg/s")

        g.addWidget(QtWidgets.QLabel("Target pos"), 2, 0)
        g.addWidget(self.ed_pos, 2, 1)
        g.addWidget(QtWidgets.QLabel("Vel"), 2, 2)
        g.addWidget(self.ed_vel, 2, 3)

        self.btn_servo_on = QtWidgets.QPushButton("Servo ON")
        self.btn_servo_off = QtWidgets.QPushButton("Servo OFF")
        self.btn_home = QtWidgets.QPushButton("Home")
        self.btn_run = QtWidgets.QPushButton("Run")
        self.btn_stop = QtWidgets.QPushButton("Stop")

        self.btn_run.setStyleSheet("font-weight: 700;")
        self.btn_stop.setStyleSheet(
            "font-weight: 700; color: white; background: #C62828;"
        )

        g.addWidget(self.btn_servo_on, 3, 0)
        g.addWidget(self.btn_servo_off, 3, 1)
        g.addWidget(self.btn_home, 3, 2)
        g.addWidget(self.btn_run, 2, 4)
        g.addWidget(self.btn_stop, 3, 4)

        # ============================================================
        # JOG VELOCITY
        # Người dùng nhập deg/s
        # Khi jog sẽ đổi sang rad/s ở _jog_start()
        # 0.3 rad/s = 17.189 deg/s
        # ============================================================

        self.ed_jog_vel = QtWidgets.QDoubleSpinBox()
        self.ed_jog_vel.setRange(0.0, math.degrees(1000.0))
        self.ed_jog_vel.setDecimals(3)
        self.ed_jog_vel.setSingleStep(1.0)
        self.ed_jog_vel.setValue(math.degrees(0.3))
        self.ed_jog_vel.setSuffix(" deg/s")

        self.btn_jog_neg = HoldButton("◀ Jog-")
        self.btn_jog_pos = HoldButton("Jog+ ▶")

        self.btn_jog_neg.setAutoRepeat(False)
        self.btn_jog_pos.setAutoRepeat(False)

        g.addWidget(QtWidgets.QLabel("Jog vel"), 4, 0)
        g.addWidget(self.ed_jog_vel, 4, 1)
        g.addWidget(self.btn_jog_neg, 4, 2)
        g.addWidget(self.btn_jog_pos, 4, 3)

    def set_axis_status(self, status: Optional[AxisStatus]) -> None:
        if status is None:
            self.lb_flags.setText("flags: --")
            self.lb_flags.setStyleSheet("font-size: 10pt; color: #607D8B;")
            return

        normal = []
        warning = []
        normal.append("Servo ON" if status.servo_on else "Servo OFF")
        if status.org_ok:
            normal.append("Home OK")
        if status.motionning:
            normal.append("Moving")
        if status.stop:
            normal.append("Stop")
        if status.error_all:
            warning.append("ERR")
        if status.limit_pos:
            warning.append("LIM+")
        if status.limit_neg:
            warning.append("LIM-")
        if status.emg:
            warning.append("EMG")
        if status.communi_err:
            warning.append("COMM")

        parts = warning + normal
        self.lb_flags.setText(
            f"flags: {', '.join(parts) if parts else 'OK'} | raw=0x{status.status_f:08X}"
        )
        color = "#B71C1C" if warning else "#1B5E20"
        self.lb_flags.setStyleSheet(f"font-size: 10pt; color: {color};")


class HardwareTab(QtWidgets.QWidget):
    def __init__(self, backend: Rs485Backend, parent=None):
        super().__init__(parent)

        self.backend = backend
        self.pool = QtCore.QThreadPool.globalInstance()

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(10, 10, 10, 10)
        v.setSpacing(10)

        # ============================================================
        # TOP BAR
        # ============================================================

        top = QtWidgets.QHBoxLayout()

        self.lb_conn = QtWidgets.QLabel("DISCONNECTED")
        self.lb_conn.setStyleSheet(
            "font-size: 12pt; "
            "font-weight: 800; "
            "color: white; "
            "background:#455A64; "
            "padding:6px 10px; "
            "border-radius:10px;"
        )

        self.lb_status = QtWidgets.QLabel("status: --")
        self.lb_status.setStyleSheet(
            "font-size: 11pt; color:#0B1F35;"
        )

        self.btn_ping = QtWidgets.QPushButton("Ping")
        self.btn_poll_now = QtWidgets.QPushButton("Poll Now")
        self.btn_servo_all_on = QtWidgets.QPushButton("Servo ALL ON")
        self.btn_servo_all_off = QtWidgets.QPushButton("Servo ALL OFF")
        self.btn_stop_all = QtWidgets.QPushButton("STOP ALL")

        self.lb_ping = QtWidgets.QLabel("ping: --")
        self.lb_ping.setStyleSheet("font-size: 10pt; color:#455A64;")

        self.btn_stop_all.setStyleSheet(
            "font-weight: 900; "
            "color: white; "
            "background:#B71C1C; "
            "padding:6px 14px;"
        )

        top.addWidget(self.lb_conn)
        top.addSpacing(10)
        top.addWidget(self.lb_status, 1)
        top.addWidget(self.btn_ping)
        top.addWidget(self.lb_ping)
        top.addWidget(self.btn_poll_now)
        top.addWidget(self.btn_servo_all_on)
        top.addWidget(self.btn_servo_all_off)
        top.addWidget(self.btn_stop_all)

        v.addLayout(top)

        # ============================================================
        # JOINT PANELS
        # ============================================================

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        v.addLayout(grid, 1)

        self.axis_panels: List[AxisPanel] = []

        n = min(6, len(self.backend.axis_ids), len(self.backend.joint_names))

        for i in range(n):
            axis_id = self.backend.axis_ids[i]
            joint = self.backend.joint_names[i]

            p = AxisPanel(axis_id, joint)
            self.axis_panels.append(p)

            r = i // 2
            c = i % 2
            grid.addWidget(p, r, c)

            p.btn_servo_on.clicked.connect(
                lambda _=False, a=axis_id: self._call(
                    self.backend.servo_on_axis, a, 1
                )
            )

            p.btn_servo_off.clicked.connect(
                lambda _=False, a=axis_id: self._call(
                    self.backend.servo_on_axis, a, 0
                )
            )

            p.btn_home.clicked.connect(
                lambda _=False, a=axis_id: self._call(
                    self.backend.home, a
                )
            )

            p.btn_run.clicked.connect(
                lambda _=False, pp=p: self._run_axis(pp)
            )

            p.btn_stop.clicked.connect(
                lambda _=False, a=axis_id: self._call(
                    self.backend.stop_axis, a
                )
            )

            p.btn_jog_neg.pressed_hold.connect(
                lambda a=axis_id, pp=p: self._jog_start(pp, a, 0)
            )

            p.btn_jog_pos.pressed_hold.connect(
                lambda a=axis_id, pp=p: self._jog_start(pp, a, 1)
            )

            p.btn_jog_neg.released_hold.connect(
                lambda a=axis_id: self._call(
                    self.backend.stop_axis, a
                )
            )

            p.btn_jog_pos.released_hold.connect(
                lambda a=axis_id: self._call(
                    self.backend.stop_axis, a
                )
            )

        # ============================================================
        # LOG WINDOW
        # ============================================================

        self.txt_log = QtWidgets.QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(500)
        v.addWidget(self.txt_log, 0)

        # ============================================================
        # REFRESH TIMER
        # ============================================================

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)

        # ============================================================
        # TOP BUTTON SIGNALS
        # ============================================================

        self.btn_ping.clicked.connect(self._ping_robot)

        self.btn_poll_now.clicked.connect(
            lambda: self._call(
                self.backend.call_trigger,
                self.backend.cli_poll_now
            )
        )

        self.btn_servo_all_on.clicked.connect(
            lambda: self._call(
                self.backend.servo_on_all, 1
            )
        )

        self.btn_servo_all_off.clicked.connect(
            lambda: self._call(
                self.backend.servo_on_all, 0
            )
        )

        self.btn_stop_all.clicked.connect(
            lambda: self._call(
                self.backend.stop_all
            )
        )

    # ============================================================
    # LOG RESULT FROM WORKER
    # ============================================================

    def _log(self, ok: bool, code: int, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        s = f"[{ts}] {'OK' if ok else 'ERR'} (code={code}): {msg}"
        self.txt_log.appendPlainText(s)

    # ============================================================
    # CALL BACKEND IN THREAD
    # ============================================================

    def _call(self, fn, *args, **kwargs) -> None:
        worker = CallWorker(fn, *args, **kwargs)
        worker.signals.done.connect(self._log)
        self.pool.start(worker)

    def _ping_robot(self) -> None:
        self.btn_ping.setEnabled(False)
        self.lb_ping.setText("ping: checking...")
        self.lb_ping.setStyleSheet("font-size: 10pt; color:#1565C0;")

        worker = CallWorker(self.backend.ping_robot)
        worker.signals.done.connect(self._finish_ping)
        self.pool.start(worker)

    def _finish_ping(self, ok: bool, code: int, msg: str) -> None:
        self.btn_ping.setEnabled(True)
        self.lb_ping.setText(f"ping: {'OK' if ok else 'ERR'}")
        color = "#1B5E20" if ok else "#B71C1C"
        self.lb_ping.setStyleSheet(f"font-size: 10pt; color:{color};")
        self._log(ok, code, msg)

    # ============================================================
    # RUN AXIS
    # GUI nhập:
    #   target position: deg
    #   target velocity: deg/s
    #
    # Backend nhận:
    #   target position: rad
    #   target velocity: rad/s
    # ============================================================

    def _run_axis(self, panel: AxisPanel) -> None:
        target_pos_deg = float(panel.ed_pos.value())
        target_vel_deg_s = float(panel.ed_vel.value())

        target_pos_rad = math.radians(target_pos_deg)
        target_vel_rad_s = math.radians(target_vel_deg_s)

        self._call(
            self.backend.run_axis,
            panel.axis_id,
            target_pos_rad,
            target_vel_rad_s
        )

    # ============================================================
    # JOG START
    # GUI nhập:
    #   jog velocity: deg/s
    #
    # Backend nhận:
    #   jog velocity: rad/s
    # ============================================================

    def _jog_start(self, panel: AxisPanel, axis_id: int, dir01: int) -> None:
        jog_vel_deg_s = float(panel.ed_jog_vel.value())
        jog_vel_rad_s = math.radians(jog_vel_deg_s)

        self._call(
            self.backend.jog,
            axis_id,
            jog_vel_rad_s,
            dir01
        )

    # ============================================================
    # REFRESH DISPLAY
    # Backend snapshot trả về rad/rad_s
    # GUI đổi sang deg/deg_s để hiển thị
    # ============================================================

    def _refresh(self) -> None:
        connected, text, joints, axis_status = self.backend.snapshot()

        if connected:
            self.lb_conn.setText("CONNECTED")
            self.lb_conn.setStyleSheet(
                "font-size: 12pt; "
                "font-weight: 800; "
                "color: white; "
                "background:#2E7D32; "
                "padding:6px 10px; "
                "border-radius:10px;"
            )
        else:
            self.lb_conn.setText("DISCONNECTED")
            self.lb_conn.setStyleSheet(
                "font-size: 12pt; "
                "font-weight: 800; "
                "color: white; "
                "background:#455A64; "
                "padding:6px 10px; "
                "border-radius:10px;"
            )

        if text:
            status_text = text
        elif connected:
            status_text = f"TCP online | flags from {len(axis_status)} axes"
        else:
            status_text = "TCP offline"
        self.lb_status.setText(f"status: {status_text}")

        now = time.time()

        for p in self.axis_panels:
            js = joints.get(p.joint_name)

            if js is None:
                p.lb_pos_rad.setText("pos: -- deg")
                p.lb_pos_deg.setText("-- deg")
                p.lb_vel.setText("vel: -- deg/s")
                p.lb_age.setText("age: -- ms")
                p.set_axis_status(axis_status.get(p.axis_id))
                continue

            pos_deg = rad_to_deg(js.pos_rad)
            vel_deg_s = rad_to_deg(js.vel_rad_s)

            p.lb_pos_rad.setText(f"pos: {fmt(pos_deg, 3)} deg")
            p.lb_pos_deg.setText(f"{fmt(pos_deg, 2)} deg")
            p.lb_vel.setText(f"vel: {fmt(vel_deg_s, 3)} deg/s")
            p.lb_age.setText(f"age: {(now - js.wall_time) * 1000.0:.0f} ms")
            p.set_axis_status(axis_status.get(p.axis_id))
