from types import SimpleNamespace

from robot_gui.main_window import (
    ALARM,
    EMG,
    ERROR_ALL,
    FAULT_LED_ACTIVE,
    FAULT_LED_NORMAL,
    JOINT_COUNT,
    NORMAL_LED_ACTIVE,
    NORMAL_LED_INACTIVE,
    ORG_SET_OK,
    RUNNING,
    SERVO_ON,
    SOF_LIMIT_M,
    SOF_LIMIT_P,
    RobotMainWindow,
)


class FakeLed:
    def __init__(self):
        self.style = ""

    def setStyleSheet(self, style):
        self.style = style


class FakeButton:
    def __init__(self):
        self._text = ""
        self._style = ""

    def setText(self, text):
        self._text = text

    def text(self):
        return self._text

    def setStyleSheet(self, style):
        self._style = style


class FakeWindow:
    _set_led_state = RobotMainWindow._set_led_state
    _set_normal_led = RobotMainWindow._set_normal_led
    _set_fault_led = RobotMainWindow._set_fault_led
    parse_robot_flags_msg = RobotMainWindow.parse_robot_flags_msg
    on_robot_flags_msg = RobotMainWindow.on_robot_flags_msg
    update_all_axis_leds_from_flags = RobotMainWindow.update_all_axis_leds_from_flags
    update_axis_leds = RobotMainWindow.update_axis_leds
    update_axis_status_leds = RobotMainWindow.update_axis_status_leds
    update_robot_enable_button = RobotMainWindow.update_robot_enable_button

    def __init__(self):
        self._robot_servo_on = False
        self._robot_enable_style = "enable-style"
        self._robot_disable_style = "disable-style"
        self.btnRobotEnable = FakeButton()
        self.logs = []
        self.robot_enable_led = None
        suffixes = (
            "ServoOn",
            "Running",
            "OrgOK",
            "LimitPositive",
            "LimitNegative",
            "Alarm",
            "EMG",
            "ErrorAll",
        )
        for axis in range(1, JOINT_COUNT + 1):
            for suffix in suffixes:
                setattr(self, f"ledAxis{axis}{suffix}", FakeLed())

    def _button(self, name):
        return getattr(self, name, None)

    def _set_status_bar_led(self, name, active):
        if name == "RobotEnableStatus_led":
            self.robot_enable_led = active

    def log_ros2(self, message):
        self.logs.append(message)


def flag_msg(flags):
    return SimpleNamespace(axes=[SimpleNamespace(status_f=status) for status in flags])


def assert_normal_led(window, axis, suffix, active):
    expected = NORMAL_LED_ACTIVE if active else NORMAL_LED_INACTIVE
    assert expected in getattr(window, f"ledAxis{axis}{suffix}").style


def assert_fault_led(window, axis, suffix, active):
    expected = FAULT_LED_ACTIVE if active else FAULT_LED_NORMAL
    assert expected in getattr(window, f"ledAxis{axis}{suffix}").style


def assert_all_axes_inactive(window):
    for axis in range(1, JOINT_COUNT + 1):
        for suffix in ("ServoOn", "Running", "OrgOK", "LimitPositive", "LimitNegative"):
            assert_normal_led(window, axis, suffix, False)
        for suffix in ("Alarm", "EMG", "ErrorAll"):
            assert_fault_led(window, axis, suffix, False)


def test_robot_flags_all_zero_sets_leds_inactive_and_enable_button():
    window = FakeWindow()

    window.on_robot_flags_msg(flag_msg([0, 0, 0, 0, 0, 0]))

    assert_all_axes_inactive(window)
    assert window.btnRobotEnable.text() == "Enable"
    assert window.robot_enable_led is False


def test_robot_flags_axis_1_servo_on_only():
    window = FakeWindow()

    window.on_robot_flags_msg(flag_msg([SERVO_ON, 0, 0, 0, 0, 0]))

    assert_normal_led(window, 1, "ServoOn", True)
    for axis in range(2, JOINT_COUNT + 1):
        assert_normal_led(window, axis, "ServoOn", False)
    assert window.btnRobotEnable.text() == "Enable"


def test_robot_flags_all_axes_servo_on_sets_disable_button():
    window = FakeWindow()

    window.on_robot_flags_msg(flag_msg([SERVO_ON] * JOINT_COUNT))

    for axis in range(1, JOINT_COUNT + 1):
        assert_normal_led(window, axis, "ServoOn", True)
    assert window.btnRobotEnable.text() == "Disable"
    assert window.robot_enable_led is True


def test_robot_flags_axis_3_running_and_origin_ok():
    window = FakeWindow()

    window.on_robot_flags_msg(flag_msg([0, 0, RUNNING | ORG_SET_OK, 0, 0, 0]))

    assert_normal_led(window, 3, "Running", True)
    assert_normal_led(window, 3, "OrgOK", True)
    for axis in (1, 2, 4, 5, 6):
        assert_normal_led(window, axis, "Running", False)
        assert_normal_led(window, axis, "OrgOK", False)


def test_robot_flags_axis_5_limits_do_not_shift_index():
    window = FakeWindow()

    window.on_robot_flags_msg(flag_msg([0, 0, 0, 0, SOF_LIMIT_P | SOF_LIMIT_M, 0]))

    assert_normal_led(window, 5, "LimitPositive", True)
    assert_normal_led(window, 5, "LimitNegative", True)
    for axis in (1, 2, 3, 4, 6):
        assert_normal_led(window, axis, "LimitPositive", False)
        assert_normal_led(window, axis, "LimitNegative", False)


def test_robot_flags_axis_6_faults_do_not_affect_other_axes():
    window = FakeWindow()

    window.on_robot_flags_msg(flag_msg([0, 0, 0, 0, 0, ALARM | EMG | ERROR_ALL]))

    assert_fault_led(window, 6, "Alarm", True)
    assert_fault_led(window, 6, "EMG", True)
    assert_fault_led(window, 6, "ErrorAll", True)
    for axis in range(1, 6):
        assert_fault_led(window, axis, "Alarm", False)
        assert_fault_led(window, axis, "EMG", False)
        assert_fault_led(window, axis, "ErrorAll", False)


def test_robot_flags_short_message_logs_warning_and_sets_missing_axes_inactive():
    window = FakeWindow()
    window.on_robot_flags_msg(flag_msg([SERVO_ON, SERVO_ON]))

    assert_normal_led(window, 1, "ServoOn", True)
    assert_normal_led(window, 2, "ServoOn", True)
    for axis in range(3, JOINT_COUNT + 1):
        assert_normal_led(window, axis, "ServoOn", False)
    assert window.btnRobotEnable.text() == "Enable"
    assert any("/robot_hw/flags has 2 axes; expected 6" in message for message in window.logs)


def test_robot_flags_parser_requires_axes_status_f():
    window = FakeWindow()
    msg = SimpleNamespace(data=[SERVO_ON] * JOINT_COUNT)

    assert window.parse_robot_flags_msg(msg) == []
    assert any("/robot_hw/flags message has no axes field" in message for message in window.logs)
