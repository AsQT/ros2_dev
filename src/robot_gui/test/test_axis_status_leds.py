from types import SimpleNamespace

from robot_gui.main_window import (
    ALARM,
    EMG,
    ERROR_ALL,
    FAULT_LED_ACTIVE,
    FAULT_LED_NORMAL,
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


class FakeWindow:
    _set_led_state = RobotMainWindow._set_led_state
    _set_normal_led = RobotMainWindow._set_normal_led
    _set_fault_led = RobotMainWindow._set_fault_led
    update_axis_status_leds = RobotMainWindow.update_axis_status_leds
    update_axis_leds = RobotMainWindow.update_axis_leds
    update_flag_status = RobotMainWindow.update_flag_status
    on_robot_flags_msg = RobotMainWindow.on_robot_flags_msg
    parse_robot_flags_msg = RobotMainWindow.parse_robot_flags_msg
    update_all_axis_leds_from_flags = RobotMainWindow.update_all_axis_leds_from_flags

    def __init__(self):
        self.robot_servo_on = None
        self.logs = []
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
        for suffix in suffixes:
            setattr(self, f"ledAxis1{suffix}", FakeLed())

    def update_robot_enable_button(self, servo_on):
        self.robot_servo_on = servo_on

    def log_ros2(self, message):
        self.logs.append(message)


def assert_led_colors(window, active_normal=(), active_fault=()):
    normal_suffixes = ("ServoOn", "Running", "OrgOK", "LimitPositive", "LimitNegative")
    fault_suffixes = ("Alarm", "EMG", "ErrorAll")

    for suffix in normal_suffixes:
        expected = NORMAL_LED_ACTIVE if suffix in active_normal else NORMAL_LED_INACTIVE
        assert expected in getattr(window, f"ledAxis1{suffix}").style

    for suffix in fault_suffixes:
        expected = FAULT_LED_ACTIVE if suffix in active_fault else FAULT_LED_NORMAL
        assert expected in getattr(window, f"ledAxis1{suffix}").style


def test_axis_status_leds_all_bits_off():
    window = FakeWindow()
    window.update_axis_status_leds(1, 0x00000000)

    assert_led_colors(window)


def test_axis_status_leds_servo_on():
    window = FakeWindow()
    window.update_axis_status_leds(1, SERVO_ON)

    assert_led_colors(window, active_normal=("ServoOn",))


def test_axis_status_leds_running_and_origin_ok():
    window = FakeWindow()
    window.update_axis_status_leds(1, RUNNING | ORG_SET_OK)

    assert_led_colors(window, active_normal=("Running", "OrgOK"))


def test_axis_status_leds_limits():
    window = FakeWindow()
    window.update_axis_status_leds(1, SOF_LIMIT_P | SOF_LIMIT_M)

    assert_led_colors(window, active_normal=("LimitPositive", "LimitNegative"))


def test_axis_status_leds_faults():
    window = FakeWindow()
    window.update_axis_status_leds(1, ALARM | EMG | ERROR_ALL)

    assert_led_colors(window, active_fault=("Alarm", "EMG", "ErrorAll"))


def test_axis_status_leds_mixed_bits():
    window = FakeWindow()
    window.update_axis_status_leds(1, SERVO_ON | RUNNING | ORG_SET_OK | ALARM)

    assert_led_colors(window, active_normal=("ServoOn", "Running", "OrgOK"), active_fault=("Alarm",))


def test_flag_status_uses_message_order_for_gui_axis():
    window = FakeWindow()
    msg = SimpleNamespace(axes=[SimpleNamespace(status_f=SERVO_ON)])

    window.update_flag_status(msg)

    assert_led_colors(window, active_normal=("ServoOn",))
