import robot_gui.main_window as main_window
from robot_gui.main_window import SERVO_ON, RobotMainWindow


class FakeSetBool:
    class Request:
        def __init__(self):
            self.data = None


class FakeResponse:
    success = True
    message = "ok"


class FakeFuture:
    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return FakeResponse()


class FakeServoClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture()


class FakeButton:
    def __init__(self, style=""):
        self._text = ""
        self._style = style

    def setText(self, text):
        self._text = text

    def text(self):
        return self._text

    def setStyleSheet(self, style):
        self._style = style

    def styleSheet(self):
        return self._style


class FakeWindow:
    set_robot_servo_all = RobotMainWindow.set_robot_servo_all
    toggle_robot_servo = RobotMainWindow.toggle_robot_servo
    update_robot_enable_from_status = RobotMainWindow.update_robot_enable_from_status
    update_robot_enable_button = RobotMainWindow.update_robot_enable_button
    _on_servo_all_response = RobotMainWindow._on_servo_all_response
    _call_ros_service = RobotMainWindow._call_ros_service

    def __init__(self, client):
        self._servo_all_client = client
        self._robot_servo_on = False
        self._robot_enable_style = "enable-style"
        self._robot_disable_style = "disable-style"
        self.btnRobotEnable = FakeButton()
        self.logs = []
        self.robot_enable_led = None
        self.connected = False

    def _button(self, name):
        return getattr(self, name, None)

    def _set_status_bar_led(self, name, active):
        if name == "RobotEnableStatus_led":
            self.robot_enable_led = active

    def log_ros2(self, message):
        self.logs.append(message)


def test_servo_service_called_even_when_connected_false(monkeypatch):
    monkeypatch.setattr(main_window, "SetBool", FakeSetBool)
    client = FakeServoClient(ready=True)
    window = FakeWindow(client)
    window.connected = False

    window.toggle_robot_servo()

    assert len(client.requests) == 1
    assert client.requests[0].data is True


def test_servo_off_status_button_enables_servo(monkeypatch):
    monkeypatch.setattr(main_window, "SetBool", FakeSetBool)
    client = FakeServoClient(ready=True)
    window = FakeWindow(client)

    window.update_robot_enable_from_status(0x00000000)
    window.toggle_robot_servo()

    assert window.btnRobotEnable.text() == "Enable"
    assert window.btnRobotEnable.styleSheet() == "enable-style"
    assert window.robot_enable_led is False
    assert client.requests[0].data is True


def test_servo_on_status_button_disables_servo(monkeypatch):
    monkeypatch.setattr(main_window, "SetBool", FakeSetBool)
    client = FakeServoClient(ready=True)
    window = FakeWindow(client)

    window.update_robot_enable_from_status(SERVO_ON)
    window.toggle_robot_servo()

    assert window.btnRobotEnable.text() == "Disable"
    assert window.btnRobotEnable.styleSheet() == "disable-style"
    assert window.robot_enable_led is True
    assert client.requests[0].data is False


def test_servo_service_not_ready_logs_warning(monkeypatch):
    monkeypatch.setattr(main_window, "SetBool", FakeSetBool)
    client = FakeServoClient(ready=False)
    window = FakeWindow(client)

    window.toggle_robot_servo()

    assert client.requests == []
    assert any("Service /robot_hw/servo_all is not available" in message for message in window.logs)
