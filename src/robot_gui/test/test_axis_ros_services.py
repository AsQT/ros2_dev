import math

import robot_gui.main_window as main_window
from robot_gui.main_window import RobotMainWindow


class FakeResponse:
    ok = True
    error_code = 0
    message = "ok"


class FakeFuture:
    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return FakeResponse()


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture()


class FakeJog:
    class Request:
        def __init__(self):
            self.id = None
            self.vel = None
            self.dir = None


class FakeHome:
    class Request:
        def __init__(self):
            self.id = None


class FakeStopAxis:
    class Request:
        def __init__(self):
            self.id = None


class FakeStopAll:
    class Request:
        pass


class FakeRunAxis:
    class Request:
        def __init__(self):
            self.id = None
            self.pos = None
            self.vel = None


class FakeWindow:
    home_axis = RobotMainWindow.home_axis
    stop_axis = RobotMainWindow.stop_axis
    start_jog = RobotMainWindow.start_jog
    stop_jog = RobotMainWindow.stop_jog
    run_absolute = RobotMainWindow.run_absolute
    emergency_stop = RobotMainWindow.emergency_stop
    _call_ros_service = RobotMainWindow._call_ros_service
    _log_ros_service_response = RobotMainWindow._log_ros_service_response
    _velocity_deg_s = RobotMainWindow._velocity_deg_s
    _to_float = staticmethod(RobotMainWindow._to_float)

    def __init__(self):
        self.axis_to_robot_id = {axis: axis - 1 for axis in range(1, 7)}
        self._jog_client = FakeClient()
        self._home_client = FakeClient()
        self._run_axis_client = FakeClient()
        self._stop_axis_client = FakeClient()
        self._stop_all_client = FakeClient()
        self._last_jog_ms = {}
        self.commanded_deg = [0.0] * 6
        self.logs = []
        self.texts = {}
        self.connected = False

    def _text(self, name, default=""):
        return self.texts.get(name, default)

    def _set_normal_led(self, name, active):
        pass

    def publish_joint_trajectory(self, axis):
        pass

    def log_ros2(self, message):
        self.logs.append(message)

    def log_hardware(self, message):
        self.logs.append(message)


def install_fake_service_types(monkeypatch):
    monkeypatch.setattr(main_window, "Jog", FakeJog)
    monkeypatch.setattr(main_window, "Home", FakeHome)
    monkeypatch.setattr(main_window, "StopAxis", FakeStopAxis)
    monkeypatch.setattr(main_window, "StopAll", FakeStopAll)
    monkeypatch.setattr(main_window, "RunAxis", FakeRunAxis)


def assert_no_connected_error(window):
    assert not any("Robot is not connected" in message for message in window.logs)


def test_axis_services_do_not_require_connected(monkeypatch):
    install_fake_service_types(monkeypatch)
    window = FakeWindow()
    window.connected = False
    window.texts["txtAxis1CommandPos"] = "50"
    window.texts["txtAxis1CommandVel"] = "10"

    window.home_axis(1)
    window.stop_axis(1)
    window.start_jog(1, 1)
    window.run_absolute(1)
    window.emergency_stop()

    assert len(window._home_client.requests) == 1
    assert len(window._stop_axis_client.requests) == 1
    assert len(window._jog_client.requests) == 1
    assert len(window._run_axis_client.requests) == 1
    assert len(window._stop_all_client.requests) == 1
    assert_no_connected_error(window)


def test_jog_axis_mapping_is_zero_based(monkeypatch):
    install_fake_service_types(monkeypatch)
    window = FakeWindow()
    window.texts["txtAxis1CommandVel"] = "10"
    window.texts["txtAxis6CommandVel"] = "10"

    window.start_jog(1, 1)
    window.start_jog(6, 1)

    assert window._jog_client.requests[0].id == 0
    assert window._jog_client.requests[0].dir == 1
    assert window._jog_client.requests[1].id == 5
    assert window._jog_client.requests[1].dir == 1
    assert_no_connected_error(window)


def test_jog_negative_uses_negative_direction(monkeypatch):
    install_fake_service_types(monkeypatch)
    window = FakeWindow()
    window.texts["txtAxis2CommandVel"] = "10"

    window.start_jog(2, -1)

    assert window._jog_client.requests[0].id == 1
    assert window._jog_client.requests[0].dir == 0


def test_run_axis_uses_degrees_input_and_ros_radians_service(monkeypatch):
    install_fake_service_types(monkeypatch)
    window = FakeWindow()
    window.texts["txtAxis1CommandPos"] = "50"
    window.texts["txtAxis1CommandVel"] = "10"

    window.run_absolute(1)

    request = window._run_axis_client.requests[0]
    assert request.id == 0
    assert math.isclose(request.pos, math.radians(50.0))
    assert math.isclose(request.vel, math.radians(10.0))
    assert window.commanded_deg[0] == 50.0
    assert_no_connected_error(window)


def test_unavailable_axis_service_logs_service_unavailable(monkeypatch):
    install_fake_service_types(monkeypatch)
    window = FakeWindow()
    window._home_client.ready = False

    window.home_axis(1)

    assert window._home_client.requests == []
    assert any("Service /robot_hw/home is not available" in message for message in window.logs)
    assert_no_connected_error(window)
