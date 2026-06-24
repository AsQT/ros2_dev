import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

try:
    from PyQt5.QtWidgets import QApplication, QLabel
except ImportError:  # pragma: no cover - convenience for Windows/dev machines.
    from PyQt6.QtWidgets import QApplication, QLabel

from robot_gui.main_window import AXIS_LED_WIDGETS, JOINT_COUNT, RobotMainWindow


@pytest.fixture
def window():
    app = QApplication.instance() or QApplication([])
    gui = RobotMainWindow()
    yield gui
    gui.close()
    app.processEvents()


def test_axis_led_widgets_are_mapped_from_ui(window):
    assert len(window.axis_leds) == JOINT_COUNT

    for axis in range(1, JOINT_COUNT + 1):
        assert set(window.axis_leds[axis]) == set(AXIS_LED_WIDGETS)
        for led_key, widget in window.axis_leds[axis].items():
            _label, suffix, _mask, _kind = AXIS_LED_WIDGETS[led_key]
            assert isinstance(widget, QLabel)
            assert widget.objectName() == f"ledAxis{axis}{suffix}"
            assert widget.minimumWidth() >= 14
            assert widget.minimumHeight() >= 14
