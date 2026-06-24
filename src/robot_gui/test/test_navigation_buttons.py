import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

try:
    from PyQt5.QtWidgets import QApplication
except ImportError:  # pragma: no cover - convenience for Windows/dev machines.
    from PyQt6.QtWidgets import QApplication

from robot_gui.main_window import NAVIGATION_PAGES, RobotMainWindow


@pytest.fixture
def window():
    app = QApplication.instance() or QApplication([])
    gui = RobotMainWindow()
    yield gui
    gui.close()
    app.processEvents()


def assert_active_nav_button(window, active_index):
    for button_name, page_index in NAVIGATION_PAGES:
        button = getattr(window, button_name)
        assert button.isCheckable()
        assert button.isChecked() is (page_index == active_index)


def test_default_page_button_is_active(window):
    current_index = window.stackedWidget_MainPages.currentIndex()

    assert_active_nav_button(window, current_index)


def test_clicking_each_page_button_updates_index_and_active_button(window):
    for button_name, page_index in NAVIGATION_PAGES:
        button = getattr(window, button_name)

        button.click()
        QApplication.processEvents()

        assert window.stackedWidget_MainPages.currentIndex() == page_index
        assert_active_nav_button(window, page_index)


def test_programmatic_page_change_updates_active_button(window):
    page_count = window.stackedWidget_MainPages.count()
    for _, page_index in NAVIGATION_PAGES:
        window.stackedWidget_MainPages.setCurrentIndex((page_index + 1) % page_count)
        QApplication.processEvents()

        window.stackedWidget_MainPages.setCurrentIndex(page_index)
        QApplication.processEvents()

        assert_active_nav_button(window, page_index)
