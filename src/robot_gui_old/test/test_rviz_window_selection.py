import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PyQt5.QtWidgets import QApplication, QLabel, QWidget
except ImportError:  # pragma: no cover - convenience for Windows/dev machines.
    from PyQt6.QtWidgets import QApplication, QLabel, QWidget

from robot_gui.rviz_embedder import RvizEmbedder
import robot_gui.rviz_embedder as rviz_embedder_module


class FakeRvizEmbedder(RvizEmbedder):
    def __init__(self, parent_widget, placeholder_label):
        super().__init__(parent_widget=parent_widget, placeholder_label=placeholder_label)
        self._xdotool_path = "/usr/bin/xdotool"
        self._wmctrl_path = "/usr/bin/wmctrl"
        self._xwininfo_path = "/usr/bin/xwininfo"
        self._xprop_path = "/usr/bin/xprop"

    def _run_tool(self, command, allow_failure=True):
        joined = " ".join(command)
        if "wmctrl -lp" in joined:
            return "\n".join(
                [
                    "0x01000001  0  1111 host RViz - external",
                    "0x01000002  0  2222 host RViz dialog",
                    "0x01000003  0  2222 host RViz main",
                    "0x01000004  0  2222 host Other tool",
                ]
            )
        if "xdotool search --onlyvisible --pid 2222" in joined:
            return "\n".join(["16777218", "16777219", "16777220", "16777221"])
        if "xwininfo -id 16777218" in joined:
            return "Width: 220\nHeight: 120\n"
        if "xwininfo -id 16777219" in joined:
            return "Width: 900\nHeight: 650\n"
        if "xwininfo -id 16777220" in joined:
            return "Width: 700\nHeight: 500\n"
        if "xwininfo -id 16777221" in joined:
            return "Width: 880\nHeight: 620\n"
        if "xprop -id 16777218" in joined:
            return '_NET_WM_PID = 2222\nWM_NAME(STRING) = "RViz dialog"\nWM_CLASS(STRING) = "rviz2", "rviz2"\n'
        if "xprop -id 16777219" in joined:
            return '_NET_WM_PID = 2222\nWM_NAME(STRING) = "RViz main"\nWM_CLASS(STRING) = "rviz2", "rviz2"\n'
        if "xprop -id 16777220" in joined:
            return '_NET_WM_PID = 2222\nWM_NAME(STRING) = "Other tool"\nWM_CLASS(STRING) = "other", "other"\n'
        if "xprop -id 16777221" in joined:
            return '_NET_WM_PID = 2222\nWM_NAME(STRING) = ""\nWM_CLASS(STRING) = "rviz2", "rviz2"\n'
        return ""


def test_rviz_window_selection_uses_launched_pid_and_largest_window():
    app = QApplication.instance() or QApplication([])
    parent = QWidget()
    placeholder = QLabel(parent)
    embedder = FakeRvizEmbedder(parent, placeholder)

    selected = embedder.find_rviz_window_by_pid(2222)

    assert selected == "16777219"
    assert embedder.selected_window_title == "RViz main"
    assert embedder.selected_window_class == "rviz2 rviz2"
    assert embedder.selected_window_size == (900, 650)

    parent.close()
    app.processEvents()


def test_rviz_config_resolves_robot_moveit_share_path(monkeypatch):
    def fake_share_directory(package_name):
        assert package_name == "robot_moveit"
        return "/tmp/robot_moveit_share"

    monkeypatch.setattr(rviz_embedder_module, "get_package_share_directory", fake_share_directory)

    assert RvizEmbedder.resolve_moveit_rviz_config_path() == "/tmp/robot_moveit_share/config/moveit.rviz"


def test_rviz_config_falls_back_to_install_path_when_share_lookup_fails(monkeypatch):
    logs = []

    def fake_share_directory(package_name):
        raise RuntimeError(f"{package_name} not found")

    monkeypatch.setattr(rviz_embedder_module, "get_package_share_directory", fake_share_directory)

    assert (
        RvizEmbedder.resolve_moveit_rviz_config_path(logs.append)
        == "/home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz"
    )
    assert any("Using fallback RViz config path" in message for message in logs)
