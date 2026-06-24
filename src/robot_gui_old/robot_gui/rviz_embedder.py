import os
import shutil
import subprocess
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

try:
    from PyQt5.QtCore import QEvent, QObject, Qt, QTimer
    from PyQt5.QtWidgets import QLabel, QWidget
except ImportError:  # pragma: no cover - convenience for Windows/dev machines.
    from PyQt6.QtCore import QEvent, QObject, Qt, QTimer
    from PyQt6.QtWidgets import QLabel, QWidget

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - direct Python run outside ROS 2.
    get_package_share_directory = None


MOVEIT_RVIZ_CONFIG_RELATIVE_PATH = os.path.join("config", "moveit.rviz")
FALLBACK_MOVEIT_RVIZ_CONFIG_PATH = "/home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz"


class RvizEmbedder(QObject):
    def __init__(
        self,
        parent_widget: QWidget,
        placeholder_label: Optional[QLabel] = None,
        rviz_config_path: Optional[str] = None,
        log_callback: Optional[Callable[[str], None]] = None,
        parent: Optional[QObject] = None,
    ):
        super().__init__(parent)
        self.parent_widget = parent_widget
        self.placeholder_label = placeholder_label
        self.rviz_config_path = rviz_config_path
        self.log_callback = log_callback
        self.rviz_process: Optional[subprocess.Popen] = None
        self.rviz_pid: Optional[int] = None
        self.rviz_window_id: Optional[str] = None
        self.rviz_top_level_window_id: Optional[str] = None
        self.launch_command: List[str] = []
        self.selected_window_title = ""
        self.selected_window_class = ""
        self.selected_window_size: Tuple[int, int] = (0, 0)
        self._poll_started_at = 0.0
        self._poll_timeout_s = 10.0
        self._xdotool_path: Optional[str] = None
        self._wmctrl_path: Optional[str] = None
        self._xwininfo_path: Optional[str] = None
        self._xprop_path: Optional[str] = None
        self._logged_rejections: Set[str] = set()
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(300)
        self._poll_timer.timeout.connect(self._poll_for_window)
        self._prepare_container_widget()
        self.parent_widget.installEventFilter(self)

    def start(self) -> None:
        if self.rviz_process is not None and self.rviz_process.poll() is None:
            return

        self._set_placeholder("RViz")
        self._log("Starting embedded RViz2...")
        self.rviz_pid = None
        self.rviz_window_id = None
        self.rviz_top_level_window_id = None
        self.launch_command = []
        self.selected_window_title = ""
        self.selected_window_class = ""
        self.selected_window_size = (0, 0)
        self._logged_rejections.clear()

        if os.environ.get("QT_QPA_PLATFORM", "").lower() == "offscreen":
            self._fail("RViz embedding skipped because Qt is running offscreen.")
            return

        if not self._container_is_ready():
            self._fail(
                "RViz embed delayed/fail: embeddedRvizWidget is not visible or has invalid size "
                f"({self.parent_widget.width()}x{self.parent_widget.height()})"
            )
            return

        session_type = os.environ.get("XDG_SESSION_TYPE", "").lower()
        if session_type and session_type != "x11":
            self._log("RViz embedding requires X11/Xorg. Current session may not support window reparenting.")
            self._fail("RViz embedding requires X11/Xorg.")
            return

        if not os.environ.get("DISPLAY"):
            self._fail("DISPLAY is not set. Cannot embed RViz window.")
            return

        rviz_path = shutil.which("rviz2")
        if rviz_path is None:
            self._fail("rviz2 not found. Cannot start embedded RViz.")
            return

        if not self.rviz_config_path:
            self.rviz_config_path = self.resolve_moveit_rviz_config_path(self._log)
        if not os.path.exists(self.rviz_config_path):
            self._fail(f"RViz config file not found: {self.rviz_config_path}")
            return

        self._xdotool_path = shutil.which("xdotool")
        self._wmctrl_path = shutil.which("wmctrl")
        self._xwininfo_path = shutil.which("xwininfo")
        self._xprop_path = shutil.which("xprop")
        if self._xdotool_path is None or self._wmctrl_path is None:
            self._fail("xdotool or wmctrl not found. Cannot embed RViz window.")
            return

        command = [
            rviz_path,
            "-d",
            self.rviz_config_path,
            "--ros-args",
            "-r",
            "__node:=embedded_rviz",
        ]
        self.launch_command = list(command)
        self._log(
            "RViz launch command: "
            f"rviz2 -d {self.rviz_config_path} --ros-args -r __node:=embedded_rviz"
        )

        env = os.environ.copy()
        env["QT_QPA_PLATFORM"] = "xcb"

        try:
            self.rviz_process = subprocess.Popen(
                command,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            self.rviz_process = None
            self._fail(f"Failed to start RViz2: {exc}")
            return

        self.rviz_pid = int(self.rviz_process.pid)
        self._log(f"RViz process pid={self.rviz_pid}")
        self._log("Waiting for RViz window...")
        self._poll_started_at = time.monotonic()
        self._poll_timer.start()

    def stop(self) -> None:
        self._log("Stopping embedded RViz2...")
        self._poll_timer.stop()
        self.rviz_window_id = None
        if self.rviz_process is None:
            return

        process = self.rviz_process
        self.rviz_process = None
        self.rviz_pid = None
        if process.poll() is not None:
            return

        try:
            process.terminate()
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
        except Exception as exc:
            self._log(f"Failed to stop RViz2 cleanly: {exc}")

    def resize_embedded_rviz(self) -> None:
        self.resize_rviz_to_container()

    def resize_rviz_to_container(self) -> None:
        if not self.rviz_window_id or self._xdotool_path is None:
            return

        width, height = self._target_size()
        self._run_xdotool(["windowmove", self.rviz_window_id, "0", "0"])
        self._run_xdotool(["windowsize", self.rviz_window_id, str(width), str(height)])
        self._log(f"Resize RViz to {width}x{height}")

    def eventFilter(self, watched, event) -> bool:
        resize_type = getattr(QEvent, "Resize", None)
        show_type = getattr(QEvent, "Show", None)
        if resize_type is None:
            resize_type = QEvent.Type.Resize
        if show_type is None:
            show_type = QEvent.Type.Show
        if watched is self.parent_widget and event.type() == resize_type:
            QTimer.singleShot(0, self.resize_embedded_rviz)
        elif watched is self.parent_widget and event.type() == show_type:
            QTimer.singleShot(0, self.refresh_embedded_rviz)
        return super().eventFilter(watched, event)

    def refresh_embedded_rviz(self) -> None:
        if not self.rviz_window_id or self._xdotool_path is None:
            return
        self._sync_parent_widget_geometry()
        self._run_xdotool(["windowmap", str(int(self.parent_widget.winId()))])
        self._run_xdotool(["windowmap", self.rviz_window_id])
        self.resize_rviz_to_container()

    def _poll_for_window(self) -> None:
        if self.rviz_process is None:
            self._poll_timer.stop()
            return

        if self.rviz_process.poll() is not None:
            self._poll_timer.stop()
            self._fail("RViz process exited before window embedding.")
            return

        window_id = self._find_rviz_window()
        if window_id:
            self._poll_timer.stop()
            self.rviz_window_id = window_id
            self._log(f"Found RViz window id={window_id}")
            self._embed_window()
            return

        if time.monotonic() - self._poll_started_at >= self._poll_timeout_s:
            self._poll_timer.stop()
            pid = self.rviz_pid if self.rviz_pid is not None else "unknown"
            self._fail(f"RViz window for launched process was not found. pid={pid}")

    def _find_rviz_window(self) -> Optional[str]:
        if self.rviz_process is None or self.rviz_pid is None:
            return None

        return self.find_rviz_window_by_pid(self.rviz_pid)

    @staticmethod
    def resolve_moveit_rviz_config_path(log_callback: Optional[Callable[[str], None]] = None) -> str:
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory("robot_moveit")
                return os.path.join(share_dir, MOVEIT_RVIZ_CONFIG_RELATIVE_PATH)
            except Exception as exc:
                if log_callback is not None:
                    log_callback(
                        "Warning: failed to resolve robot_moveit share directory. "
                        f"Using fallback RViz config path. Error: {exc}"
                    )
        elif log_callback is not None:
            log_callback("Warning: ament_index_python is not available. Using fallback RViz config path.")

        return FALLBACK_MOVEIT_RVIZ_CONFIG_PATH

    def find_rviz_window_by_pid(self, pid: int) -> Optional[str]:
        self._log_once(f"search:{pid}", f"Searching RViz window by PID={pid}")
        visible_windows = self._visible_windows_for_pid(pid)
        tree_windows = self._xwininfo_tree_windows()
        top_level_windows = self._wmctrl_windows()
        top_level_by_id = {str(window["id"]): window for window in top_level_windows}
        candidates = self._candidate_windows(pid, top_level_windows, visible_windows, tree_windows)
        selected = None
        selected_area = -1

        for window in candidates:
            window_id = window["id"]
            window_pid = window["pid"]
            title = window["title"]
            if window_pid != pid:
                if "rviz" in title.lower():
                    self._log_once(
                        f"reject-pid:{window_id}",
                        f"Rejected window id={window_id}: pid mismatch ({window_pid} != {pid})",
                    )
                continue

            if visible_windows and window_id not in visible_windows:
                self._log_once(f"reject-visible:{window_id}", f"Rejected window id={window_id}: not visible")
                continue

            xprop_pid, wm_title, wm_class = self._window_properties(window_id)
            if xprop_pid is not None and xprop_pid != pid:
                self._log_once(
                    f"reject-xprop-pid:{window_id}",
                    f"Rejected window id={window_id}: pid mismatch ({xprop_pid} != {pid})",
                )
                continue

            width, height = self._window_size(window_id)
            title = wm_title or title
            area = width * height
            self._log_once(
                f"candidate:{window_id}",
                f"Candidate window id={window_id}, pid={pid}, title={title}, size={width}x{height}",
            )

            if width < 300 or height < 200:
                self._log_once(f"reject-small:{window_id}", f"Rejected window id={window_id}: too small ({width}x{height})")
                continue

            if not bool(window.get("is_top_level", False)):
                self._log_once(f"reject-child:{window_id}", f"Rejected window id={window_id}: not a top-level RViz window")
                continue

            class_or_title = f"{title} {wm_class}".lower()
            if "rviz" not in class_or_title:
                self._log_once(
                    f"reject-class:{window_id}",
                    f"Rejected window id={window_id}: title/class does not look like RViz",
                )
                continue

            if area > selected_area:
                selected_area = area
                selected = {
                    "id": window_id,
                    "title": title,
                    "class": wm_class,
                    "size": (width, height),
                    "top_level_id": window.get("top_level_id"),
                }

        if selected is None:
            self._log_once(f"not-found:{pid}", f"Failed to find RViz window for pid={pid}")
            return None

        self.selected_window_title = selected["title"]
        self.selected_window_class = selected["class"]
        self.selected_window_size = selected["size"]
        self.rviz_top_level_window_id = selected.get("top_level_id")
        self._log(f"Selected RViz window id={selected['id']}")
        return selected["id"]

    def _candidate_windows(
        self,
        pid: int,
        top_level_windows: List[Dict[str, object]],
        visible_windows: Set[str],
        tree_windows: Set[str],
    ) -> List[Dict[str, object]]:
        candidates_by_id: Dict[str, Dict[str, object]] = {}
        first_top_level_id = None

        for window in top_level_windows:
            window_id = str(window["id"])
            is_pid_match = int(window["pid"]) == int(pid)
            if is_pid_match and first_top_level_id is None:
                first_top_level_id = window_id
            candidates_by_id[window_id] = {
                **window,
                "is_top_level": True,
                "top_level_id": window_id if is_pid_match else None,
            }

        for window_id in visible_windows:
            if window_id in candidates_by_id:
                continue
            xprop_pid, title, wm_class = self._window_properties(window_id)
            if xprop_pid != pid:
                if "rviz" in f"{title} {wm_class}".lower():
                    self._log_once(
                        f"reject-pid:{window_id}",
                        f"Rejected window id={window_id}: pid mismatch ({xprop_pid} != {pid})",
                    )
                continue
            candidates_by_id[window_id] = {
                "id": window_id,
                "pid": pid,
                "title": title,
                "is_top_level": False,
                "top_level_id": first_top_level_id,
            }

        for window_id in tree_windows:
            if window_id in candidates_by_id:
                continue
            xprop_pid, title, wm_class = self._window_properties(window_id)
            if xprop_pid != pid:
                continue
            candidates_by_id[window_id] = {
                "id": window_id,
                "pid": pid,
                "title": title,
                "is_top_level": False,
                "top_level_id": first_top_level_id,
            }

        return list(candidates_by_id.values())

    def _largest_valid_content_candidate(self, pid: int, candidates: List[Dict[str, object]]) -> Optional[Dict[str, object]]:
        selected = None
        selected_area = -1
        for window in candidates:
            window_id = str(window["id"])
            xprop_pid, wm_title, wm_class = self._window_properties(window_id)
            if xprop_pid is not None and xprop_pid != pid:
                continue
            width, height = self._window_size(window_id)
            if width < 300 or height < 200:
                continue
            title = wm_title or str(window.get("title", ""))
            if "rviz" not in f"{title} {wm_class}".lower():
                continue
            area = width * height
            if area > selected_area:
                selected_area = area
                selected = {
                    "id": window_id,
                    "title": title,
                    "class": wm_class,
                    "size": (width, height),
                    "top_level_id": window.get("top_level_id"),
                }
        if selected is not None:
            self._log(f"Selected RViz content window id={selected['id']}")
        return selected

    def _wmctrl_windows(self) -> List[Dict[str, object]]:
        if self._wmctrl_path is None:
            return []
        result = self._run_tool([self._wmctrl_path, "-lp"]) or ""
        windows: List[Dict[str, object]] = []
        for line in result.splitlines():
            parts = line.split(None, 4)
            if len(parts) < 3:
                continue
            window_id = self._normalize_window_id(parts[0])
            if window_id is None:
                continue
            try:
                pid = int(parts[2])
            except ValueError:
                continue
            title = parts[4] if len(parts) >= 5 else ""
            windows.append({"id": window_id, "pid": pid, "title": title})
        return windows

    def _visible_windows_for_pid(self, pid: int) -> Set[str]:
        if self._xdotool_path is None:
            return set()
        result = self._run_tool([self._xdotool_path, "search", "--onlyvisible", "--pid", str(pid)]) or ""
        visible = set()
        for token in result.splitlines():
            window_id = self._normalize_window_id(token.strip())
            if window_id is not None:
                visible.add(window_id)
        return visible

    def _xwininfo_tree_windows(self) -> Set[str]:
        if self._xwininfo_path is None:
            return set()
        result = self._run_tool([self._xwininfo_path, "-root", "-tree"]) or ""
        windows = set()
        for line in result.splitlines():
            parts = line.strip().split(None, 1)
            if not parts:
                continue
            window_id = self._normalize_window_id(parts[0])
            if window_id is not None:
                windows.add(window_id)
        return windows

    def _window_size(self, window_id: str) -> Tuple[int, int]:
        if self._xwininfo_path is None:
            return (0, 0)
        result = self._run_tool([self._xwininfo_path, "-id", window_id]) or ""
        width = 0
        height = 0
        for line in result.splitlines():
            stripped = line.strip()
            if stripped.startswith("Width:"):
                width = self._parse_int_after_colon(stripped)
            elif stripped.startswith("Height:"):
                height = self._parse_int_after_colon(stripped)
        return (width, height)

    def _window_geometry(self, window_id: str) -> Optional[Tuple[int, int, int, int]]:
        if self._xwininfo_path is None:
            return None
        result = self._run_tool([self._xwininfo_path, "-id", window_id]) or ""
        x = None
        y = None
        width = None
        height = None
        for line in result.splitlines():
            stripped = line.strip()
            if stripped.startswith("Relative upper-left X:"):
                x = self._parse_int_after_colon(stripped)
            elif stripped.startswith("Relative upper-left Y:"):
                y = self._parse_int_after_colon(stripped)
            elif stripped.startswith("Width:"):
                width = self._parse_int_after_colon(stripped)
            elif stripped.startswith("Height:"):
                height = self._parse_int_after_colon(stripped)
        if x is None or y is None or width is None or height is None:
            return None
        return (x, y, width, height)

    def _window_parent(self, window_id: str) -> Optional[str]:
        if self._xwininfo_path is None:
            return None
        result = self._run_tool([self._xwininfo_path, "-id", window_id]) or ""
        for line in result.splitlines():
            stripped = line.strip()
            if not stripped.startswith("Parent window id:"):
                continue
            for token in stripped.split():
                normalized = self._normalize_window_id(token)
                if normalized is not None:
                    return normalized
        return None

    def _verify_embedded_geometry(self, container_win_id: str, expected_width: int, expected_height: int) -> bool:
        parent_after = self._window_parent(self.rviz_window_id)
        geometry = self._window_geometry(self.rviz_window_id)
        tree_ids = self._window_tree_ids(container_win_id)
        tree_contains_window = self.rviz_window_id in tree_ids
        self._log(f"Selected RViz window parent after reparent={parent_after}")
        self._log(f"RViz window found in container tree={tree_contains_window}")
        if geometry is not None:
            x, y, width, height = geometry
            self._log(f"RViz geometry after resize: x={x}, y={y}, size={width}x{height}")
        self._log_xwininfo_tree(container_win_id)
        parent_matches = parent_after == container_win_id or tree_contains_window
        if not parent_matches or geometry is None:
            return False
        x, y, width, height = geometry
        return x == 0 and y == 0 and width == expected_width and height == expected_height

    def _log_xwininfo_tree(self, container_win_id: str) -> None:
        if self._xwininfo_path is None:
            return
        result = self._run_tool([self._xwininfo_path, "-tree", "-id", container_win_id]) or ""
        lines = [line.strip() for line in result.splitlines() if line.strip()]
        if not lines:
            return
        self._log("xwininfo tree after reparent:")
        for line in lines[:12]:
            self._log(line)

    def _window_properties(self, window_id: str) -> Tuple[Optional[int], str, str]:
        if self._xprop_path is None:
            return (None, "", "")
        result = self._run_tool([self._xprop_path, "-id", window_id, "_NET_WM_PID", "WM_NAME", "WM_CLASS"]) or ""
        pid = None
        title = ""
        wm_class = ""
        for line in result.splitlines():
            if line.startswith("_NET_WM_PID"):
                try:
                    pid = int(line.rsplit("=", 1)[1].strip())
                except (IndexError, ValueError):
                    pid = None
            elif line.startswith("WM_NAME"):
                title = self._xprop_string_value(line)
            elif line.startswith("WM_CLASS"):
                wm_class = self._xprop_string_value(line)
        return (pid, title, wm_class)

    def _embed_window(self) -> None:
        if self.rviz_window_id is None:
            return

        if not self._container_is_ready():
            self._fail(
                "RViz embed failed: embeddedRvizWidget is not visible or has invalid size "
                f"({self.parent_widget.width()}x{self.parent_widget.height()})"
            )
            return

        self._prepare_container_widget()
        container_win_id = str(int(self.parent_widget.winId()))
        width, height = self._target_size()
        parent_before = self._window_parent(self.rviz_window_id)
        self._log(f"Selected RViz window parent before reparent={parent_before}")
        self._log(f"Container winId={container_win_id}, size={width}x{height}")
        self._remove_window_decoration(self.rviz_window_id)
        self._log(f"Embedding RViz into embeddedRvizWidget winId={container_win_id}")
        reparent = self._run_xdotool(["windowreparent", self.rviz_window_id, container_win_id])
        if reparent is None:
            self._fail("Failed to reparent RViz window.")
            return

        self._run_xdotool(["windowmove", self.rviz_window_id, "0", "0"])
        self._run_xdotool(["windowsize", self.rviz_window_id, str(width), str(height)])
        self._run_xdotool(["windowmap", self.rviz_window_id])
        self.resize_rviz_to_container()
        QTimer.singleShot(100, self.resize_rviz_to_container)
        QTimer.singleShot(300, self.resize_rviz_to_container)
        QTimer.singleShot(800, self.resize_rviz_to_container)
        QTimer.singleShot(1500, self.resize_rviz_to_container)
        QTimer.singleShot(1600, lambda: self._finish_embed_after_settle(container_win_id))

    def _container_is_ready(self) -> bool:
        if not self.parent_widget.isVisible():
            return False
        window = self.parent_widget.window()
        if window is not None and not window.isVisible():
            return False
        width, height = self._target_size()
        return width > 10 and height > 10

    def _finish_embed_after_settle(self, container_win_id: str) -> None:
        if self.rviz_window_id is None:
            return
        width, height = self._target_size()
        if not self._verify_embedded_geometry(container_win_id, width, height):
            self._fail("RViz embed failed: geometry/parent verification did not pass.")
            return
        self._hide_placeholder()
        self._log("RViz embedded successfully.")

    def _prepare_container_widget(self) -> None:
        self.parent_widget.setAttribute(self._qt_widget_attribute("WA_NativeWindow"), True)
        self.parent_widget.setAttribute(self._qt_widget_attribute("WA_DontCreateNativeAncestors"), False)
        self.parent_widget.setContentsMargins(0, 0, 0, 0)
        layout = self.parent_widget.layout()
        if layout is not None:
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)

    def _hide_placeholder(self) -> None:
        if self.placeholder_label is None:
            return
        self.placeholder_label.hide()
        layout = self.parent_widget.layout()
        if layout is not None:
            layout.invalidate()

    def _remove_window_decoration(self, window_id: str) -> None:
        if self._xprop_path is None:
            self._log("Decoration removed: no (xprop not found)")
            return
        result = self._run_tool(
            [
                self._xprop_path,
                "-id",
                window_id,
                "-f",
                "_MOTIF_WM_HINTS",
                "32c",
                "-set",
                "_MOTIF_WM_HINTS",
                "0x2, 0x0, 0x0, 0x0, 0x0",
            ],
            allow_failure=False,
        )
        if result is None:
            self._log("Decoration removed: no")
            return
        self._run_tool([self._xdotool_path, "windowunmap", window_id], allow_failure=True)
        self._run_tool([self._xdotool_path, "windowmap", window_id], allow_failure=True)
        self._log("Decoration removed: yes")

    def _adopt_embedded_child_if_available(self) -> None:
        if self.rviz_window_id is None:
            return
        embedded_child = self._largest_rviz_descendant(str(int(self.parent_widget.winId())))
        if not embedded_child or embedded_child == self.rviz_window_id:
            return
        if self.rviz_top_level_window_id is None:
            self.rviz_top_level_window_id = self.rviz_window_id
        self._log(f"Using embedded RViz child window id={embedded_child}")
        self.rviz_window_id = embedded_child
        if self.rviz_top_level_window_id != self.rviz_window_id:
            self._park_top_level_window(self.rviz_top_level_window_id)

    def _sync_parent_widget_geometry(self) -> None:
        parent = self.parent_widget.parentWidget()
        if parent is None:
            return
        rect = parent.contentsRect()
        if rect.width() <= 0 or rect.height() <= 0:
            return
        if rect.width() > self.parent_widget.width() or rect.height() > self.parent_widget.height():
            self.parent_widget.setGeometry(rect)

    def _target_size(self) -> Tuple[int, int]:
        width = max(1, int(self.parent_widget.width()))
        height = max(1, int(self.parent_widget.height()))
        return width, height

    def _park_top_level_window(self, window_id: str) -> None:
        screen_width, screen_height = self._root_window_size()
        x = max(0, screen_width - 120)
        y = max(0, screen_height - 120)
        if self._wmctrl_path is not None:
            self._run_tool(
                [self._wmctrl_path, "-ir", window_id, "-b", "remove,maximized_vert,maximized_horz"],
                allow_failure=True,
            )
            self._run_tool(
                [self._wmctrl_path, "-ir", window_id, "-e", f"0,{x},{y},115,117"],
                allow_failure=True,
            )
        self._run_xdotool(["windowmove", window_id, str(x), str(y)])
        self._run_xdotool(["windowsize", window_id, "115", "117"])

    def _root_window_size(self) -> Tuple[int, int]:
        if self._xwininfo_path is None:
            return (1920, 1080)
        result = self._run_tool([self._xwininfo_path, "-root"]) or ""
        width = 1920
        height = 1080
        for line in result.splitlines():
            stripped = line.strip()
            if stripped.startswith("Width:"):
                width = self._parse_int_after_colon(stripped)
            elif stripped.startswith("Height:"):
                height = self._parse_int_after_colon(stripped)
        return (max(1, width), max(1, height))

    def _largest_rviz_descendant(self, container_win_id: str) -> Optional[str]:
        if self.rviz_pid is None:
            return None
        selected = None
        selected_area = -1
        for window_id in self._window_tree_ids(container_win_id):
            if window_id == container_win_id:
                continue
            xprop_pid, title, wm_class = self._window_properties(window_id)
            if xprop_pid != self.rviz_pid:
                continue
            if "rviz" not in f"{title} {wm_class}".lower():
                continue
            width, height = self._window_size(window_id)
            if width < 300 or height < 200:
                continue
            area = width * height
            if area > selected_area:
                selected = window_id
                selected_area = area
        return selected

    def _window_tree_ids(self, window_id: str) -> Set[str]:
        if self._xwininfo_path is None:
            return set()
        result = self._run_tool([self._xwininfo_path, "-id", window_id, "-tree"]) or ""
        windows = set()
        for line in result.splitlines():
            parts = line.strip().split(None, 1)
            if not parts:
                continue
            parsed_window_id = self._normalize_window_id(parts[0])
            if parsed_window_id is not None:
                windows.add(parsed_window_id)
        return windows

    def _run_xdotool(self, args) -> Optional[str]:
        if self._xdotool_path is None:
            return None
        return self._run_tool([self._xdotool_path] + list(args), allow_failure=False)

    @staticmethod
    def _qt_widget_attribute(name: str):
        if hasattr(Qt, name):
            return getattr(Qt, name)
        return getattr(Qt.WidgetAttribute, name)

    def _run_tool(self, command, allow_failure: bool = True) -> Optional[str]:
        try:
            completed = subprocess.run(
                command,
                check=not allow_failure,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
            )
            return completed.stdout.strip()
        except Exception as exc:
            if not allow_failure:
                self._log(f"Command failed: {' '.join(command)} ({exc})")
                return None
            return ""

    def _fail(self, message: str) -> None:
        self._log(message)
        self._set_placeholder(message)
        if self.placeholder_label is not None:
            self.placeholder_label.show()
        if self.rviz_process is not None and self.rviz_process.poll() is None:
            self.stop()

    def _set_placeholder(self, text: str) -> None:
        if self.placeholder_label is not None and hasattr(self.placeholder_label, "setText"):
            self.placeholder_label.setText(text)

    def _log(self, message: str) -> None:
        if self.log_callback is not None:
            self.log_callback(message)

    def _log_once(self, key: str, message: str) -> None:
        if key in self._logged_rejections:
            return
        self._logged_rejections.add(key)
        self._log(message)

    @staticmethod
    def _parse_int_after_colon(value: str) -> int:
        try:
            return int(value.split(":", 1)[1].strip())
        except (IndexError, ValueError):
            return 0

    @staticmethod
    def _xprop_string_value(line: str) -> str:
        if "=" not in line:
            return ""
        value = line.split("=", 1)[1].strip()
        parts = []
        current = []
        in_quote = False
        escaped = False
        for char in value:
            if escaped:
                current.append(char)
                escaped = False
                continue
            if char == "\\":
                escaped = True
                continue
            if char == '"':
                if in_quote:
                    parts.append("".join(current))
                    current = []
                in_quote = not in_quote
                continue
            if in_quote:
                current.append(char)
        return " ".join(parts)

    @staticmethod
    def _normalize_window_id(value: str) -> Optional[str]:
        if not value:
            return None
        try:
            if value.lower().startswith("0x"):
                return str(int(value, 16))
            return str(int(value))
        except ValueError:
            return None
