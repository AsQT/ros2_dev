# Embedded RViz Report

## 1. File đã sửa

- `robot_gui/robot_gui/rviz_embedder.py`
- `robot_gui/robot_gui/main_window.py`
- `robot_gui/test/test_rviz_window_selection.py`

## 2. Launch mode

- GUI tự launch RViz: yes
- Có launch RViz độc lập trong `real.launch.py` không: no
- `robot_gui.launch.py`: chỉ launch node `robot_gui`
- `robot_moveit/launch/moveit.launch.py`: `node_rviz` không nằm trong `LaunchDescription`
- Lưu ý: `moveit_mock.launch.py` vẫn có RViz riêng nếu chạy mock launch độc lập.

## 3. RViz config

- Package: `robot_moveit`
- Config: `config/moveit.rviz`
- Resolved path: `/home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz`
- Exists: yes

## 4. RViz process

- Command: `rviz2 -d /home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz --ros-args -r __node:=embedded_rviz`
- PID kiểm tra trực tiếp gần nhất: `44757`
- Parent PID: `44565` (`robot_gui`)

## 5. Window selection

- Method: launch RViz bằng subprocess, lưu PID, tìm top-level window theo đúng PID qua `wmctrl -lp`, `xdotool search --pid`, `xwininfo`, `xprop`, rồi chọn window top-level lớn nhất.
- Candidate windows quan sát được:
  - Top-level RViz: `0x2200106`, PID `44757`, class `rviz2`, title `/home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz - RViz`, size `1920x1011`
  - Content child RViz: `0x2200013`, PID `44757`, class `rviz2`, title empty, observed under `embeddedRvizWidget`
- Selected window id: top-level RViz window lớn nhất cùng PID. Không dùng fallback `xdotool search --name RViz`, nên không attach nhầm RViz khác.
- Selected PID: PID của RViz do GUI tự launch.
- Selected title/class: selected top-level title thường là `<robot_moveit config path> - RViz`, class `rviz2`.
- Selected size: resize theo `embeddedRvizWidget`.

## 6. Embed result

- Container widget: `embeddedRvizWidget`
- Container winId quan sát được: `0x1a00021`
- Reparent result: reparent trực tiếp selected top-level RViz window vào container `embeddedRvizWidget`.
- Resize result: code resize qua `xdotool windowmove/windowsize` ngay sau reparent và lặp lại bằng `QTimer` ở 100 ms, 300 ms, 800 ms.
- Lưu ý thực tế: default page của `.ui` là Robot (`currentIndex=2`), nên RViz page Main ban đầu có thể `UnMapped`; `update_tab_buttons()` gọi `refresh_embedded_rviz()` khi chuyển sang page Main.

## 7. Cleanup result

- RViz process stopped when GUI closed: implemented in `closeEvent()` via `RvizEmbedder.stop()`.
- Test bằng Ctrl-C launch: sau khi launch bị kill, không còn process `rviz2`/`robot_gui` sót lại.
- Lưu ý: Ctrl-C của launch hiện làm `robot_gui` bị SIGKILL sau timeout do `KeyboardInterrupt` trong Qt spin timer; đây là shutdown path của launch, không phải close GUI bằng nút đóng cửa sổ.

## 8. Test result

- Build: `colcon build --packages-select robot_gui robot_bringup robot_moveit robot_hardware_interface robot_control --event-handlers console_direct+` passed.
- Incremental build after final edit: `colcon build --packages-select robot_gui --event-handlers console_direct+` passed.
- Unit test: `PYTHONPATH=/home/minhquang/ros2_dev/src/robot_gui pytest robot_gui/test/test_rviz_window_selection.py -q` passed, `3 passed`.
- Test without external RViz: GUI tự launch RViz PID `44757`, command/config đúng, parent là `robot_gui`, không cần mở RViz sẵn.
- Test with external RViz already open: covered by unit test `test_rviz_window_selection_uses_launched_pid_and_largest_window`; external PID mismatch bị reject.
- Direct `real.launch.py`: partial pass. RViz launch/config/PID/window reparent được xác nhận; launch tổng thể vẫn fail do `Robot TCP connect failed: 192.168.2.50:5000: TCP connect timeout`, sau đó `ros2_control_node` abort và `move_group` segfault khi shutdown.
- Current ROS graph check: no ROS nodes/topics running during this edit session, so live robot model visibility was not re-verified.
- Final status: RViz embedding logic dùng PID-owned subprocess, chọn top-level RViz window lớn nhất theo PID và reparent/resize vào `embeddedRvizWidget`; lỗi hardware TCP/shutdown nếu còn xuất hiện là ngoài phạm vi RViz embedder.

## Fullscreen / Geometry Fix

- embeddedRvizWidget size: lấy trực tiếp từ `embeddedRvizWidget.width()` / `height()`, không lấy kích thước parent ngoài.
- container winId: tạo native window bằng `WA_NativeWindow=True`, `WA_DontCreateNativeAncestors=False` trước khi lấy `winId()`.
- selected RViz window id: log runtime `Selected RViz window id=<id>`.
- selected RViz window size before reparent: log runtime `Candidate window id=<id>, pid=<pid>, title=<title>, size=<w>x<h>`.
- selected RViz window size after resize: log runtime `Resize RViz to <w>x<h>`.
- decoration removed: attempted with `_MOTIF_WM_HINTS`; log runtime `Decoration removed: yes/no`.
- placeholder hidden/removed: placeholder is retained in the layout; `labelRvizPlaceholder.hide()` is called only after embed geometry verification passes.
- final result: code-side fullscreen geometry fix implemented; visual OK/FAIL cần xác nhận trong X11 GUI session.

## Robot Model Load Check

- RViz config path: resolved bằng `get_package_share_directory("robot_moveit")/config/moveit.rviz`.
- fixed frame: `world` trong `robot_moveit/config/moveit.rviz`.
- latest live check: see `Robot Model Visibility` below.
- root cause if not visible: nếu `/robot_description` thiếu thì RViz không load model; nếu có description nhưng thiếu `/joint_states` thì model có thể ở default pose hoặc báo state lỗi; nếu thiếu TF `world -> base_link` thì Fixed Frame `world` có thể làm model không hiển thị đúng; nếu config sai path thì MotionPlanning/RobotModel không load đúng.

## Runtime Failure Investigation

- GUI load without RViz: `ros2 launch robot_bringup real.launch.py embed_rviz:=false` smoke test ran; GUI process started and no `rviz2` process was launched.
- embed_rviz parameter: added to `robot_gui.launch.py`, forwarded from `real.launch.py`, and read by `RobotMainWindow` as ROS parameter `embed_rviz`.
- Main page visible before RViz start: fixed. RViz is no longer started in `_setup_rviz_embedder()` immediately; it starts only from `_maybe_start_rviz_embedder()` when page Main is active.
- embeddedRvizWidget visible: checked before start with `isVisible()`, `window().isVisible()`, and size > 10.
- embeddedRvizWidget size: mock test observed `971x600`.
- Placeholder retained until embed success: fixed. `labelRvizPlaceholder.setParent(None)` removed; placeholder hides only after geometry verification passes.
- RViz started only when widget visible: OK. Runtime log: `RViz start check: main_active=True, widget_visible=True, window_visible=True, size=971x600`.
- Selected top-level window test: OK. Runtime selected PID-owned top-level RViz window `62914566`; smaller/transient windows were rejected.
- Selected child/content window test: content child mode not selected. Verification uses `xwininfo -tree` to confirm the selected RViz window becomes a child of `embeddedRvizWidget`.
- Final selected window: top-level PID-owned RViz window, then reparented into `embeddedRvizWidget`.
- xwininfo parent after reparent: direct parent query returned `None` on this X11 session, but `xwininfo -tree -id <container>` showed child `0x3c00006 "rviz2"` under container.
- geometry after resize: `971x600+0+0`; verified by `xwininfo`.
- fallback behavior: if start/window/reparent/geometry verification fails, GUI stops its RViz subprocess and placeholder displays the explicit error text instead of leaving a blank widget.

## Robot Model Visibility

- Test launch used: `ros2 launch robot_gui embedded_rviz_test.launch.py`.
- robot_description: OK. `/robot_state_publisher` returned URDF XML from `robot_description/urdf/robot.urdf.xacro`.
- robot_description_semantic: OK. `/move_group` returned SRDF XML from `robot_moveit/config/robot.srdf`.
- joint_states: OK. `/joint_states` contained `joint_1..joint_6`, `joint_gl`, `joint_gr`, all at `0.0`.
- tf_static: OK. `/tf_static` contains `world -> base_link` plus static robot sensor/tool transforms.
- fixed frame: `world`.
- model visible: embed/model pipeline OK by runtime inputs and RViz successful embed; final visual inspection should be done in the GUI session.
- notes: `move_group` still segfaults on Ctrl-C shutdown in this environment; this is separate from RViz embedding. `robot_gui` now catches `KeyboardInterrupt` in its Qt spin timer to reduce forced shutdown.
