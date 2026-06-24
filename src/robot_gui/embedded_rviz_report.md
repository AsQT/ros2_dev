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

- Method: launch RViz bằng subprocess, lưu PID, tìm window theo đúng PID qua `wmctrl -lp`, `xdotool search --pid`, `xwininfo -root -tree`, `xprop`.
- Candidate windows quan sát được:
  - Top-level RViz: `0x2200106`, PID `44757`, class `rviz2`, title `/home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz - RViz`, size `1920x1011`
  - Content child RViz: `0x2200013`, PID `44757`, class `rviz2`, title empty, observed under `embeddedRvizWidget`
- Selected window id: ưu tiên content child cùng PID; fallback/adopt lại trong resize nếu content child xuất hiện muộn.
- Selected PID: PID của RViz do GUI tự launch.
- Selected title/class: class `rviz2`; title có thể empty với content child.
- Selected size: resize theo `embeddedRvizWidget`.

## 6. Embed result

- Container widget: `embeddedRvizWidget`
- Container winId quan sát được: `0x1a00021`
- Reparent result: RViz child `0x2200013` xuất hiện dưới container `0x1a00021` trong `xwininfo -root -tree`.
- Resize result: code đã resize qua `xdotool windowmove/windowsize` và có timer/adopt lại khi child window xuất hiện muộn.
- Lưu ý thực tế: default page của `.ui` là Robot (`currentIndex=2`), nên RViz page Main ban đầu có thể `UnMapped`; `update_tab_buttons()` gọi `refresh_embedded_rviz()` khi chuyển sang page Main.

## 7. Cleanup result

- RViz process stopped when GUI closed: implemented in `closeEvent()` via `RvizEmbedder.stop()`.
- Test bằng Ctrl-C launch: sau khi launch bị kill, không còn process `rviz2`/`robot_gui` sót lại.
- Lưu ý: Ctrl-C của launch hiện làm `robot_gui` bị SIGKILL sau timeout do `KeyboardInterrupt` trong Qt spin timer; đây là shutdown path của launch, không phải close GUI bằng nút đóng cửa sổ.

## 8. Test result

- Build: `colcon build --packages-select robot_gui robot_bringup robot_hardware_interface robot_moveit --event-handlers console_direct+` passed.
- Unit test: `pytest src/robot_gui/test` passed, `31 passed`.
- Test without external RViz: GUI tự launch RViz PID `44757`, command/config đúng, parent là `robot_gui`, không cần mở RViz sẵn.
- Test with external RViz already open: covered by unit test `test_rviz_window_selection_uses_launched_pid_and_largest_window`; external PID mismatch bị reject.
- Direct `real.launch.py`: partial pass. RViz launch/config/PID/window reparent được xác nhận; launch tổng thể vẫn fail do `Robot TCP connect failed: 192.168.2.50:5000: TCP connect timeout`, sau đó `ros2_control_node` abort và `move_group` segfault khi shutdown.
- Final status: RViz embedding logic đã chuyển sang PID-owned subprocess và có fallback/adopt content child; lỗi còn lại trong direct launch là hardware TCP/shutdown ngoài phạm vi RViz.
