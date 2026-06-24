# robot_gui Runtime Fix Report

## Summary

Đã sửa các lỗi runtime chính của package `robot_gui` C++/Qt native RViz theo `codex.md`.

## Runtime Error Fix

### robot_description missing

- Cause: `robot_gui.launch.py` là launch GUI standalone, trước đây mặc định bật embedded RViz nhưng không launch `robot_state_publisher`, `joint_state_publisher`, `move_group`, `/joint_states`, `/tf`, `/tf_static`.
- Fix:
  - `robot_gui.launch.py` đổi mặc định `embed_rviz` từ `true` sang `false`.
  - Khi người dùng bật `embed_rviz:=true` nhưng graph chưa có provider robot model, GUI log warning rõ:
    `RViz enabled but robot_description provider was not detected...`
  - Tạo `robot_gui/launch/embedded_rviz_cpp_test.launch.py` để chạy đủ robot model test.
- Test launch:
  - `ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false`: OK.
  - `ros2 launch robot_gui robot_gui.launch.py embed_rviz:=true`: GUI không crash trong headless/offscreen; RViz init failure do GLX được catch và hiển thị/log.
  - `ros2 launch robot_gui embedded_rviz_cpp_test.launch.py`: launch được `robot_state_publisher`, `joint_state_publisher`, `move_group`, `robot_gui`.

### Duplicate node name

- Cause: `robot_gui.launch.py` set `name="robot_gui"`, tạo global remap `__node:=robot_gui` có thể ảnh hưởng node nội bộ RViz/MoveIt trong cùng process.
- Fix:
  - Bỏ `name="robot_gui"` khỏi launch `Node`.
  - Executable tự tạo main node tên `robot_gui`.
  - RViz node nội bộ vẫn dùng tên riêng `robot_gui_rviz`.

### class_loader namespace collision

- Cause: executable link trực tiếp `rviz_default_plugins`, có thể gây plugin factory namespace collision.
- CMake change:
  - Bỏ `find_package(rviz_default_plugins REQUIRED)`.
  - Bỏ `rviz_default_plugins` khỏi `ament_target_dependencies(robot_gui_node ...)`.
  - Giữ `rviz_default_plugins` là runtime dependency trong `package.xml` để pluginlib/RViz tự discover plugin.

### Stylesheet parse errors

- Widgets:
  - `btnAxis1Enable`
  - `btnAxis2Enable`
  - `btnAxis3Enable`
  - `btnAxis4Enable`
  - `btnAxis5Enable`
  - `btnAxis6Enable`
- Cause: QSS cũ có selector không hợp lệ `QPushButton# { ... }`.
- Fix:
  - Đổi thành selector hợp lệ `QPushButton#btnAxisNEnable:hover` và `QPushButton#btnAxisNEnable:pressed`.
- Result:
  - Test standalone sau build không còn log `Could not parse stylesheet`.

### Shutdown

- Previous:
  - Ctrl-C launch không thoát, bị escalate SIGTERM/SIGKILL.
- Fixed:
  - Thêm `rclcpp::on_shutdown()` để gọi `QApplication::quit()`.
  - Kết nối `QCoreApplication::aboutToQuit` để `executor.cancel()`.
  - Destroy `MainWindow`/RViz panel trước khi `rclcpp::shutdown()`.
- Ctrl-C result:
  - `robot_gui.launch.py embed_rviz:=false`: process finished cleanly.
  - `robot_gui.launch.py embed_rviz:=true` trong offscreen/headless: process finished cleanly sau SIGINT.
  - `embedded_rviz_cpp_test.launch.py`: `robot_gui_node`, `robot_state_publisher`, `joint_state_publisher` thoát sạch; `move_group` có segfault khi shutdown trong test headless, đây là process ngoài `robot_gui_node`.

### RViz/OpenGL headless handling

- Cause: `QT_QPA_PLATFORM=offscreen` không cung cấp parent GLX window hợp lệ cho RViz/OGRE.
- Fix:
  - Bọc `VisualizationFrame::initialize()` bằng `try/catch`.
  - Nếu RViz native init fail, GUI giữ nguyên và hiển thị/log lỗi thay vì crash.

### Final tests

- Build:
  - `colcon build --packages-select robot_gui robot_bringup robot_moveit robot_description robot_hardware_interface robot_control --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
  - Result: OK. `robot_hardware_interface` còn warning unused variable trong test, không liên quan GUI.
- `robot_gui.launch.py embed_rviz=false`:
  - Result: OK, no stylesheet parse error, Ctrl-C clean.
- `robot_gui.launch.py embed_rviz=true`:
  - Result: OK trong headless/offscreen theo nghĩa GUI không crash; warning thiếu robot_description rõ nếu chạy standalone; RViz GLX lỗi được catch.
- `embedded_rviz_cpp_test.launch.py`:
  - Result: model stack OK.
  - `/robot_state_publisher` có `robot_description`.
  - `/move_group` có `robot_description_semantic`.
  - `/joint_states` có `joint_1..joint_6`, `joint_gl`, `joint_gr`.
  - `/tf_static` có `world -> base_link`.
  - Visual confirmation embedded RViz model visible chưa verify được trong môi trường headless/offscreen.
- `real.launch.py`:
  - Arguments parse OK từ lần kiểm tra trước; chưa chạy real hardware/TCP trong turn này.

