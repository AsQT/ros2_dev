# UI Layout Usage Report

## 1. Constraint

- robot_gui.ui modified: no
- Layout source: `robot_gui/ui/robot_gui.ui`

## 2. Old issue

- Code was creating layout manually: partially yes
- Files containing manual layout:
  - `robot_gui/src/rviz_panel.cpp` previously created an extra wrapper layout and placeholder label.
  - `robot_gui/src/main_window.cpp` keeps only the allowed small `QVBoxLayout` fallback for `embeddedRvizWidget`.

## 3. Fix

- MainWindow loads .ui using: UIC generated header `ui_robot_gui.h` and `ui_->setupUi(this)`.
- CMake AUTOUIC: enabled with `CMAKE_AUTOUIC ON`.
- UI file installed: yes, `install/robot_gui/share/robot_gui/ui/robot_gui.ui`.
- Logo.png installed: yes, `install/robot_gui/share/robot_gui/ui/Logo.png`.
- Logo path handling: `MainWindow` temporarily sets current directory to the installed `share/robot_gui/ui` before `setupUi()` because the `.ui` uses `<pixmap>Logo.png</pixmap>`.

## 4. Widgets bound from .ui

- stackedWidget: `ui_->stackedWidget_MainPages`.
- embeddedRvizWidget: `ui_->embeddedRvizWidget`.
- labelRvizPlaceholder: `ui_->labelRvizPlaceholder`.
- robot enable button: `btnRobotEnable` from the `.ui`.
- axis widgets: `btnAxis{1..6}*`, `txtAxis{1..6}*`, and `ledAxis{1..6}*` are resolved by objectName from the `.ui`.
- tab buttons: `btnHome`, `btnMain`, `btnRobot`, `btnVision`, `btnSetting`, `btnLog` from the `.ui`.

## 5. RViz integration

- RViz added into existing widget: yes, `rviz_common::VisualizationFrame` is parented to and added into `embeddedRvizWidget`.
- Created new RViz container manually: no.
- The only remaining manual layout creation is the allowed fallback `QVBoxLayout` on `embeddedRvizWidget` if the `.ui` does not already provide one at runtime.

## 6. Test result

- Build: OK with `colcon build --packages-select robot_gui --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`.
- Standalone GUI: OK with `ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false`.
- Logo visible: OK after resolving the `.ui` relative pixmap path from `share/robot_gui/ui`.
- UI Designer change reflected: OK by build path: `ui/robot_gui.ui` is in target sources and AUTOUIC regenerates `ui_robot_gui.h`; no manual main layout is used. No temporary `.ui` edit was committed because this task forbids modifying `robot_gui.ui`.
- MoveIt GUI: OK with `ros2 launch robot_moveit moveit_gui.launch.py`; robot model appears in the existing MAIN page RViz area.
- Remaining issues:
  - The current `.ui` stylesheet logs parse warnings for `btnAxis1Enable` through `btnAxis6Enable`; the `.ui` was not modified in this task.
  - RViz/MoveIt still logs the known `InteractiveMarkerDisplay` class_loader warning and `/recognize_objects not available`.
  - `move_group` can segfault during Ctrl-C shutdown; `robot_gui_node` exits cleanly.
