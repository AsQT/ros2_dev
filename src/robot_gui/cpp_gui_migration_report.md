# C++ GUI Migration Report

## Runtime status

- Package `robot_gui` is now the C++/Qt GUI package.
- Legacy Python GUI was moved out of the active package path and ignored by colcon in the previous migration step.
- `robot_gui_node` starts and stops cleanly on Ctrl-C in standalone GUI mode.
- Native RViz is created with `rviz_common::VisualizationFrame` and is embedded as a child widget inside `embeddedRvizWidget`.

## Launch behavior

- `robot_gui/launch/robot_gui.launch.py` is standalone GUI only by default.
- Default `embed_rviz=false` keeps the GUI usable without requiring the full MoveIt stack.
- Optional arguments are available: `embed_rviz`, `initial_page`, `rviz_config_package`, and `rviz_config_relative_path`.

## Fixes applied

- Removed launch-level node rename that caused duplicate global node remapping.
- Added ROS parameter override support with `automatically_declare_parameters_from_overrides(true)`.
- Made `RobotGuiNode` read existing launch-provided parameters before declaring defaults.
- Added graph checks for `robot_state_publisher`, `move_group`, and `/joint_states`.
- Forced RViz `VisualizationFrame` to behave as a Qt child widget, preventing a separate top-level RViz window.
- Added RViz config/fixed-frame logging.

## Visual verification

- Real GUI display was used: `DISPLAY=:0`, X11 session.
- Embedded GUI screenshot: `/tmp/robot_gui_moveit_gui_main2.png`.
- Result: embedded RViz appears inside the MAIN page.
- Robot model visible: OK.
- Grid visible: OK.
- Top-level RViz window leak: fixed. `xwininfo` shows only the main GUI window and an internal child window for RViz.

## Remaining warnings

- RViz/MoveIt still logs `class_loader` namespace collision for `InteractiveMarkerDisplay`.
- The same warning appears in standalone `rviz2 -d moveit.rviz`, so this is not specific to the embedded GUI.
- `/recognize_objects not available` appears in both embedded and standalone RViz; this is a MoveIt perception action warning, not a GUI crash.
- `move_group` can segfault during Ctrl-C shutdown; `robot_gui_node` itself exits cleanly.
