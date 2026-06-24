# MoveIt GUI Launch Report

## 1. File da sua

- `robot_moveit/launch/moveit_gui.launch.py`: added full MoveIt stack launch plus embedded `robot_gui_node`.
- `robot_bringup/launch/real.launch.py`: removed direct GUI launch; added `launch_gui` switch.
- `robot_gui/launch/robot_gui.launch.py`: standalone GUI launch, default `embed_rviz=false`, plus `initial_page` argument.
- `robot_gui/src/main_window.cpp`: setup/logging for native embedded RViz and required ROS graph inputs.
- `robot_gui/src/rviz_panel.cpp`: native `rviz_common::VisualizationFrame` loading `robot_moveit/config/moveit.rviz`.
- `robot_moveit/config/moveit.rviz`: includes a direct `RobotModel` display using `/robot_description`.

## 2. Launch separation

- `robot_gui.launch.py`: standalone GUI only by default; does not start MoveIt stack.
- `moveit_gui.launch.py`: starts `robot_state_publisher`, `joint_state_publisher`, `move_group`, optional controllers, then `robot_gui_node` with `embed_rviz=true`.
- `real.launch.py`: default `launch_gui=false`; uses normal `robot_moveit/launch/moveit.launch.py`. With `launch_gui=true`, uses `robot_moveit/launch/moveit_gui.launch.py`.

## 3. MoveIt stack

- `robot_state_publisher`: started by `moveit_gui.launch.py`.
- `move_group`: started by `moveit_gui.launch.py`.
- `joint_states`: provided by `joint_state_publisher` in mock test mode.
- `tf_static`: provided through `robot_state_publisher`.
- `robot_description`: available in embedded GUI test.
- `robot_description_semantic`: available in embedded GUI test because `moveit_config.to_dict()` is passed to `robot_gui_node`.

## 4. RViz native

- Config path: `/home/minhquang/ros2_dev/install/robot_moveit/share/robot_moveit/config/moveit.rviz`.
- API used: `rviz_common::VisualizationFrame`.
- Grid visible: OK.
- Robot model visible: OK.
- MotionPlanning visible/load status: OK in embedded GUI; group `arm` loaded and MoveGroup interface became ready.
- Fixed frame: `world`.
- Visual verification method: real X11 GUI screenshot with `xwd`; converted to PNG at `/tmp/robot_gui_moveit_gui_main2.png`.

## 5. Comparison with standalone RViz

- Standalone RViz result: `rviz2 -d moveit.rviz` shows the robot model and grid.
- Embedded RViz result: robot model and grid show inside the GUI MAIN page; MotionPlanning loads with group `arm`.
- Difference: raw standalone `rviz2 -d moveit.rviz` does not receive MoveIt params, so MotionPlanning later reports missing `robot_description_semantic`. Embedded GUI receives the MoveIt params from `moveit_gui.launch.py`, so MotionPlanning loads correctly.
- Common warnings: both standalone and embedded RViz show the `InteractiveMarkerDisplay` class_loader collision and `/recognize_objects not available`.

## 6. Remaining issues

- `move_group` can segfault on Ctrl-C shutdown. The GUI process exits cleanly.
- The class_loader warning remains present in both standalone and embedded RViz.
- The `/recognize_objects` warning remains present unless a perception action server is launched or the MoveIt config disables that capability.

## 7. Commands verified

- `colcon build --packages-select robot_gui robot_moveit --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`: OK.
- `ros2 launch robot_moveit moveit_gui.launch.py --show-args`: OK.
- `ros2 launch robot_bringup real.launch.py --show-args`: OK, default `launch_gui=false`.
- `ros2 launch robot_gui robot_gui.launch.py --show-args`: OK, default `embed_rviz=false`.
- `ros2 launch robot_moveit moveit_gui.launch.py`: OK on real X11 display.
