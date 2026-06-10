# robot_gui (ament_python)

Tabs:
- Hardware: RS-485 services + joint_states readout
- TF: live pose base_link -> tcp_lik (0.5s)
- Planning: MoveIt plan / execute

## Build
```bash
cd ~/ros2_arm_ws/src
# copy this folder (robot_gui) here
cd ~/ros2_arm_ws
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 run robot_gui robot_gui
# or
ros2 launch robot_gui robot_gui.launch.py
```

## Notes
- Hardware tab depends on `robot_hardware_interface` custom services.
- TF/Planning requires MoveIt up (move_group + /query_planner_interface + /display_planned_path).
