# Task Action GUI Update Report

## Files changed

- `robot_gui/include/robot_gui/task_action_controller.hpp`
- `robot_gui/src/task_action_controller.cpp`
- `robot_gui/include/robot_gui/main_window.hpp`
- `robot_gui/src/main_window.cpp`
- `robot_gui/CMakeLists.txt`
- `robot_gui/package.xml`

## GUI action mappings

| Tab | Button | Action | execute |
|---|---|---|---|
| Move Pose | `btnStartTask` (`Plan`) | `/move_to_pose` or `/move_to_pose_cartesian` | `false` |
| Move Pose | `btnResetTask` (`Execute`) | `/move_to_pose` or `/move_to_pose_cartesian` | `true` |
| Move Pose RL | `btnRLPlan`, `btnRLExecute` | no backend mapping | log only |
| Gripper | `btnTaskGripperOpen` | `/move_gripper` | `true` |
| Gripper | `btnTaskGripperClose` | `/move_gripper` | `true` |
| Gripper | `btnGripperRun` | `/move_gripper` | `true` |
| Pick Place | `btnPickPlacePlan` | `/pickplace` | `false` |
| Pick Place | `btnPickPlaceStart` | `/pickplace` | `true` |
| Pick Place Vision | `btnPickPlaceVisionPlan` | `/pickplace` with manual/object pose fallback | `false` |
| Pick Place Vision | `btnPickPlaceVisionStart` | `/pickplace` with manual/object pose fallback | `true` |
| Pick Place RL | `btnPickPlaceRLPlan` | `/drl_pickplace` | `false` |
| Pick Place RL | `btnPickPlaceRLStart` | `/drl_pickplace` | `true` |
| Check Board | `btnCheckBoardPlan` | `/move_checker_board` | `false` |
| Check Board | `btnCheckBoardStart` | `/move_checker_board` | `true` |
| Repeatability Test | `btnRepeatPlan` | `/repeatability_test` | `false` |
| Repeatability Test | `btnRepeatStart` | `/repeatability_test` | `true` |

Stop buttons currently log `cancel chưa implement` as required.

## Input and default handling

- Empty numeric fields use defaults from `codex.md`/`Call_action.md`.
- Invalid numeric input logs an error to the GUI log and does not send a goal.
- Velocity defaults to `0.5` for task tabs without a velocity field.
- Repeatability velocity defaults to `0.25`.
- Gripper defaults: open `0.048`, close `0.028`.
- Move Pose orientation uses Roll/Pitch/Yaw as degrees and converts to quaternion.
- If Move Pose orientation fields are empty, the quaternion remains the requested constant: `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}`.
- Pick/Place yaw fields are treated as degrees; empty yaw uses the same default orientation constant.
- DRL Pick Place and Repeatability use `{x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}` where `Call_action.md` specifies it.
- Repeatability axis selector maps `X/Y/Z` to `0/1/2`.

## GUI updates

- Added `TaskActionController` to isolate ROS action client logic from `MainWindow`.
- Added dynamic Plan buttons for tabs that did not have a Plan button in the `.ui`: Pick Place, Pick Place Vision, Pick Place RL, Check Board.
- Added a dynamic Repeatability axis selector when the `.ui` does not provide one.
- Reused `txtROS2Log` as the action log target when `txtActionLog` is not present.
- Feedback/result/status are appended via queued Qt invocations so ROS callbacks do not touch widgets directly from the executor thread.
- `chkMovePoseCartesian` is configured with visible text `Move Pose Cartesian`.

## Verification

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_gui robot_task_manager --symlink-install
```

Result: passed for both packages.

Action server launch:

```bash
source install/setup.bash
ros2 launch robot_task_manager task_servers.launch.py
ros2 action list | sort
```

Observed action names:

```text
/drl_pickplace
/move_action
/move_checker_board
/move_gripper
/move_to_pose
/move_to_pose_cartesian
/pickplace
/repeatability_test
```

Checked with `ros2 action info`:

- `/move_to_pose`
- `/move_to_pose_cartesian`
- `/move_gripper`
- `/pickplace`
- `/move_checker_board`
- `/repeatability_test`
- `/drl_pickplace`

GUI smoke test:

```bash
QT_QPA_PLATFORM=offscreen ros2 run robot_gui robot_gui_node --ros-args -p embed_rviz:=false
```

Result: GUI initialized without crashing; command exited by `timeout`.

## Notes

- DRL Pick Place server is present in `task_servers.launch.py`, but full DRL behavior still depends on the DRL planner stack at runtime.
- Move Pose RL is intentionally not sent to a backend action because no corresponding action mapping exists.
- Offscreen GUI testing cannot validate real mouse clicks visually; the button-to-action mapping is implemented in code and the target action servers were verified available.
