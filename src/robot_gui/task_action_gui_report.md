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
| Move Pose RL | `btnRLPlan` | `/move_pose_rl` | `false` |
| Move Pose RL | `btnRLExecute` | `/move_pose_rl` | `true` |
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
- Move Pose RL reads `rlPosePositionX/Y/Z`, converts `rlPoseOrientationRoll/Pitch/Yaw` from degrees to quaternion, and uses `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` when all orientation fields are empty.
- Move Pose RL reads velocity from `txtVelocityScale`, defaulting to `0.5`.
- Pick/Place yaw fields are treated as degrees; empty yaw uses the same default orientation constant.
- DRL Pick Place and Repeatability use `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` where `Call_action.md` specifies it.
- Repeatability axis selector maps `X/Y/Z` to `0/1/2`.

## GUI updates

- Added `TaskActionController` to isolate ROS action client logic from `MainWindow`.
- Added dynamic Plan buttons for tabs that did not have a Plan button in the `.ui`: Pick Place, Pick Place Vision, Pick Place RL, Check Board.
- Added a dynamic Repeatability axis selector when the `.ui` does not provide one.
- Created a runtime `QPlainTextEdit#txtActionLog` in the lower `ShortLogArea` when the `.ui` does not provide one.
- Replaced the old visible action-log target `QLabel#txtMainLog` for action feedback display; `txtMainLog` had a 52pt font in the `.ui` and was the source of the oversized action feedback text.
- Feedback/result/status are appended via queued Qt invocations so ROS callbacks do not touch widgets directly from the executor thread.
- `chkMovePoseCartesian` is configured with visible text `Move Pose Cartesian`.
- Move Pose RL preflights `/move_pose_rl` plus DRL services before sending a goal and disables `btnRLPlan`/`btnRLExecute` while the action is active.

## Fix action log font

- Nguyên nhân font log vẫn to:
  - `TaskActionController::appendActionLog()` appends action feedback/result lines such as `feedback stage=...`.
  - The code looked for `QTextEdit#txtActionLog`, but the `.ui` file does not define `txtActionLog`.
  - It then fell back to `QTextEdit#txtROS2Log` and also set the latest line into `QLabel#txtMainLog`.
  - `QLabel#txtMainLog` is the visible lower log area in `ShortLogArea` and had `pointsize=52` in `robot_gui.ui`, so action feedback appeared huge and clipped.
- Widget thực tế hiển thị log trước sửa:
  - `objectName`: `txtMainLog`
  - class: `QLabel`
  - parent area: `ShortLogArea`
  - `.ui` font: Arial 52pt
- Widget action log sau sửa:
  - `objectName`: `txtActionLog`
  - class: `QPlainTextEdit`
  - created at runtime in `TaskActionController::configureUi()` after `setupUi(this)`
  - parent area: `ShortLogArea`
  - geometry copied from old `txtMainLog`
  - old `txtMainLog` is hidden for action feedback display
- File đã sửa:
  - `robot_gui/src/task_action_controller.cpp`
  - `robot_gui/task_action_gui_report.md`
- Font sau sửa:
  - `DejaVu Sans Mono`, `8pt`
  - selector: `QPlainTextEdit#txtActionLog`
  - read-only, no rich text, action lines appended with `appendPlainText()`
- Kết quả test:
  - `colcon build --packages-select robot_gui --symlink-install`: passed.
  - `ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=0`: GUI initialized without crashing and was stopped by `timeout`.
  - The action log path now uses `QPlainTextEdit#txtActionLog`, so feedback/result lines are displayed with 8pt mono text instead of the old 52pt `QLabel#txtMainLog`.

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
/move_pose_rl
/move_to_pose
/move_to_pose_cartesian
/pickplace
/repeatability_test
```

Checked with `ros2 action info`:

- `/move_to_pose`
- `/move_to_pose_cartesian`
- `/move_pose_rl`
- `/move_gripper`
- `/pickplace`
- `/move_checker_board`
- `/repeatability_test`
- `/drl_pickplace`

GUI smoke test:

```bash
source install/setup.bash
ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=0
```

Result: GUI initialized without crashing on `DISPLAY=:0`; command exited by `timeout`.

## Notes

- DRL Pick Place and Move Pose RL both depend on the DRL planner stack at runtime.
- Move Pose RL reports the missing backend service/action explicitly and does not send a goal when preflight fails.
- The environment did not have an image conversion tool (`convert`, `magick`, `xwdtopnm`, or `pnmtopng`) available for producing a PNG screenshot from X11. The font fix is applied directly to the runtime widget that receives action feedback.
