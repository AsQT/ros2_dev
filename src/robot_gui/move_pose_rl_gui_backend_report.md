# Move Pose RL GUI backend report

## 1. File da sua

- `robot_gui/include/robot_gui/task_action_controller.hpp`
- `robot_gui/src/task_action_controller.cpp`
- `robot_gui/task_action_gui_report.md`

## 2. Widget objectName da ket noi

- Tab: `tabMovePoseRL`
- Plan button: `btnRLPlan`
- Execute button: `btnRLExecute`
- Stop button: `btnRLStop` hien van log cancel chua implement
- Target position: `rlPosePositionX`, `rlPosePositionY`, `rlPosePositionZ`
- Target orientation RPY: `rlPoseOrientationRoll`, `rlPoseOrientationPitch`,
  `rlPoseOrientationYaw`
- Velocity scale: `txtVelocityScale`
- Action log: `txtActionLog`, fallback `txtROS2Log`

## 3. Action name GUI goi

GUI goi dung action:

```text
/move_pose_rl
```

Khong goi sang `/drl_pickplace`.

## 4. Goal gui khi Plan

Nut `btnRLPlan` gui:

```yaml
target_pose:
  position:
    x: <rlPosePositionX>
    y: <rlPosePositionY>
    z: <rlPosePositionZ>
  orientation:
    x: <converted/default qx>
    y: <converted/default qy>
    z: <converted/default qz>
    w: <converted/default qw>
velocity_scale: <txtVelocityScale or 0.5>
execute: false
```

Neu ca 3 field RPY trong tab Move Pose RL de trong, GUI dung quaternion mac
dinh `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}`.

## 5. Goal gui khi Execute

Nut `btnRLExecute` gui cung pose/velocity nhu Plan, nhung:

```yaml
execute: true
```

## 6. Backend duoc kiem tra truoc khi gui goal

GUI kiem tra action server:

```text
/move_pose_rl
```

GUI kiem tra service DRL cho ca Plan va Execute:

```text
/drl_unified_planner_node/set_parameters
/drl/clear_trajectory
/drl/plan
```

Khi Execute, GUI kiem tra them:

```text
/drl/execute_forward
/drl/get_execution_status
```

## 7. Xu ly khi thieu parameter service

Neu thieu `/drl_unified_planner_node/set_parameters`, GUI khong gui goal va log:

```text
[MovePoseRL] Backend not ready: missing /drl_unified_planner_node/set_parameters. Please launch DRL planner node before using move_pose_rl.
```

Neu thieu action server hoac service khac, GUI log:

```text
[MovePoseRL] Backend not ready: missing <service/action>.
```

`btnRLPlan` va `btnRLExecute` duoc disable khi dang preflight/action va enable lai
khi preflight fail, goal bi reject, hoac action co result.

## 8. Build

Da chay:

```bash
cd /home/minhquang/ros2_dev
source install/setup.bash
colcon build --packages-select robot_gui robot_task_manager --symlink-install
colcon build --packages-select robot_drl --symlink-install
```

Ket qua:

- `robot_gui`: thanh cong.
- `robot_task_manager`: thanh cong; co warning CMake cu ve
  `rosidl_target_interfaces()`.
- `robot_drl`: thanh cong.

## 9. Test CLI

Da chay stack mock:

```bash
ros2 launch robot_drl drl_mock_hw.launch.py \
  auto_plan_on_start:=false \
  auto_execute_after_plan:=false \
  manual_prompt_on_start:=false
ros2 launch robot_task_manager task_servers.launch.py
```

Ghi chu: `mock_environment_node` trong launch tren van fail do import
`Box` tu `robot_vision_pipeline.msg`, nhung `drl_unified_planner_node` van
khoi dong va cung cap cac service `/drl/*` can cho test manual.

Da kiem tra interface:

```bash
ros2 interface show robot_task_manager/action/MovePoseRl
```

Ket qua dung interface mong muon voi `target_pose`, `velocity_scale`, `execute`,
`success/message/failed_stage`, `current_stage/progress/current_pose`.

Da kiem tra action:

```bash
ros2 action list | rg 'move_pose_rl'
```

Ket qua:

```text
/move_pose_rl
```

Da kiem tra service:

```bash
ros2 service list | rg '(/drl|drl_unified_planner_node)'
```

Ket qua co toi thieu:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
```

Plan-only:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.375, y: 0.0, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: false}" \
  --feedback
```

Ket qua: succeeded, feedback den `done_plan_only`, message
`DRL plan succeeded; execution skipped because execute=false`.

Execute:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.375, y: 0.0, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: true}" \
  --feedback
```

Ket qua: succeeded, feedback den `done`, message
`MovePoseRl completed successfully`.

## 10. Test GUI

Da chay smoke test:

```bash
ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=1
```

Ket qua: GUI khoi dong, mapping widget load thanh cong, command duoc dung chu
dong bang `timeout`; khong crash.

Plan/Execute GUI: logic button da duoc ket noi trong
`TaskActionController::connectUiSignals()` va goal path da duoc xac nhan bang
CLI voi cung action `/move_pose_rl`. Chua thuc hien click GUI tu dong trong
luot nay.

Thieu backend: code preflight da ngan gui goal neu thieu action/service va log
ro service/action bi thieu. Chua thuc hien click GUI tu dong voi backend bi tat
trong luot nay.

## 11. Anh huong tab khac

Khong doi action name/goal cua cac tab khac. Cac helper doc input va send action
cu van giu nguyen; thay doi moi chi them `MovePoseRl` va thay mapping
`btnRLPlan`/`btnRLExecute` tu log-only sang `/move_pose_rl`.

