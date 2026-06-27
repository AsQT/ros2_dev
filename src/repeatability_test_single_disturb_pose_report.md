# RepeatabilityTest single disturb pose report

## 1. Files changed

- `robot_task_manager/action/RepeatabilityTest.action`
- `robot_task_manager/src/repeatability_test_server.cpp`
- `robot_task_manager/scripts/repeatability_test_client.py`
- `robot_gui/src/task_action_controller.cpp`
- `robot_gui/ui/robot_gui.ui`
- `robot_task_manager/Call_action.md`
- `robot_task_manager/README.md`
- `Call_action.md`

## 2. New RepeatabilityTest.action interface

```action
uint8 AXIS_X=0
uint8 AXIS_Y=1
uint8 AXIS_Z=2

geometry_msgs/PoseStamped retract_pose
geometry_msgs/PoseStamped disturb_pose_1
uint8 axis
float64 meas_offset
int32 repeat_count
float64 velocity_scale
bool execute
---
bool success
string message
int32 completed_count
---
int32 current_index
string current_step
```

## 3. Deleted field

- Removed `disturb_pose_2` from the action goal.
- Removed validation, server flow, client assignment, GUI read/send path, and docs/examples for the second disturb pose.

## 4. Flow change

Old flow:

1. MoveToPose to `retract_pose`.
2. Cartesian to `meas_pose`.
3. Wait at measurement pose.
4. Cartesian back to `retract_pose`.
5. MoveToPose to `disturb_pose_1`.
6. MoveToPose to `disturb_pose_2`.
7. MoveToPose back to `retract_pose`.

New flow:

1. MoveToPose to `retract_pose` with `fast_velocity_scale`.
2. Cartesian from `retract_pose` to `meas_pose` with `goal.velocity_scale`.
3. Wait `measurement_settle_time_s`.
4. Cartesian back to `retract_pose` with `fast_velocity_scale`.
5. MoveToPose to `disturb_pose_1` with `fast_velocity_scale`.
6. MoveToPose back to `retract_pose` with `fast_velocity_scale`.

## 5. Client and launch updates

- `repeatability_test_client.py` now sends only `retract_pose` and `disturb_pose_1`.
- `repeatability_test_client.launch.py` had no second-pose launch arguments; verified with `--show-args`.
- GUI action controller no longer reads `repeatDisturb2*` fields or sends the removed field.
- GUI `.ui` no longer shows the Disturb position 2 group.

## 6. Call_action.md update

- `robot_task_manager/Call_action.md` updated for the new goal shape, sequence, CLI examples, plan-only example, and default pose table.
- Root `Call_action.md` was also updated to keep workspace docs consistent.

## 7. Build result

Command:

```bash
cd ~/ros2_dev
source /opt/ros/*/setup.bash
colcon build --packages-select robot_task_manager robot_gui
```

Result:

```text
Summary: 2 packages finished [7min 19s]
```

## 8. Interface show result

Command:

```bash
source install/setup.bash
ros2 interface show robot_task_manager/action/RepeatabilityTest
```

Confirmed:

- `disturb_pose_1` is present.
- `disturb_pose_2` is absent.
- `AXIS_Z=2` is present.
- `execute` is present.

## 9. Grep confirmation

Command:

```bash
grep -R "disturb_pose_2" -n src/robot_task_manager src/robot_gui src/Call_action.md
```

Result:

```text
no output
```

Note: a stale Python `__pycache__` file initially contained the old string and was removed.

## 10. Test results

### execute=false

Command sent with the new goal shape and `execute: false`.

Observed:

- Goal was accepted by `/repeatability_test`.
- Feedback used the new plan-only flow:
  - `Waiting for sub action servers (plan-only)`
  - `Plan MoveToPose to retract_pose (execution skipped)`
- No removed field error occurred.
- No second disturb pose feedback appeared.
- Result aborted because child `/move_to_pose` did not return the goal response before the action result timeout:

```text
success: false
message: 'Initial MoveToPose to retract_pose failed: Timeout while sending MoveToPose goal'
completed_count: 0
```

This appears to be an environment/child-server readiness issue, not an interface mismatch.

### execute=true

Not run automatically. This goal can command real robot motion, so it should be run only with hardware/simulation confirmed safe by the operator.

### launch client

Command:

```bash
source install/setup.bash
ros2 launch robot_task_manager repeatability_test_client.launch.py --show-args
```

Result:

- Launch arguments are only `axis`, `repeat_count`, `meas_offset`, `velocity_scale`, `execute`, and `frame_id`.
- No second disturb-pose argument exists.

## 11. Other actions

- No other `.action` definitions were modified.
- `robot_task_manager` and dependent `robot_gui` both built successfully after the interface change.
