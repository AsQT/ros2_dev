# RepeatabilityTest axis Y all-pose orientation report

## Muc tieu

Sua action `/repeatability_test` de khi `axis == AXIS_Y`, toan bo pose lam viec trong flow do lap lai dung cung orientation da xoay tool +90 deg quanh truc Z cua tool:

- `working_retract_pose`
- `meas_pose`
- `working_disturb_pose_1`

Action interface, action name `/repeatability_test`, action type va field goal/result/feedback khong doi.

## Nguyen nhan loi cu

Logic truoc do chi xoay orientation cua `meas_pose` khi chay truc Y, trong khi `retract_pose` va `disturb_pose_1` van giu orientation goc.

Do flow co cac doan Cartesian:

```text
retract_pose -> meas_pose -> retract_pose
```

nen voi truc Y, Cartesian path phai vua tinh tien theo Y vua doi orientation 90 deg trong doan ngan. Dieu nay lam `computeCartesianPath()` de fail.

## Thay doi da thuc hien

File chinh:

- `robot_task_manager/src/repeatability_test_server.cpp`

Noi dung sua:

- Them helper `rotate_tool_yaw(input, yaw_offset_rad)` dung `tf2::Quaternion`.
- Dung parameter `axis_y_tool_yaw_offset_rad`, default `1.5707963267948966`.
- Tao cac pose lam viec:
  - `working_retract_pose = retract_pose`
  - `working_disturb_pose_1 = disturb_pose_1`
  - `meas_pose = working_retract_pose`
- Khi `axis == AXIS_Y`, xoay orientation tu `retract_pose.orientation`, sau do gan cung orientation nay cho:
  - `working_retract_pose.orientation`
  - `working_disturb_pose_1.orientation`
  - `meas_pose.orientation`
- Sua lai logic tinh vi tri:
  - `axis=0`: `meas_pose.position.x = working_retract_pose.position.x + meas_offset`
  - `axis=1`: `meas_pose.position.y = working_retract_pose.position.y + meas_offset`
  - `axis=2`: `meas_pose.position.z = working_retract_pose.position.z + meas_offset`
- Toan bo flow phia sau dung pose lam viec:
  - MoveToPose toi `working_retract_pose`
  - Cartesian toi `meas_pose`
  - Cartesian ve `working_retract_pose`
  - MoveToPose toi `working_disturb_pose_1`
  - MoveToPose ve `working_retract_pose`
- Them log cho `working_retract_pose`, `meas_pose`, `working_disturb_pose_1`.

File build:

- `robot_task_manager/CMakeLists.txt`

Noi dung sua:

- Them dependency `tf2` va `tf2_geometry_msgs` cho target `repeatability_test_server`.

File tai lieu:

- `robot_task_manager/Call_action.md`

Noi dung sua:

- Cap nhat sequence cua `/repeatability_test` de dung `working_retract_pose` va `working_disturb_pose_1`.
- Ghi ro `axis=1` xoay orientation cua ca 3 pose lam viec bang `axis_y_tool_yaw_offset_rad`.

## Hanh vi moi theo tung truc

`axis=0`:

- `working_retract_pose.orientation = retract_pose.orientation`
- `working_disturb_pose_1.orientation = disturb_pose_1.orientation`
- `meas_pose.orientation = working_retract_pose.orientation`
- Chi thay doi `meas_pose.position.x`.

`axis=1`:

- `rotated_orientation = rotate_tool_yaw(retract_pose.orientation, axis_y_tool_yaw_offset_rad_)`
- `working_retract_pose.orientation = rotated_orientation`
- `meas_pose.orientation = rotated_orientation`
- `working_disturb_pose_1.orientation = rotated_orientation`
- Chi thay doi `meas_pose.position.y`.

`axis=2`:

- `working_retract_pose.orientation = retract_pose.orientation`
- `working_disturb_pose_1.orientation = disturb_pose_1.orientation`
- `meas_pose.orientation = working_retract_pose.orientation`
- Chi thay doi `meas_pose.position.z`.

## Build

Lenh:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager --symlink-install
```

Ket qua:

- Build thanh cong.
- Co warning deprecation san co tu `rosidl_target_interfaces()` trong `CMakeLists.txt`, khong lien quan toi thay doi orientation.

## Test

Launch dung de test:

```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

### Axis X plan-only

Goal:

- `axis: 0`
- `meas_offset: 0.02`
- `execute: false`

Ket qua:

- Action succeed.
- Log server moi:

```text
working_retract_pose position=(0.4000, 0.0000, 0.1800) orientation=(0.7071068, 0.7071068, 0.0000000, 0.0000000)
meas_pose            position=(0.4200, 0.0000, 0.1800) orientation=(0.7071068, 0.7071068, 0.0000000, 0.0000000)
working_disturb_pose_1 position=(0.3500, -0.0800, 0.1800) orientation=(0.7071068, 0.7071068, 0.0000000, 0.0000000)
```

X khong bi doi orientation.

### Axis Y plan-only

Goal:

- `axis: 1`
- `meas_offset: 0.02`
- `execute: false`

Ket qua:

- Server moi tinh dung pose lam viec:

```text
working_retract_pose position=(0.4000, 0.0000, 0.1800) orientation=(1.0000000, 0.0000000, 0.0000000, 0.0000000)
meas_pose            position=(0.4000, 0.0200, 0.1800) orientation=(1.0000000, 0.0000000, 0.0000000, 0.0000000)
working_disturb_pose_1 position=(0.3500, -0.0800, 0.1800) orientation=(1.0000000, 0.0000000, 0.0000000, 0.0000000)
```

- MoveToPose dau tien voi `working_retract_pose` fail trong MoveIt voi `result code = 6` tren moi truong test hien tai.
- Moi truong dang co hon mot action server `/repeatability_test`, nen `ros2 action send_goal` bao `There may be more than one action server`. Vi vay feedback co dong cu va dong moi bi lan nhau, nhung log tu process vua build xac nhan orientation cua 3 pose da dong nhat.

### Axis Y execute=true

Goal:

- `axis: 1`
- `meas_offset: 0.02`
- `execute: true`

Ket qua:

- Server moi van tinh dung 3 pose orientation `(1.0000000, 0.0000000, 0.0000000, 0.0000000)`.
- MoveToPose dau tien voi `working_retract_pose` fail trong planner voi `result code = 6`, nen chua toi buoc Cartesian.
- Do do chua xac nhan duoc robot chay that trong moi truong nay.

## Ket luan

Phan loi logic orientation da duoc sua: voi truc Y, Cartesian `working_retract_pose -> meas_pose -> working_retract_pose` khong con doi orientation giua hai dau segment nua.

Truc X/Z giu nguyen huong cu va chi doi position theo truc tuong ung.
