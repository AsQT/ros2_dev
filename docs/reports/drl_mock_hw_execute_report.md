# Bao cao kiem tra `drl_mock_hw.launch.py`

Ngay thuc hien: 2026-06-10

## Muc tieu

Kiem tra launch:

```bash
ros2 launch robot_drl drl_mock_hw.launch.py
```

voi mock hardware, MoveIt, task executor va DRL planner. Trong qua trinh kiem
tra co bat them execute:

```bash
ros2 launch robot_drl drl_mock_hw.launch.py \
  auto_plan_on_start:=true \
  manual_prompt_on_start:=false \
  auto_execute_after_plan:=true
```

## Moi truong

- ROS 2: Jazzy
- Python/RL venv: `/home/minhquang/venvs/ros_rl`
- Workspace: `/home/minhquang/ros2_dev`

## Cac thay doi da thuc hien

### 1. `robot_drl/launch/drl_mock_hw.launch.py`

Sua cach truyen controller config vao `ros2_control_node`.

Truoc do launch gop noi dung YAML vao list parameter dict. `controller_manager`
khong doc dung namespace `controller_manager.ros__parameters`, dan den loi:

```text
The 'type' param was not defined for 'joint_state_broadcaster'
The 'type' param was not defined for 'arm_controller'
The 'type' param was not defined for 'gripper_controller'
```

Sau sua:

```python
ros2_control_params = [
    *_make_params(moveit_config.robot_description, use_sim_time_bool),
    controllers_yaml,
]
```

Ket qua:

```text
Loaded hardware 'robot' from plugin 'mock_components/GenericSystem'
Configured and activated joint_state_broadcaster
Configured and activated arm_controller
Configured and activated gripper_controller
```

Them tham so de execute dung fixed TCP-down orientation thay vi lay current
orientation tu TF:

```python
"use_current_tcp_orientation_for_execution": False,
```

Ly do: log truoc khi sua cho thay DRL execute dang gui orientation hien tai:

```text
using current tcp_link orientation quat=(0.500000, 0.500000, 0.500000, 0.500000)
```

Sau khi sua, pose sequence gui sang task executor dung quaternion TCP-down theo
convention hien co:

```text
FIRST pose[0] ... quat=(0.000000, 1.000000, 0.000000, 0.000000)
LAST  pose[20] ... quat=(0.000000, 1.000000, 0.000000, 0.000000)
```

Bo sung `robot_description_kinematics` vao `task_executor_node` de MoveIt
client co KDL plugin:

```python
**moveit_config.robot_description_kinematics,
```

Truoc do task executor canh bao:

```text
No kinematics plugins defined. Fill and load kinematics.yaml!
```

Sau khi them, log da load KDL:

```text
task_executor_node.moveit.kinematics.kdl_kinematics_plugin:
Joint weights for group 'arm': 1 1 1 1 1 1
```

### 2. `robot_drl/robot_drl/drl_planner_node_base.py`

Doi quaternion execute mac dinh tu `xyzw=[1,0,0,0]` sang `xyzw=[0,1,0,0]`.
Test service rieng le cho thay waypoint tiep theo plan duoc voi quaternion Y,
con quaternion X fail:

```text
qx=(1,0,0,0): success=False
qy=(0,1,0,0): success=True
```

### 3. `robot_task_executor`

Doi fixed Cartesian orientation/fallback orientation trong task executor sang
cung quy uoc TCP-down:

```text
roll=0, pitch=pi, yaw=0
quat=(0.000000, 1.000000, 0.000000, 0.000000)
```

### 4. `robot_moveit/config/kinematics.yaml`

Tang timeout KDL IK tu `0.005s` len `0.05s` va them attempts:

```yaml
kinematics_solver_timeout: 0.050000000000000003
kinematics_solver_attempts: 5
```

### 5. `robot_description/CMakeLists.txt`

Bo dependency thua:

```cmake
find_package(urdf_tutorial REQUIRED)
```

Ly do: package `urdf_tutorial` khong duoc dung trong package nay va lam
`robot_description` fail build khi may chua cai tutorial package.

### 6. Cac thay doi phu

- `robot_control/config/initial_positions.yaml`: chi thay doi newline cuoi file,
  khong thay doi gia tri joint.
- `robot_description/urdf/ros2_control.xacro`: chi thay doi newline cuoi file,
  khong thay doi logic.
- `robot_vision_pipeline/robot_vision_pipeline/__pycache__/__init__.cpython-312.pyc`:
  file generated bi Python cham vao khi import/build.

## Ket qua build

Da build thanh cong cac package lien quan:

```bash
source /opt/ros/jazzy/setup.bash
source /home/minhquang/venvs/ros_rl/bin/activate
source /home/minhquang/ros2_dev/install/setup.bash

colcon build --symlink-install --packages-select robot_drl
colcon build --symlink-install --packages-select robot_task_executor
colcon build --symlink-install --packages-select robot_description robot_control robot_moveit
```

## Ket qua launch va plan

Launch mock hardware da len du cac thanh phan:

```text
robot_state_publisher started
ros2_control_node started
mock_components/GenericSystem loaded
joint_state_broadcaster active
arm_controller active
gripper_controller active
move_group ready
task_executor_node ready
drl_unified_planner_node started
RViz started
```

DRL planner tao duoc trajectory:

```text
Trajectory planning complete
converged=True
forward_waypoints=21
backward_waypoints=21
target_base=(0.5750, 0.0500, 0.1200)
```

## Kiem tra lai khi user bao van khong chay duoc

Lan kiem tra lai phat hien cac node con cua lan launch truoc van con chay nen.
Trang thai luc do con cac process:

```text
ros2_control_node
move_group
task_executor_node
mock_environment_node
drl_unified_planner_node
rviz2
```

Khi cac node nay con song, viec launch lai co the bi trung node, service,
controller manager hoac controller action, dan den cam giac "khong chay duoc".

Da don cac PID cu, sau do chay lai command mac dinh:

```bash
source /opt/ros/jazzy/setup.bash
source /home/minhquang/venvs/ros_rl/bin/activate
source /home/minhquang/ros2_dev/install/setup.bash

ros2 launch robot_drl drl_mock_hw.launch.py
```

Ket qua recheck:

```text
Loaded hardware 'robot' from plugin 'mock_components/GenericSystem'
Configured and activated joint_state_broadcaster
Configured and activated arm_controller
Configured and activated gripper_controller
MoveGroup: You can start planning now!
Task executor node constructed.
Trajectory planning complete
Published trajectories | forward=21wp ... backward=21wp ...
RViz started
```

Ket luan recheck: launch mac dinh chay duoc va plan duoc. Neu khong chay duoc,
viec dau tien can kiem tra la node cu con treo:

```bash
pgrep -af 'ros2 launch|ros2_control_node|move_group|task_executor_node|drl_unified_planner_node|mock_environment_node|rviz2'
```

Neu con process cu, dung launch cu bang Ctrl-C neu terminal van con. Neu terminal
da mat, kill cac PID con treo roi launch lai.

## Ket qua execute

### Truoc khi ep TCP-down

DRL lay current TCP orientation:

```text
quat=(0.500000, 0.500000, 0.500000, 0.500000)
```

`/move_cartesian_pose_sequence` nhan 21 pose. Cartesian interpolation chi dat:

```text
computeCartesianPath fraction=0.285714
```

Task executor fallback sang PTP va execute duoc 6 waypoint:

```text
completed waypoint 1/21
...
completed waypoint 6/21
```

Sau do fail:

```text
plan failed at waypoint 6/21
status=FAILED_FORWARD
```

### Sau khi ep TCP-down lan 1

Pose gui sang task executor da dung fixed TCP-down:

```text
quat=(1.000000, 0.000000, 0.000000, 0.000000)
```

Tuy nhien execute van chua hoan tat. Ket qua thu:

```text
computeCartesianPath fraction=0.000000
PTP fallback completed waypoint 1/21
plan failed at waypoint 1/21
status=FAILED_FORWARD
```

Mot lan thu voi mock initial joint pose khac cho thay current TCP bi lech xa
hon waypoint dau, nen thay doi do da duoc revert.

### Sau khi doi TCP-down sang quaternion Y

Ket qua test rieng le cho waypoint 2:

```text
pose=(0.5311, 0.0092, 0.3053)
qx=(1,0,0,0): success=False
qy=(0,1,0,0): success=True
```

Sau khi doi DRL va task executor sang `quat=(0,1,0,0)`, auto execute khong con
fail ngay o waypoint 2. Ket qua:

```text
completed waypoint 1/21
completed waypoint 2/21
completed waypoint 3/21
completed waypoint 4/21
plan failed at waypoint 4/21
PTP fallback failed at fraction: 0.190476
status=FAILED_FORWARD
```

Waypoint fail luc nay o gan:

```text
current tcp_link xyz=(0.5440, 0.0267, 0.2858)
next waypoint xyz=(0.5500, 0.0346, 0.2759)
```

Thu lai pose nay voi nhieu quaternion tu current state van khong tim duoc
nghiem on dinh:

```text
qx=(1,0,0,0): fraction=0.255556, fallback failed
qy=(0,1,0,0): fraction=0.000000, fallback failed
qz=(0,0,1,0): fraction=0.000000, fallback failed
```

## Ket luan

Launch va controller stack da duoc sua de chay dung:

- mock hardware load OK
- controller spawn OK
- MoveIt ready
- DRL planner plan OK
- service execute duoc goi dung
- arm_controller thuc su nhan va execute trajectory

Van con blocker o phan execute trajectory:

1. Launch, controller, MoveIt, DRL plan va service call da chay dung.
2. Orientation TCP-down dung voi model hien tai la `xyzw=[0,1,0,0]`, khong phai
   `xyzw=[1,0,0,0]`.
3. Full execute cua trajectory mac dinh van fail o doan di xuong thap
   (`z` khoang `0.276m` va tiep tuc den target `0.120m`). MoveIt/OMPL bao
   `Unable to sample any valid states for goal tree`.
4. De execute het trajectory can dieu chinh them mot trong cac phan sau:
   start/target pose cua DRL, workspace z toi thieu, hoac chien luoc IK/trajectory
   cho vung thap gan vat.
3. MoveIt phai plan tu current TCP sang waypoint dau cua DRL trajectory, nen de
   fail IK/OMPL.

Huong xu ly tiep theo:

- Tim joint state IK hop le cho `calibrated_start_tcp_base=[0.5241, 0.0, 0.315]`
  voi TCP-down, roi dung joint state do lam initial state cua mock hardware.
- Hoac cho DRL lay start TCP tu TF/current robot state thay vi dung fixed
  calibrated start.
- Sau khi current state va waypoint dau khop nhau, chay lai
  `auto_execute_after_plan:=true` de kiem tra toan bo 21 waypoint.
