# robot_drl_executor migration report

Ngay tao: 2026-06-27

## 1. Muc tieu migration

Tao package moi `robot_drl_executor` song song voi `robot_task_executor`, chi migrate phan executor MoveIt ma flow DRL/RL dang dung truc tiep. Package cu `robot_task_executor` duoc giu nguyen de rollback va cho cac flow legacy chua duoc xac nhan.

## 2. Cac report da doc

- `robot_task_manager/drl_backend_launch_audit_report.md`
- `robot_task_manager/move_pose_rl_action_report.md`
- `robot_task_manager/task_servers_launch_fix_report.md`
- `robot_task_manager/checker_board_current_flow_report.md`
- `robot_task_manager/checker_board_segmented_cartesian_report.md`
- `robot_task_manager/gohome_action_fix_report.md`
- `robot_task_manager/pickplace_start_state_fix_report.md`
- `robot_gui/move_pose_rl_gui_backend_report.md`

Audit truoc khi sua da tao tai:

- `robot_drl_executor_migration_audit_report.md`

## 3. Cac file da tao moi

- `robot_drl_executor/package.xml`
- `robot_drl_executor/CMakeLists.txt`
- `robot_drl_executor/src/robot_drl_executor_node.cpp`
- `robot_drl_executor/launch/robot_drl_executor.launch.py`
- `robot_drl_executor/config/robot_drl_executor.yaml`
- `robot_drl_executor/README.md`
- `robot_drl_executor/robot_drl_executor_migration_report.md`
- `robot_drl_executor_migration_audit_report.md`

## 4. Cac file da sua

- `robot_drl/package.xml`
- `robot_drl/README.md`
- `robot_drl/launch/drl_mock_hw.launch.py`
- `robot_drl/launch/drl_gazebo.launch.py`
- `robot_drl/launch/rl_sim_rviz.launch.py`
- `robot_bringup/package.xml`
- `robot_bringup/README.md`
- `robot_bringup/launch/drl_test.launch.py`
- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`
- `robot_bringup/launch/sim.launch.py`
- `robot_task_manager/package.xml`
- `robot_task_manager/README.md`

## 5. Cac service/action/topic da migrate

Service executor da migrate:

- `/move_cartesian_pose_sequence`
  - Type: `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`
  - Package moi: `robot_drl_executor`
  - Executable moi: `robot_drl_executor_node`
  - Node chinh: `/robot_drl_executor_node`

Action/DRL flow dung service nay:

- `/move_pose_rl`
- `/drl_pickplace`
- `robot_drl/drl_unified_planner_node` khi execute forward trajectory.

Khong migrate topic/service cua `robot_drl` vi chung van thuoc backend planner:

- `/drl/plan`
- `/drl/clear_trajectory`
- `/drl/execute_forward`
- `/drl/get_execution_status`
- `/drl/forward_trajectory_poses`
- `/drl_unified_planner_node/set_parameters`

## 6. Cac dependency da doi

- `robot_drl/package.xml`
  - Doi `exec_depend` tu `robot_task_executor` sang `robot_drl_executor`.
  - Giu `robot_task_executor_msgs` vi service type van nam trong package msg hien co.
- `robot_task_manager/package.xml`
  - Them `exec_depend` toi `robot_drl_executor`.
- `robot_bringup/package.xml`
  - Them `exec_depend` toi `robot_drl_executor`.
- Launch DRL/RL da doi include:
  - `robot_drl/launch/drl_mock_hw.launch.py`
  - `robot_drl/launch/drl_gazebo.launch.py`
  - `robot_drl/launch/rl_sim_rviz.launch.py`
  - `robot_bringup/launch/drl_test.launch.py`
  - `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`

## 7. Cac thanh phan co y chua migrate

Giu lai trong `robot_task_executor`:

- `/move_to_named_target`
- `/move_to_joint_target`
- `/move_to_pose_target`
- `/move_to_named_pose_target`
- `/move_to_cartesian_target`
- `/move_to_named_cartesian_target`
- `/move_cartesian_sequence`
- `/move_sequence`
- legacy waypoint YAML: `joint_waypoints.yaml`, `cartesian_points.yaml`, `pose_waypoints.yaml`

Ly do: trace code cho thay DRL execute path chi goi `/move_cartesian_pose_sequence`. Cac service named/joint/pose khong duoc migrate de tranh copy toan bo package cu.

## 8. Lenh build da chay

Build package lien quan:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/minhquang/ros2_dev
colcon build --symlink-install --packages-select robot_drl_executor robot_drl robot_task_manager robot_bringup
```

Full workspace build:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/minhquang/ros2_dev
colcon build --symlink-install
```

## 9. Ket qua build

- Build lien quan: thanh cong.
- Full workspace build lan cuoi: `Summary: 13 packages finished`.
- Warning con lai:
  - `robot_drl_executor`: deprecation warning cua `create_service(..., rmw_qos_profile_t, ...)` va MoveIt `computeCartesianPath(... jump_threshold ...)`.
  - Khong co compile error.

Trong qua trinh full build ban dau co loi symlink-install o cac build artifact cu:

- `build/robot_vision_pipeline_msgs/.../robot_vision_pipeline_msgs`
- `build/robot_task_executor_msgs/.../robot_task_executor_msgs`
- `build/robot_hardware_interface/.../robot_hardware_interface`
- `build/robot_vision_pipeline/.../robot_vision_pipeline`

Da xoa cac artifact generated trong `build/` roi build lai thanh cong.

## 10. Lenh test da chay

Mock hardware DRL stack:

```bash
source /opt/ros/jazzy/setup.bash
source /home/minhquang/ros2_dev/install/setup.bash
export ROS_DOMAIN_ID=80
ros2 launch robot_drl drl_mock_hw.launch.py \
  auto_plan_on_start:=false \
  auto_execute_after_plan:=false \
  manual_prompt_on_start:=false \
  input_mode:=manual
```

Task action servers dung backend ngoai:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

Service/action checks:

```bash
ros2 service type /move_cartesian_pose_sequence
ros2 service info /move_cartesian_pose_sequence
ros2 service info /drl/plan
ros2 action list | rg '(/move_pose_rl|/drl_pickplace)'
```

`/move_pose_rl execute=false`:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.375, y: 0.0, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.1, execute: false}" \
  --feedback
```

`/move_pose_rl execute=true`:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.375, y: 0.0, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.1, execute: true}" \
  --feedback
```

`task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args \
  -p task_name:=move_pose_rl \
  -p execute:=false \
  -p target_x:=0.375 \
  -p target_y:=0.0 \
  -p target_z:=0.25
```

`/drl_pickplace execute=false` smoke:

```bash
timeout 90 ros2 action send_goal /drl_pickplace robot_task_manager/action/DrlPickPlace \
  "{target_pick: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.40, y: 0.05, z: 0.08}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, target_place: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.34, y: -0.10, z: 0.08}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, gripper_close_width_m: 0.028, execute: false}" \
  --feedback
```

## 11. Ket qua test

- `/move_cartesian_pose_sequence`
  - Type: `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`.
  - `Services count: 1`.
  - `Clients count: 1`.
- `/drl/plan`
  - Type: `std_srvs/srv/Trigger`.
  - `Services count: 1`.
  - `Clients count: 2`.
- Action list co:
  - `/move_pose_rl`
  - `/drl_pickplace`
- `/move_pose_rl execute=false`
  - `SUCCEEDED`
  - Message: `DRL plan succeeded; execution skipped because execute=false`
- `/move_pose_rl execute=true`
  - `SUCCEEDED`
  - Message: `MovePoseRl completed successfully`
  - Final TCP feedback gan target:
    - `x: 0.3749350673741437`
    - `y: 0.000025787237344226943`
    - `z: 0.250043398751905`
- `task_manager_client move_pose_rl execute=false`
  - `success: true`
  - Message: `DRL plan succeeded; execution skipped because execute=false`
- `/drl_pickplace execute=false`
  - Di qua cac stage:
    - `VALIDATE_GOAL_PLAN_ONLY`
    - `WAIT_FOR_SERVERS`
    - `PLAN_OPEN_GRIPPER_EXECUTION_SKIPPED`
    - `PLAN_TO_PRE_PICK_EXECUTION_SKIPPED`
    - `PLAN_DESCEND_TO_PICK_EXECUTION_SKIPPED`
    - `PLAN_CLOSE_GRIPPER_EXECUTION_SKIPPED`
    - `PLAN_LIFT_FROM_PICK_EXECUTION_SKIPPED`
    - `PLAN_TO_PLACE_EXECUTION_SKIPPED`
  - Backend DRL va action con da duoc goi.
  - Flow khong hoan tat result trong smoke test: DRL plan toi target place `(0.34, -0.10, 0.08)` khong converge (`dist=0.2412 m`) va server bat dau retry. CLI bi `timeout 90` cat; sau do launch duoc dung bang Ctrl-C.

## 12. Loi gap phai va cach sua

- Full build loi do symlink-install gap thu muc generated cu trong `build/`.
  - Da xoa dung artifact generated trong `build/`, khong sua source package khong lien quan.
- `ros2 node list` ban dau canh bao duplicate `/robot_drl_executor_node`.
  - Nguyen nhan: launch `name=` remap anh huong node phu cua MoveIt trong process.
  - Da bo `name="robot_drl_executor_node"` trong launch; node chinh van dat ten trong C++.
- `/move_pose_rl execute=true` lan dau timeout o `execution_status`.
  - Nguyen nhan: DRL tao chuoi gan-degenerate `[target, near target, target]`; Cartesian path chi dat `fraction=0.25`, PTP fallback chay ca 3 waypoint cham hon `drl_timeout_sec=120`.
  - Da them logic trong `robot_drl_executor_node`: neu PTP fallback nhan loop degenerate co first/last pose trung nhau trong `1e-4 m`, chi execute final pose.
- `/move_pose_rl execute=true` lan retry gan target fail do Cartesian trajectory co `time_from_start` khong tang.
  - Da them check timing hop le; neu Cartesian trajectory non-increasing time hoac execute fail thi fallback PTP.
- `mock_environment_node` trong `robot_drl drl_mock_hw.launch.py` van fail import `Box` tu `robot_vision_pipeline.msg`.
  - Day la loi da duoc report cu ghi nhan; manual-mode DRL planner van chay va test `/move_pose_rl` thanh cong.
- Khi dung launch bang Ctrl-C, mot so MoveIt/RViz process van co shutdown segfault/RCLError.
  - Hien tuong nay da xuat hien trong report cu va xay ra o shutdown, khong chan build/test chinh.

## 13. Trang thai cuoi cung

Chay duoc cho flow migration chinh:

- Package moi `robot_drl_executor` build duoc.
- `robot_task_executor` van con trong workspace va khong bi xoa.
- `robot_drl` launch mock hardware goi `robot_drl_executor`.
- `robot_task_manager` action `/move_pose_rl` goi backend DRL, backend goi `/move_cartesian_pose_sequence` cua `robot_drl_executor`, va action plan/execute thanh cong.
- Full workspace build thanh cong.

Gioi han con lai:

- `/drl_pickplace execute=false` smoke da qua backend/action con toi stage place planning, nhung khong hoan tat trong test vi target place trong command smoke khong converge voi DRL planner hien tai. Khong phai loi thieu `robot_drl_executor`; service executor moi da duoc xac nhan qua `/move_pose_rl execute=true`.
