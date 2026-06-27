# robot_drl_executor migration audit report

Ngay tao: 2026-06-27

## 1. Danh sach file report da doc

- `robot_task_manager/drl_backend_launch_audit_report.md`
- `robot_task_manager/move_pose_rl_action_report.md`
- `robot_task_manager/task_servers_launch_fix_report.md`
- `robot_task_manager/checker_board_current_flow_report.md`
- `robot_task_manager/checker_board_segmented_cartesian_report.md`
- `robot_task_manager/gohome_action_fix_report.md`
- `robot_task_manager/pickplace_start_state_fix_report.md`
- `robot_gui/move_pose_rl_gui_backend_report.md`

Da kiem tra bang `find robot_task_executor robot_task_manager robot_drl robot_bringup robot_description -type f -iname '*report*.md'`. Trong cac package duoc yeu cau dac biet, chi `robot_task_manager` co report; `robot_task_executor`, `robot_drl`, `robot_bringup`, va `robot_description` khong co file `*report*.md`.

## 2. Flow DRL/RL hien dang chay duoc

- `/move_pose_rl` cua `robot_task_manager` goi backend `robot_drl/drl_unified_planner_node`.
  - Plan-only mock hardware da tung thanh cong voi `execute=false`.
  - Execute mock hardware da tung thanh cong voi `execute=true`.
  - Backend can `/drl_unified_planner_node/set_parameters`, `/drl/clear_trajectory`, `/drl/plan`, `/drl/forward_trajectory_poses`, va khi execute can `/drl/execute_forward`, `/drl/get_execution_status`.
- `/drl_pickplace` cua `robot_task_manager` goi cung backend `robot_drl/drl_unified_planner_node`.
  - Backend DRL da tung qua buoc wait service; flow day du con phu thuoc action con `/move_gripper` va `/move_to_pose_cartesian`.
- `robot_drl drl_mock_hw.launch.py` la test mock hardware uu tien. Report cu ghi `mock_environment_node` co the fail do import `Box`, nhung `drl_unified_planner_node` van cung cap `/drl/*` va flow manual van test duoc.
- `robot_drl drl_gazebo.launch.py`, `robot_bringup drl_test.launch.py`, va `robot_bringup rl_pick_place_box_gazebo_demo.launch.py` la cac launch Gazebo/RL co dung executor cu.

## 3. Cac service/action/topic dang duoc dung boi DRL

Action cua `robot_task_manager`:

- `/move_pose_rl` (`robot_task_manager/action/MovePoseRl`)
- `/drl_pickplace` (`robot_task_manager/action/DrlPickPlace`)

Service/topic cua `robot_drl/drl_unified_planner_node`:

- `/drl_unified_planner_node/set_parameters`
- `/drl/plan`
- `/drl/clear_trajectory`
- `/drl/execute_forward`
- `/drl/get_execution_status`
- `/drl/execute_backward`
- `/drl/execute_trajectory`
- `/drl/forward_trajectory_poses`
- `/drl/forward_trajectory_marker`
- `/drl/backward_trajectory_poses`
- `/drl/backward_trajectory_marker`
- `/drl/next_pose`
- `/drl/execution_status`

Service executor MoveIt ma DRL goi truc tiep:

- `/move_cartesian_pose_sequence` type `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`

Dependency phu tro:

- `/joint_states`
- TF `base_link <- tcp_link`
- `/get_planning_scene` khi bat planning scene obstacles
- `/move_gripper` va `/move_to_pose_cartesian` trong flow `/drl_pickplace`

## 4. Cac file dang import/goi `robot_task_executor`

Ket qua trace `rg -n "robot_task_executor" .` va `rg -n "task_executor" .` cho cac flow lien quan:

- `robot_drl/package.xml`
  - `exec_depend` toi `robot_task_executor`
  - `depend` toi `robot_task_executor_msgs` van can giu vi service type van o package msg hien co.
- `robot_drl/launch/drl_mock_hw.launch.py`
  - include `robot_task_executor/launch/task_executor.launch.py`.
- `robot_drl/launch/drl_gazebo.launch.py`
  - include `robot_task_executor/launch/task_executor.launch.py`.
- `robot_drl/launch/rl_sim_rviz.launch.py`
  - include `robot_task_executor/launch/task_executor.launch.py`.
  - goi CLI `/move_to_named_pose_target` type `robot_task_executor_msgs/srv/MoveToNamedPoseTarget` de dua robot ve `pose_A`.
- `robot_bringup/launch/drl_test.launch.py`
  - include `robot_task_executor/launch/task_executor.launch.py`.
- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`
  - include `robot_task_executor/launch/task_executor.launch.py`.
- `robot_drl/robot_drl/drl_planner_node_base.py`
  - import `robot_task_executor_msgs.srv._move_cartesian_pose_sequence.MoveCartesianPoseSequence`.
  - default `cartesian_pose_sequence_service_name` la `/move_cartesian_pose_sequence`.
- `robot_drl/robot_drl/gazebo_obstacle_test.py`
  - import `MoveCartesianPoseSequence`.
  - goi `/move_cartesian_pose_sequence`.
- `robot_bringup/launch/sim.launch.py`
  - khong include package `robot_task_executor`; no include `robot_task_manager/launch/task_servers_sim.launch.py`.

## 5. Cac thanh phan khong dung hoac nghi ngo khong dung

Khong migrate trong dot nay:

- `/move_to_named_target`
- `/move_to_joint_target`
- `/move_to_pose_target`
- `/move_to_named_pose_target`
- `/move_to_cartesian_target`
- `/move_to_named_cartesian_target`
- `/move_cartesian_sequence`
- `/move_sequence`
- YAML waypoint legacy: `joint_waypoints.yaml`, `cartesian_points.yaml`, `pose_waypoints.yaml`

Ly do: flow DRL hien tai chi goi `/move_cartesian_pose_sequence` trong `robot_drl/drl_planner_node_base.py` va `robot_drl/robot_drl/gazebo_obstacle_test.py`. Rieng `robot_drl/launch/rl_sim_rviz.launch.py` con dung `/move_to_named_pose_target` de dua robot ve `pose_A`; phan nay la compatibility launch cu, chua duoc xac nhan la flow DRL action chinh. Se khong migrate service named pose trong dot nay de tranh copy toan bo `robot_task_executor`.

## 6. Ke hoach migrate cu the

1. Tao package moi `robot_drl_executor` song song, khong xoa hay rename `robot_task_executor`.
2. Tao node C++ moi `robot_drl_executor_node` chi cung cap service `/move_cartesian_pose_sequence` voi type `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`.
3. Giu hanh vi can thiet cua handler cu:
   - validate danh sach pose khong rong;
   - default frame rong ve `base_frame`;
   - chi chap nhan pose trong `base_frame`;
   - thay quaternion zero bang fallback `[0.7071068, 0.7071068, 0.0, 0.0]`;
   - goi MoveIt `computeCartesianPath`;
   - neu fraction thap thi fallback point-to-point qua tung waypoint;
   - `execute=false` chi plan, `execute=true` execute.
4. Tao launch `robot_drl_executor/launch/robot_drl_executor.launch.py` voi tham so tuong duong executor cu nhung package/executable/node name moi.
5. Cap nhat cac launch DRL/RL da trace de include `robot_drl_executor` thay vi `robot_task_executor` khi flow can `/move_cartesian_pose_sequence`.
6. Cap nhat dependency runtime:
   - `robot_drl/package.xml`: doi `exec_depend` tu `robot_task_executor` sang `robot_drl_executor`.
   - `robot_bringup/package.xml`: them `exec_depend` toi `robot_drl_executor` neu launch cua package nay include package moi.
   - `robot_task_manager/package.xml`: them `exec_depend` toi `robot_drl_executor` de flow DRL action co executor moi khi execute backend.
7. Khong sua reward, observation, action space, Gazebo spawn, pick/place pose, hay logic planner DRL.
8. Build va test theo cac flow report: build package lien quan, smoke launch executor moi, smoke launch mock DRL/action services, va ghi ket qua vao `robot_drl_executor/robot_drl_executor_migration_report.md`.
