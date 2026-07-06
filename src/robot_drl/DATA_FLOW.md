    # robot_drl - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Target/box từ mock hoặc vision -> unified planner tạo observation -> model DRL sinh waypoint -> publish PoseArray/Marker -> `/drl/execute_forward` gọi `/move_cartesian_pose_sequence` -> MoveIt executor plan/execute.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Service `/drl/plan`, `/drl/replan`, `/drl/execute_forward`, `/drl/execute_backward`, `/drl/execute_trajectory`, `/drl/clear_trajectory`, `/drl/get_execution_status`, `/drl/get_planning_status`; service client `/compute_ik`, planning scene và `/move_cartesian_pose_sequence`; topic `/drl/forward_trajectory_poses`, `/drl/backward_trajectory_poses`, `/drl/next_pose`, `/drl/execution_status`, `/drl/plan_stats`, mock target/box topics.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `robot_drl/drl_unified_planner_node.py`: planner DDPG/SAC/TD3 unified.
- `robot_drl/drl_planner_node_base.py`: service `/drl/execute_forward`, `/drl/clear_trajectory`, marker/pose publisher.
- `robot_drl/mock_environment_node.py`: publish target/box mock.
- `mock_hw_obstacle_test.py`, `gazebo_obstacle_test.py`: test obstacle.
- `models/run*/model/*.zip`: model DRL được cài vào share.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Target[Mock/Vision target] --> Planner[DRL unified planner]
  Planner --> Poses[/drl/forward_trajectory_poses]
  Client[/drl/execute_forward] --> Executor[/move_cartesian_pose_sequence]
  Executor --> MoveIt[MoveIt trajectory]
    ```

    ## 6. Liên kết với package khác
    Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
