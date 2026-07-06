    # robot_drl

    ## 1. Vai trò package
    Package Python DRL planner cho quỹ đạo tránh vật cản. Node chính `drl_unified_planner_node` publish trajectory PoseArray và cung cấp service điều khiển plan/execute/clear/status.

    ## 2. Vị trí trong hệ thống
    Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

    ## 3. Thành phần chính
    - `robot_drl/drl_unified_planner_node.py`: planner DDPG/SAC/TD3 unified.
- `robot_drl/drl_planner_node_base.py`: service `/drl/execute_forward`, `/drl/clear_trajectory`, marker/pose publisher.
- `robot_drl/mock_environment_node.py`: publish target/box mock.
- `mock_hw_obstacle_test.py`, `gazebo_obstacle_test.py`: test obstacle.
- `models/run*/model/*.zip`: model DRL được cài vào share.
- Launch mock/gazebo/main/rviz/test trong `launch/`.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Console scripts: `drl_unified_planner_node`, `mock_environment_node`, `mock_hw_obstacle_test`, `gazebo_obstacle_test`. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | /drl/plan | service/topic | DRL planner |
| /drl/execute_forward | service/topic | DRL planner |
| /drl/clear_trajectory | service/topic | DRL planner |
| /drl/get_execution_status | service/topic | DRL planner |
| /drl/forward_trajectory_poses | PoseArray topic | DRL forward trajectory |
| /drl/backward_trajectory_poses | PoseArray topic | DRL backward trajectory |
| /drl/last_plan_observation_15d | Float64MultiArray topic | Observation 15D của plan gần nhất |

    Ghi chú interface: Service `/drl/plan`, `/drl/replan`, `/drl/execute_forward`, `/drl/execute_backward`, `/drl/execute_trajectory`, `/drl/clear_trajectory`, `/drl/get_execution_status`, `/drl/get_planning_status`; service client `/compute_ik`, planning scene và `/move_cartesian_pose_sequence`; topic `/drl/forward_trajectory_poses`, `/drl/backward_trajectory_poses`, `/drl/forward_trajectory_marker`, `/drl/backward_trajectory_marker`, `/drl/next_pose`, `/drl/execution_status`, `/drl/plan_stats`, `/drl/last_plan_observation_15d`, mock target/box topics.

    ## 6. File launch liên quan
    main.launch.py, drl_unified_planner.launch.py, mock_environment.launch.py, mock_drl.launch.py, mock_drl_rviz.launch.py, drl_mock_hw.launch.py, drl_gazebo.launch.py, rl_sim_rviz.launch.py, drl_mock_hw_obstacle_test.launch.py, drl_gazebo_obstacle_test.launch.py, rviz_drl.launch.py

    ## 7. File cấu hình liên quan
    models/run*/config.json, models/run2/config.yaml, rviz/drl_markers.rviz

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_drl
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_drl main.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_drl`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - DRL không dùng MoveGroupInterface trực tiếp trong file chính như C++, nhưng phụ thuộc MoveIt qua `/compute_ik`, planning scene và service Cartesian executor `/move_cartesian_pose_sequence`. Plan-only là gọi `/drl/plan` nhưng không execute; execute gọi `/drl/execute_forward`.
