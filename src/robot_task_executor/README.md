    # robot_task_executor

    ## 1. Vai trò package
    Service-based MoveIt task executor legacy. Cung cấp service move named/joint/pose/cartesian/sequence dựa trên YAML waypoint.

    ## 2. Vị trí trong hệ thống
    Backend MoveIt service độc lập cho client hoặc DRL đời cũ; song song với `robot_drl_executor` và `robot_task_manager` action servers.

    ## 3. Thành phần chính
    - `src/task_executor_node.cpp`: service server chính.
- `planner_utils.cpp`: wrapper MoveGroupInterface plan/execute/cartesian fallback.
- `waypoint_loader.cpp`: load YAML waypoints.
- `visualization_utils.cpp`: publish path/marker.
- `transform_utils.cpp`: TF helper.
- `config/*.yaml`: named/joint/cartesian/pose waypoints.
- `launch/task_executor.launch.py`: chạy node.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Executable `task_executor_node`. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | Xem mô tả bên dưới | runtime interface | package dependent |

    Ghi chú interface: Service servers từ `robot_task_executor_msgs`: `/move_to_named_target`, `/move_to_joint_target`, `/move_to_pose_target`, `/move_to_named_pose_target`, `/move_to_cartesian_target`, `/move_to_named_cartesian_target`, `/move_cartesian_sequence`, `/move_cartesian_pose_sequence`, `/move_sequence`; publish marker/path visualization.

    ## 6. File launch liên quan
    task_executor.launch.py

    ## 7. File cấu hình liên quan
    config/task_executor.yaml, config/joint_waypoints.yaml, config/cartesian_points.yaml, config/pose_waypoints.yaml

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_task_executor
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_task_executor task_executor.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_task_executor`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Các service đều liên quan MoveIt. `execute=false` là plan-only; Cartesian yêu cầu fraction >= 0.95 hoặc fallback PTP tùy hàm.
