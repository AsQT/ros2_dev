    # robot_drl_executor

    ## 1. Vai trò package
    C++ service executor chuyên nhận chuỗi `geometry_msgs/PoseStamped[]` từ DRL và thực thi bằng MoveIt Cartesian path.

    ## 2. Vị trí trong hệ thống
    Backend MoveIt cho `robot_drl`; cũng có thể được gọi trực tiếp bởi client qua service `/move_cartesian_pose_sequence`.

    ## 3. Thành phần chính
    - `src/robot_drl_executor_node.cpp`: node/service server MoveIt.
- `config/robot_drl_executor.yaml`: cấu hình mặc định.
- `launch/robot_drl_executor.launch.py`: launch node với tham số MoveIt/Cartesian.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Executable `robot_drl_executor_node`. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | /move_cartesian_pose_sequence | MoveCartesianPoseSequence.srv | service server |

    Ghi chú interface: Service server `/move_cartesian_pose_sequence` type `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`; dùng MoveGroupInterface group `arm`, frame `base_link`, ee `tcp_link`.

    ## 6. File launch liên quan
    robot_drl_executor.launch.py

    ## 7. File cấu hình liên quan
    config/robot_drl_executor.yaml

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_drl_executor
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_drl_executor robot_drl_executor.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_drl_executor`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Service `/move_cartesian_pose_sequence` là interface MoveIt chính; `execute=false` chỉ plan và trả fraction, `execute=true` gọi execute trajectory nếu plan đạt threshold hoặc fallback thành công.
