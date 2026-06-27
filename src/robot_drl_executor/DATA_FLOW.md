    # robot_drl_executor - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    PoseStamped[] -> validate frame/orientation -> computeCartesianPath -> nếu fraction thấp thử PTP fallback -> execute hoặc trả plan-only result.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Service server `/move_cartesian_pose_sequence` type `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`; dùng MoveGroupInterface group `arm`, frame `base_link`, ee `tcp_link`.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `src/robot_drl_executor_node.cpp`: node/service server MoveIt.
- `config/robot_drl_executor.yaml`: cấu hình mặc định.
- `launch/robot_drl_executor.launch.py`: launch node với tham số MoveIt/Cartesian.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Input[Input] --> Package[robot_drl_executor]
  Package --> Output[Output/interface]
    ```

    ## 6. Liên kết với package khác
    Backend MoveIt cho `robot_drl`; cũng có thể được gọi trực tiếp bởi client qua service `/move_cartesian_pose_sequence`.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
