    # robot_task_executor - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Service request -> load/resolve target -> MoveGroupInterface plan hoặc computeCartesianPath -> optional execute -> response success/message/fraction; visualization publish path/marker.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Service servers từ `robot_task_executor_msgs`: `/move_to_named_target`, `/move_to_joint_target`, `/move_to_pose_target`, `/move_to_named_pose_target`, `/move_to_cartesian_target`, `/move_to_named_cartesian_target`, `/move_cartesian_sequence`, `/move_cartesian_pose_sequence`, `/move_sequence`; publish marker/path visualization.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `src/task_executor_node.cpp`: service server chính.
- `planner_utils.cpp`: wrapper MoveGroupInterface plan/execute/cartesian fallback.
- `waypoint_loader.cpp`: load YAML waypoints.
- `visualization_utils.cpp`: publish path/marker.
- `transform_utils.cpp`: TF helper.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Input[Input] --> Package[robot_task_executor]
  Package --> Output[Output/interface]
    ```

    ## 6. Liên kết với package khác
    Backend MoveIt service độc lập cho client hoặc DRL đời cũ; song song với `robot_drl_executor` và `robot_task_manager` action servers.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
