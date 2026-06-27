    # robot_task_executor - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | move_group_name | arm | string | node/launch | node/launch | MoveIt group |
| base_frame | base_link | string | node/launch | node/launch | Frame task |
| ee_link | tcp_link | string | node/launch | node/launch | EEF |
| waypoints_config_path | config/joint_waypoints.yaml | path | launch | launch | YAML joint waypoints |
| cartesian_points_config_path | config/cartesian_points.yaml | path | launch | launch | YAML cartesian points |
| pose_waypoints_config_path | config/pose_waypoints.yaml | path | launch | launch | YAML pose waypoints |
| planning_time | 2.0 | double | node/launch | node/launch | Thời gian plan |
| num_planning_attempts | 5 | int | node/launch | node/launch | Số lần thử |
| max_velocity_scaling_factor | 0.1 | double | node/launch | node/launch | Scale vận tốc |
| max_acceleration_scaling_factor | 0.5 | double | node/launch | node/launch | Scale gia tốc |

    ## 3. Parameter theo launch file
    task_executor.launch.py

    ## 4. Parameter theo YAML config
    config/task_executor.yaml, config/joint_waypoints.yaml, config/cartesian_points.yaml, config/pose_waypoints.yaml

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_task_executor`.
