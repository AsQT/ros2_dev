    # robot_drl_executor - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | move_group_name | arm | string | node/launch/yaml | node/launch/yaml | MoveIt planning group |
| base_frame | base_link | string | node/launch/yaml | node/launch/yaml | Frame nhận pose |
| ee_link | tcp_link | string | node/launch/yaml | node/launch/yaml | End effector link |
| cartesian_pose_sequence_service_name | /move_cartesian_pose_sequence | string | node/launch/yaml | node/launch/yaml | Tên service server |
| planning_time | 2.0 | double | node/launch/yaml | node/launch/yaml | Thời gian plan |
| num_planning_attempts | 5 | int | node/launch/yaml | node/launch/yaml | Số lần plan |
| max_velocity_scaling_factor | 0.1 | double | node/launch/yaml | node/launch/yaml | Scale vận tốc |
| max_acceleration_scaling_factor | 0.5 | double | node/launch/yaml | node/launch/yaml | Scale gia tốc |
| cartesian_eef_step | 0.01 | double | node/launch/yaml | node/launch/yaml | Bước Cartesian mét |
| cartesian_jump_threshold | 0.0 | double | node/launch/yaml | node/launch/yaml | Ngưỡng jump |
| cartesian_success_threshold | 0.95 | double | node/launch/yaml | node/launch/yaml | Fraction tối thiểu |

    ## 3. Parameter theo launch file
    robot_drl_executor.launch.py

    ## 4. Parameter theo YAML config
    config/robot_drl_executor.yaml

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_drl_executor`.
