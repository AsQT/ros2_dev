    # robot_hardware_interface - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | robot_ip | varies | string | params.yaml/tcp_system_hardware | params.yaml/tcp_system_hardware | IP controller TCP |
| robot_port | varies | int | params.yaml/tcp_system_hardware | params.yaml/tcp_system_hardware | Port TCP |
| connect_timeout_ms | varies | int | tcp client/config | tcp client/config | Timeout connect |
| poll_status_hz | 5.0 | double | rs485_hw_node | rs485_hw_node | Tần số poll flag |
| poll_pos_hz | 20.0 | double | rs485_hw_node | rs485_hw_node | Tần số poll joint position |
| home_positions_rad | [] | double[] | rs485_hw_node/config | rs485_hw_node/config | Vị trí home theo rad |
| axis_ids | 0..7 | int[] | rs485_hw_node | rs485_hw_node | Mapping id trục |
| joint_names | joint_1.. | string[] | rs485_hw_node/config | rs485_hw_node/config | Tên joint xuất `/joint_states` |

    ## 3. Parameter theo launch file
    hardware_interface.launch.py

    ## 4. Parameter theo YAML config
    config/params.yaml, plugin.xml

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_hardware_interface`.
