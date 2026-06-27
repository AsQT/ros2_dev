    # robot_control - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | use_sim_time | false | bool | controllers.launch.py | controllers.launch.py | Clock simulation |
| use_mock | true | bool | controllers.launch.py | controllers.launch.py | Truyền vào xacro/hardware mock |
| is_sim | false | bool | slider_controller.launch.py | slider_controller.launch.py | Mode simulation cho slider |
| is_ignition | false | bool | slider_controller.launch.py | slider_controller.launch.py | Mode Ignition/Gazebo |
| use_mock_hardware | true | bool | slider_controller.launch.py | slider_controller.launch.py | Dùng mock hardware |

    ## 3. Parameter theo launch file
    controllers.launch.py, slider_controller.launch.py

    ## 4. Parameter theo YAML config
    config/robot_controllers.yaml, config/arm_controller.yaml, config/gripper_controller.yaml, config/initial_positions.yaml

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_control`.
