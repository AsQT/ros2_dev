    # robot_moveit - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | use_sim_time | false | bool | launch | launch | Clock simulation |
| use_mock | true | bool | launch include robot_control | launch include robot_control | Dùng mock hardware trong xacro/controller |
| start_controller_manager | true | bool | moveit.launch.py/moveit_gui.launch.py | moveit.launch.py/moveit_gui.launch.py | Bật include controller manager |
| gui_delay | 3.0 | double | moveit_gui.launch.py | moveit_gui.launch.py | Delay mở robot_gui |
| initial_page | 1 | int | moveit_gui.launch.py | moveit_gui.launch.py | Page GUI ban đầu |

    ## 3. Parameter theo launch file
    moveit.launch.py, moveit_gui.launch.py, moveit_mock.launch.py

    ## 4. Parameter theo YAML config
    config/robot.srdf, config/kinematics.yaml, config/joint_limits.yaml, config/moveit_controllers.yaml, config/pilz_cartesian_limits.yaml, config/sensors_3d.yaml, config/moveit.rviz

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_moveit`.
