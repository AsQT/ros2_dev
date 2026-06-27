    # robot_description - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | spawn_demo_woods | true | bool | gazebo.launch.py | gazebo.launch.py | Bật/tắt spawn gỗ demo |
| robot_description | xacro command | string | display/gazebo launch | display/gazebo launch | URDF render từ xacro |
| gz_args | world file | string | gazebo.launch.py/gazebo_new.launch.py | gazebo.launch.py/gazebo_new.launch.py | World Gazebo được ros_gz_sim mở |

    ## 3. Parameter theo launch file
    display.launch.py, gazebo.launch.py, gazebo_new.launch.py

    ## 4. Parameter theo YAML config
    config/joint_names_robot.yaml, rviz/config.rviz, urdf/*.xacro, worlds/*/*.sdf

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_description`.
