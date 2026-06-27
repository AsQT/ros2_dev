    # robot_gui - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | embed_rviz | false | bool | robot_gui_node/launch | robot_gui_node/launch | Nhúng RViz trong GUI |
| initial_page | -1 | int | robot_gui_node/launch | robot_gui_node/launch | Tab/page mở ban đầu |
| rviz_config_package | robot_moveit | string | robot_gui_node/launch | robot_gui_node/launch | Package chứa RViz config |
| rviz_config_relative_path | config/moveit.rviz | string | robot_gui_node/launch | robot_gui_node/launch | Đường dẫn RViz config |
| DEFAULT_GUI_VELOCITY_SCALE | 0.1 | constant | task_action_controller.cpp | task_action_controller.cpp | Velocity scale mặc định cho action |
| GUI pose units | mm input -> m goal | conversion | task_action_controller.cpp | task_action_controller.cpp | Các ô pose nhập mm, ROS action nhận m bằng `mm / 1000.0` |

    ## 3. Parameter theo launch file
    robot_gui.launch.py, embedded_rviz_cpp_test.launch.py

    ## 4. Parameter theo YAML config
    config/config.yaml, ui/robot_gui.ui

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_gui`.
