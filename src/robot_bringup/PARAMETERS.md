    # robot_bringup - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
| spawn_demo_woods | true | bool | sim.launch.py, robot_gazebo/gazebo.launch.py | sim.launch.py, robot_gazebo/gazebo.launch.py | Bật spawn gỗ demo trong Gazebo |
| auto_plan_on_start | true | bool | drl_test.launch.py | drl_test.launch.py | Cho DRL planner tự plan khi start |
| manual_prompt_on_start | false | bool | drl_test.launch.py | drl_test.launch.py | Hỏi xác nhận trước khi chạy nếu bật |
| auto_execute_after_plan | false | bool | drl_test.launch.py | drl_test.launch.py | Tự execute sau khi plan |
| randomize_box | false | bool | rl_pick_place_box_gazebo_demo.launch.py | rl_pick_place_box_gazebo_demo.launch.py | Random vị trí hộp demo |
| box_x, box_y, box_size | 0.42, 0.00, 0.03 | double | rl_pick_place_box_gazebo_demo.launch.py | rl_pick_place_box_gazebo_demo.launch.py | Pose/kích thước hộp theo mét |
| gripper_close_width | 0.025 | double | rl_pick_place_box_gazebo_demo.launch.py | rl_pick_place_box_gazebo_demo.launch.py | Độ đóng gripper theo mét |
| demo_client_delay | 50.0 | double | rl_pick_place_box_gazebo_demo.launch.py | rl_pick_place_box_gazebo_demo.launch.py | Delay trước khi chạy client demo |

    ## 3. Parameter theo launch file
    sim.launch.py, real.launch.py, drl_test.launch.py, rl_pick_place_box_gazebo_demo.launch.py

    ## 4. Parameter theo YAML config
    config/common.yaml, config/ros2_controllers.yaml

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_bringup`.
