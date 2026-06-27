    # robot_drl - Parameters

    ## 1. Tổng quan
    File này tổng hợp parameter thấy được từ `declare_parameter`, `get_parameter`, `DeclareLaunchArgument`, `LaunchConfiguration`, `parameters=[...]` và YAML/config trong package.

    ## 2. Bảng parameter
    | Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
    |---|---:|---|---|---|---|
    | input_mode | manual | string | launch/node | launch/node | manual/mock/vision input mode |
| auto_plan_on_start | true | bool | launch/node | launch/node | Tự plan sau khi start |
| manual_prompt_on_start | true/false | bool | launch/node | launch/node | Yêu cầu prompt thủ công |
| auto_execute_after_plan | false/true | bool | launch/node | launch/node | Tự gọi execute sau khi plan |
| calibrated_start_tcp_base | [x,y,z,...] | array | launch/node | launch/node | TCP start đã calibrate trong base |
| use_sim_time | true/false | bool | launch | launch | Clock simulation |
| target_x/y/z | varies | double | mock launches | mock launches | Target mock theo mét |
| target_class_name | box | string | mock_environment_node | mock_environment_node | Tên class target |
| model path/config | models/run* | path | setup.py/models | setup.py/models | Model DRL và metadata |

    ## 3. Parameter theo launch file
    main.launch.py, drl_unified_planner.launch.py, mock_environment.launch.py, mock_drl.launch.py, mock_drl_rviz.launch.py, drl_mock_hw.launch.py, drl_gazebo.launch.py, rl_sim_rviz.launch.py, drl_mock_hw_obstacle_test.launch.py, drl_gazebo_obstacle_test.launch.py, rviz_drl.launch.py

    ## 4. Parameter theo YAML config
    models/run*/config.json, models/run2/config.yaml, rviz/drl_markers.rviz

    ## 5. Giá trị mặc định quan trọng
    - Các scale vận tốc MoveIt mặc định trong executor/task thường là `0.1`; acceleration scale thường là `0.5` khi source khai báo.
    - Cartesian path trong executor dùng fraction threshold `0.95` khi source có kiểm tra Cartesian success.
    - `execute=false` có nghĩa plan-only ở các action/service task.

    ## 6. Ghi chú thay đổi / rủi ro cấu hình
    - Parameter có trong YAML nhưng node không load file đó sẽ không có hiệu lực; cần kiểm tra launch có truyền YAML hay không.
    - Parameter có trong launch nhưng service/action server phụ thuộc package khác vẫn cần node đích đang chạy.
    - Source of truth là source code hiện tại trong `robot_drl`.
