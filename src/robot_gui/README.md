    # robot_gui

    ## 1. Vai trò package
    GUI Qt5/C++ để điều khiển hardware, hiển thị trạng thái, gửi action task và có tùy chọn nhúng RViz.

    ## 2. Vị trí trong hệ thống
    Tầng operator UI. Gọi service hardware từ `robot_hardware_interface`, publish trajectory controller trực tiếp, và gửi action đến `robot_task_manager`.

    ## 3. Thành phần chính
    - `ui/robot_gui.ui`: layout Qt Designer chính.
- `src/robot_gui_node.cpp`: node ROS cho hardware service/topic và camera topics.
- `src/task_action_controller.cpp`: controller action cho task tab.
- `src/main_window.cpp`, `rviz_panel.cpp`: shell Qt/RViz.
- `config/config.yaml`: cấu hình GUI.
- Các widget quan trọng trong source/UI: `TaskControlPanel`, `cbModeControl`, `taskModeTabs`, action log.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Executable `robot_gui_node`. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | Xem mô tả bên dưới | runtime interface | package dependent |

    Ghi chú interface: Subscribe `/joint_states`, `/robot_hw/flags`, camera image topics, `/vision/wood_objects`, `/vision/box_objects`; service clients `/robot_hw/servo_all`, `/robot_hw/home`, `/robot_hw/jog`, `/robot_hw/run_axis`, `/robot_hw/stop_axis`, `/robot_hw/stop_all`; publish `arm_controller/joint_trajectory`, `gripper_controller/joint_trajectory`; action clients `/gohome`, `/gohome_2`, `/move_to_pose`, `/move_to_pose_cartesian`, `/move_to_pose_obstacle`, `/move_gripper`, `/pickplace`, `/drl_pickplace`, `/move_checker_board`, `/move_pose_rl`, `/repeatability_test`.

    ## 6. File launch liên quan
    robot_gui.launch.py, embedded_rviz_cpp_test.launch.py

    ## 7. File cấu hình liên quan
    config/config.yaml, ui/robot_gui.ui

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_gui
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_gui robot_gui.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_gui`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - GUI không plan bằng MoveIt trực tiếp trong task controller; nó gửi action tới `robot_task_manager`. Button Plan gửi `execute=false`, Start gửi `execute=true`. MoveIt chạy ở server phía task manager/move_group.
