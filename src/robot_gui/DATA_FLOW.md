    # robot_gui - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Người vận hành thao tác UI -> GUI validate input/chuyển đơn vị -> service/action/trajectory -> hardware/task manager -> feedback/result trả về action log/status widget.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Subscribe `/joint_states`, `/robot_hw/flags`, camera image topics; service clients `/robot_hw/servo_all`, `/robot_hw/home`, `/robot_hw/jog`, `/robot_hw/run_axis`, `/robot_hw/stop_axis`, `/robot_hw/stop_all`; publish `arm_controller/joint_trajectory`, `gripper_controller/joint_trajectory`; action clients `/gohome`, `/move_to_pose`, `/move_to_pose_cartesian`, `/move_gripper`, `/pickplace`, `/move_checker_board`, `/move_pose_rl`, `/repeatability_test`.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `ui/robot_gui.ui`: layout Qt Designer chính.
- `src/robot_gui_node.cpp`: node ROS cho hardware service/topic và camera topics.
- `src/task_action_controller.cpp`: controller action cho task tab.
- `src/main_window.cpp`, `rviz_panel.cpp`: shell Qt/RViz.
- `config/config.yaml`: cấu hình GUI.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Input[Input] --> Package[robot_gui]
  Package --> Output[Output/interface]
    ```

    ## 6. Liên kết với package khác
    Tầng operator UI. Gọi service hardware từ `robot_hardware_interface`, publish trajectory controller trực tiếp, và gửi action đến `robot_task_manager`.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
