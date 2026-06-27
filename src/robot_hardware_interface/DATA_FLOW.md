    # robot_hardware_interface - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Service/trajectory command bằng rad/rad/s -> hardware node/plugin đổi sang protocol robot (source TCP có hàm deg/mdeg tùy frame) -> controller trả position/velocity/flag -> node publish rad/rad/s và status.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Publish `/joint_states`, `/robot_hw/connected`, `/hardware/connected` legacy, `/robot_hw/status_text`, `/robot_hw/flags`, `/hardware/flags`; subscribe `/robot_hw/servo_axis_cmd`, `/robot_hw/run_axis_cmd`, `/robot_hw/jog_cmd`, `/robot_hw/run_all_cmd`, controller trajectory; services `/robot_hw/connect`, `/robot_hw/disconnect`, `/robot_hw/poll_now`, `/robot_hw/servo_all`, `/robot_hw/servo_on_axis`, `/robot_hw/servo_on_all`, `/robot_hw/jog`, `/robot_hw/home`, `/robot_hw/run_axis`, `/robot_hw/stop_axis`, `/robot_hw/stop_all`.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `src/robot_hw_node.cpp`: node hardware standalone qua TCP client.
- `src/tcp_system_hardware.cpp`: plugin ros2_control lifecycle.
- `src/tcp_client.cpp`: framing/protocol TCP.
- `src/rs485_hw_node.cpp`, `rs485_system_hardware.cpp`: biến thể RS485 trong source.
- `srv/*.srv`: service điều khiển axis/all.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  GUI[GUI/controller] --> Service[robot_hw services/trajectory]
  Service --> TCP[TCP/RS485 protocol]
  TCP --> Robot[Robot drives]
  Robot --> State[/joint_states + flags]
    ```

    ## 6. Liên kết với package khác
    Tầng sát phần cứng. Nhận lệnh từ GUI/controller_manager, publish joint state/status và cung cấp service connect/servo/home/jog/run/stop.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
