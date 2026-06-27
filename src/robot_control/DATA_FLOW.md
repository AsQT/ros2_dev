    # robot_control - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Robot description được đưa vào `ros2_control_node`; controller manager sinh joint states và nhận trajectory command từ MoveIt/GUI.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Topic/controller interface do ros2_control tạo, tiêu biểu `/joint_states`, `arm_controller/joint_trajectory`, `gripper_controller/joint_trajectory`.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `config/robot_controllers.yaml`: controller manager, joint state broadcaster, arm/gripper controller.
- `config/arm_controller.yaml`, `gripper_controller.yaml`: cấu hình controller riêng.
- `config/initial_positions.yaml`: giá trị khởi tạo joint.
- `launch/controllers.launch.py`: chạy `robot_state_publisher`, `ros2_control_node`, controller spawner.
- `launch/slider_controller.launch.py`: test slider GUI/launch phụ thuộc `panda_controller`.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Input[Input] --> Package[robot_control]
  Package --> Output[Output/interface]
    ```

    ## 6. Liên kết với package khác
    Nằm giữa mô tả robot/hardware và MoveIt/task execution. `robot_moveit` và `robot_bringup` include launch controller từ package này.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
