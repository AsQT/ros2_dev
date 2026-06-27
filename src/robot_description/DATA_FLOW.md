    # robot_description - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Xacro sinh URDF -> robot_state_publisher phát TF -> Gazebo spawn entity -> ros_gz_bridge nối clock/joint state/sensor cần thiết.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Xuất `robot_description` parameter, TF từ `robot_state_publisher`, bridge Gazebo qua `ros_gz_bridge`, topic thông tin object từ script spawn khi có.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `urdf/robot.urdf.xacro`, `robot.xacro`: mô tả chính.
- `urdf/ros2_control.xacro`: tag ros2_control/mock/real hardware.
- `urdf/gazebo.xacro`, `sensors.xacro`, `materials.xacro`: Gazebo/sensor/material.
- `meshes/`: hình học visual/collision.
- `worlds/*/*.sdf`: table, wood block, checker board, pick box.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Input[Input] --> Package[robot_description]
  Package --> Output[Output/interface]
    ```

    ## 6. Liên kết với package khác
    Là nguồn `robot_description` cho controller, MoveIt, RViz và Gazebo; các package khác không nên tự định nghĩa frame robot.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
