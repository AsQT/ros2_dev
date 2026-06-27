    # robot_moveit - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    robot_description + SRDF/config -> move_group -> task/action executors gọi MoveGroupInterface hoặc action -> controller trajectory -> hardware/sim.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: MoveIt actions `/move_action` và `/execute_trajectory` từ move_group, service query planner, `/joint_states`, controller action từ ros2_control.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `config/robot.srdf`: group/state/EEF MoveIt.
- `config/kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, `pilz_cartesian_limits.yaml`, `sensors_3d.yaml`.
- `config/moveit.rviz`: RViz config.
- `launch/moveit.launch.py`, `moveit_gui.launch.py`, `moveit_mock.launch.py`.
- `gui/moveit_gui.py`: GUI Python thử nghiệm gọi MoveGroup/ExecuteTrajectory action.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  URDF[robot_description] --> MoveGroup[move_group]
  Config[SRDF/YAML] --> MoveGroup
  MoveGroup --> Controller[ros2_control]
  Client[Task/GUI executor] --> MoveGroup
    ```

    ## 6. Liên kết với package khác
    Cung cấp `move_group` và config cho task manager, task executor, DRL executor, GUI và bringup.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
