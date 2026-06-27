    # robot_bringup - Data Flow

    ## 1. Mục tiêu luồng dữ liệu
    Launch tổng hợp nạp mô tả robot, controller, MoveIt, action servers và DRL nodes; dữ liệu điều khiển đi qua action/service của các package con.

    ## 2. Input
    - Launch arguments, YAML config hoặc request/action goal từ package gọi.
    - Interface đầu vào cụ thể: Không định nghĩa topic/service/action riêng. Các interface runtime đến từ package được include: action của `robot_task_manager`, service `/move_cartesian_pose_sequence`, topic Gazebo/ROS bridge và controller.

    ## 3. Output
    - Node, topic, service, action result hoặc file/message được mô tả trong README.
    - Output runtime phải kiểm tra bằng ROS CLI sau khi launch.

    ## 4. Internal processing
    - `launch/sim.launch.py`: Gazebo + MoveIt + task servers simulation + controller spawner.
- `launch/real.launch.py`: MoveIt GUI + task servers với `use_mock=false`.
- `launch/drl_test.launch.py`: Gazebo + MoveIt + `robot_drl_executor` + DRL planner + RViz.
- `launch/rl_pick_place_box_gazebo_demo.launch.py`: demo pick-place hộp trong Gazebo, spawn box và client demo.
- `config/common.yaml`, `config/ros2_controllers.yaml`: cấu hình dùng kèm bringup/controller.

    ## 5. Sơ đồ luồng dữ liệu
    ```mermaid
    flowchart LR
  Launch[bringup launch] --> Desc[robot_description]
  Launch --> MoveIt[robot_moveit/move_group]
  Launch --> Task[robot_task_manager actions]
  Launch --> DRL[robot_drl + robot_drl_executor]
  Task --> Controller[ros2_control/controllers]
    ```

    ## 6. Liên kết với package khác
    Đứng ở tầng orchestration: include `robot_description`, `robot_control`, `robot_moveit`, `robot_task_manager`, `robot_drl`, `robot_drl_executor`, RViz/Gazebo tùy mode.

    ## 7. Các điểm cần chú ý
    - Frame thường gặp trong workspace: `base_link`, `tcp_link`, `world`, `aruco_world`; package dùng frame cụ thể được nêu trong parameter/launch.
    - Đơn vị pose/joint runtime: MoveIt/ROS dùng mét và radian; GUI có chỗ nhập mm rồi đổi sang mét.
    - Source of truth: source code hiện tại, không suy diễn từ file build cũ.
