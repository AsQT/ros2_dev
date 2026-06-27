    # robot_bringup

    ## 1. Vai trò package
    Package launch tổng hợp để khởi động robot ở chế độ simulation, real hardware và demo DRL pick-place. Source of truth: `launch/*.launch.py` và `package.xml` hiện tại.

    ## 2. Vị trí trong hệ thống
    Đứng ở tầng orchestration: include `robot_description`, `robot_control`, `robot_moveit`, `robot_task_manager`, `robot_drl`, `robot_drl_executor`, RViz/Gazebo tùy mode.

    ## 3. Thành phần chính
- `launch/sim.launch.py`: Gazebo từ `robot_gazebo` + MoveIt + task servers simulation + controller spawner.
- `launch/real.launch.py`: MoveIt GUI + task servers với `use_mock=false`.
- `launch/drl_test.launch.py`: Gazebo + MoveIt + `robot_drl_executor` + DRL planner + RViz.
- `launch/rl_pick_place_box_gazebo_demo.launch.py`: demo pick-place hộp trong Gazebo, spawn box bằng `robot_gazebo` và client demo.
- `config/common.yaml`, `config/ros2_controllers.yaml`: cấu hình dùng kèm bringup/controller.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Không build executable riêng; launch file khởi tạo node từ package khác như `move_group`, `drl_unified_planner_node`, `robot_drl_executor_node`, `task_servers_*`, controller spawner và RViz. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | Xem mô tả bên dưới | runtime interface | package dependent |

    Ghi chú interface: Không định nghĩa topic/service/action riêng. Các interface runtime đến từ package được include: action của `robot_task_manager`, service `/move_cartesian_pose_sequence`, topic Gazebo/ROS bridge và controller.

    ## 6. File launch liên quan
    sim.launch.py, real.launch.py, drl_test.launch.py, rl_pick_place_box_gazebo_demo.launch.py

    ## 7. File cấu hình liên quan
    config/common.yaml, config/ros2_controllers.yaml

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_bringup
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_bringup sim.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_bringup`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Bringup không gọi MoveIt trực tiếp trong code, nhưng launch `move_group`, `task_servers_*`, `robot_drl_executor_node` và GUI/client. Thất bại thường đến từ include thiếu package, controller chưa active hoặc action/server chưa sẵn sàng.
