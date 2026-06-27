    # robot_control

    ## 1. Vai trò package
    Package cấu hình ros2_control và launch controller cho robot; không có node ứng dụng riêng.

    ## 2. Vị trí trong hệ thống
    Nằm giữa mô tả robot/hardware và MoveIt/task execution. `robot_moveit` và `robot_bringup` include launch controller từ package này.

    ## 3. Thành phần chính
    - `config/robot_controllers.yaml`: controller manager, joint state broadcaster, arm/gripper controller.
- `config/arm_controller.yaml`, `gripper_controller.yaml`: cấu hình controller riêng.
- `config/initial_positions.yaml`: giá trị khởi tạo joint.
- `launch/controllers.launch.py`: chạy `robot_state_publisher`, `ros2_control_node`, controller spawner.
- `launch/slider_controller.launch.py`: test slider GUI/launch phụ thuộc `panda_controller`.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Không build executable riêng. Launch chạy node từ `robot_state_publisher`, `controller_manager`, `joint_state_publisher_gui` và package ngoài. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | Xem mô tả bên dưới | runtime interface | package dependent |

    Ghi chú interface: Topic/controller interface do ros2_control tạo, tiêu biểu `/joint_states`, `arm_controller/joint_trajectory`, `gripper_controller/joint_trajectory`.

    ## 6. File launch liên quan
    controllers.launch.py, slider_controller.launch.py

    ## 7. File cấu hình liên quan
    config/robot_controllers.yaml, config/arm_controller.yaml, config/gripper_controller.yaml, config/initial_positions.yaml

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_control
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_control controllers.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_control`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Không có phụ thuộc MoveIt/action đặc biệt được xác định từ source.
