    # robot_moveit

    ## 1. Vai trò package
    MoveIt configuration package cho robot: SRDF, kinematics, controller mapping, joint limits, RViz config và launch move_group/RViz/GUI.

    ## 2. Vị trí trong hệ thống
    Cung cấp `move_group` và config cho task manager, task executor, DRL executor, GUI và bringup.

    ## 3. Thành phần chính
    - `config/robot.srdf`: group/state/EEF MoveIt.
- `config/kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, `pilz_cartesian_limits.yaml`, `sensors_3d.yaml`.
- `config/moveit.rviz`: RViz config.
- `launch/moveit.launch.py`, `moveit_gui.launch.py`, `moveit_mock.launch.py`.
- `gui/moveit_gui.py`: GUI Python thử nghiệm gọi MoveGroup/ExecuteTrajectory action.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Không build C++ executable; cài script `moveit_gui.py`; launch chạy `move_group`, `rviz2`, `static_transform_publisher`, `robot_gui_node` tùy file. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | Xem mô tả bên dưới | runtime interface | package dependent |

    Ghi chú interface: MoveIt actions `/move_action` và `/execute_trajectory` từ move_group, service query planner, `/joint_states`, controller action từ ros2_control.

    ## 6. File launch liên quan
    moveit.launch.py, moveit_gui.launch.py, moveit_mock.launch.py

    ## 7. File cấu hình liên quan
    config/robot.srdf, config/kinematics.yaml, config/joint_limits.yaml, config/moveit_controllers.yaml, config/pilz_cartesian_limits.yaml, config/sensors_3d.yaml, config/moveit.rviz

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_moveit
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_moveit moveit.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_moveit`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Đây là runtime MoveIt trung tâm. Plan-only/execute do client quyết định: task servers dùng MoveGroupInterface, GUI Python dùng MoveGroup/ExecuteTrajectory action.
