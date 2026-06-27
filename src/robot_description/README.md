    # robot_description

    ## 1. Vai trò package
Package mô tả hình học, URDF/Xacro, frame, mesh, world Gazebo và script spawn object cho robot.

Ghi chú migration: Gazebo launch/world/script đã được tách sang package `robot_gazebo`. Các file Gazebo cũ trong package này vẫn được giữ để backward compatibility và chưa bị xóa.

    ## 2. Vị trí trong hệ thống
    Là nguồn `robot_description` cho controller, MoveIt, RViz và Gazebo; các package khác không nên tự định nghĩa frame robot.

    ## 3. Thành phần chính
    - `urdf/robot.urdf.xacro`, `robot.xacro`: mô tả chính.
- `urdf/ros2_control.xacro`: tag ros2_control/mock/real hardware.
- `urdf/gazebo.xacro`, `sensors.xacro`, `materials.xacro`: Gazebo/sensor/material.
- `meshes/`: hình học visual/collision.
- `worlds/*/*.sdf`: table, wood block, checker board, pick box; bản mới phục vụ bringup nằm ở `robot_gazebo/worlds`.
- `gazebo/*.py`: spawn random wood/checker board/pick box; bản mới phục vụ bringup nằm ở `robot_gazebo/gazebo`.
- `launch/display.launch.py`, `gazebo.launch.py`, `gazebo_new.launch.py`: hiển thị và simulation.

    ## 4. Node / executable
    | Node / executable | Nguồn | Vai trò |
    |---|---|---|
    | Không có executable C++ riêng; cài các script Python Gazebo: `random_wood_blocks.py`, `wood_blocks_3.py`, `checker_board.py`, `spawn_pick_box.py`. | package source/launch | Runtime của package hoặc node được launch |

    ## 5. Topic / Service / Action
    | Interface | Type | Vai trò |
    |---|---|---|
    | Xem mô tả bên dưới | runtime interface | package dependent |

    Ghi chú interface: Xuất `robot_description` parameter, TF từ `robot_state_publisher`, bridge Gazebo qua `ros_gz_bridge`, topic thông tin object từ script spawn khi có.

    ## 6. File launch liên quan
    display.launch.py, gazebo.launch.py, gazebo_new.launch.py

    ## 7. File cấu hình liên quan
    config/joint_names_robot.yaml, rviz/config.rviz, urdf/*.xacro, worlds/*/*.sdf

    ## 8. Cách build riêng package
    ```bash
    cd ~/ros2_dev
    colcon build --packages-select robot_description
    source install/setup.bash
    ```

    ## 9. Cách chạy nhanh
    ```bash
    source ~/ros2_dev/install/setup.bash
    ros2 launch robot_description display.launch.py
    ```
    Nếu package không có launch/runtime node, dùng nó bằng cách phụ thuộc interface hoặc include file từ package khác.

    ## 10. Ghi chú kỹ thuật / giới hạn hiện tại
    - Source of truth là source code hiện tại trong `robot_description`.
    - Tài liệu này không thay thế kiểm tra runtime bằng `ros2 node list`, `ros2 topic list`, `ros2 service list`, `ros2 action list` sau khi launch.
    - Không có phụ thuộc MoveIt/action đặc biệt được xác định từ source.
