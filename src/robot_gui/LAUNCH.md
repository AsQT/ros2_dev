    # robot_gui - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | robot_gui.launch.py | Chạy GUI chính | robot_gui_node | Cần source workspace; cần task/hardware server nếu dùng điều khiển thật |
| embedded_rviz_cpp_test.launch.py | Test GUI với move_group và robot_state_publisher | robot_state_publisher, joint_state_publisher, move_group, robot_gui_node | Cần MoveIt config và robot_description |

    ## 2. Chi tiết từng launch file
    ### robot_gui.launch.py

#### Chức năng
Chạy GUI chính

#### Node được khởi tạo
robot_gui_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Tầng operator UI. Gọi service hardware từ `robot_hardware_interface`, publish trajectory controller trực tiếp, và gửi action đến `robot_task_manager`.

#### Điều kiện thực thi
Cần source workspace; cần task/hardware server nếu dùng điều khiển thật

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_gui robot_gui.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### embedded_rviz_cpp_test.launch.py

#### Chức năng
Test GUI với move_group và robot_state_publisher

#### Node được khởi tạo
robot_state_publisher, joint_state_publisher, move_group, robot_gui_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Tầng operator UI. Gọi service hardware từ `robot_hardware_interface`, publish trajectory controller trực tiếp, và gửi action đến `robot_task_manager`.

#### Điều kiện thực thi
Cần MoveIt config và robot_description

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_gui embedded_rviz_cpp_test.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

