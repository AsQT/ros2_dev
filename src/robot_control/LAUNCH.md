    # robot_control - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | controllers.launch.py | Khởi tạo ros2_control stack | robot_state_publisher, ros2_control_node, joint_state_broadcaster/arm/gripper spawner | Cần xacro robot_description và controller YAML |
| slider_controller.launch.py | Test điều khiển bằng slider | joint_state_publisher_gui và slider_controller.py từ package ngoài | Cần package `panda_controller` nếu dùng |

    ## 2. Chi tiết từng launch file
    ### controllers.launch.py

#### Chức năng
Khởi tạo ros2_control stack

#### Node được khởi tạo
robot_state_publisher, ros2_control_node, joint_state_broadcaster/arm/gripper spawner

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa mô tả robot/hardware và MoveIt/task execution. `robot_moveit` và `robot_bringup` include launch controller từ package này.

#### Điều kiện thực thi
Cần xacro robot_description và controller YAML

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_control controllers.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### slider_controller.launch.py

#### Chức năng
Test điều khiển bằng slider

#### Node được khởi tạo
joint_state_publisher_gui và slider_controller.py từ package ngoài

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa mô tả robot/hardware và MoveIt/task execution. `robot_moveit` và `robot_bringup` include launch controller từ package này.

#### Điều kiện thực thi
Cần package `panda_controller` nếu dùng

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_control slider_controller.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

