    # robot_moveit - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | moveit.launch.py | MoveIt + RViz + static TF, optional controller manager | move_group, rviz2, static_transform_publisher; include robot_control/controllers.launch.py | Cần robot_description và controller config |
| moveit_gui.launch.py | MoveIt + robot_gui_node | move_group, static_transform_publisher, robot_gui_node; include controllers.launch.py | Cần Qt GUI dependency |
| moveit_mock.launch.py | MoveIt mock kèm RViz và task servers | move_group, rviz2, static TF; include controllers và task_servers | Phù hợp test không hardware thật |

    ## 2. Chi tiết từng launch file
    ### moveit.launch.py

#### Chức năng
MoveIt + RViz + static TF, optional controller manager

#### Node được khởi tạo
move_group, rviz2, static_transform_publisher; include robot_control/controllers.launch.py

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Cung cấp `move_group` và config cho task manager, task executor, DRL executor, GUI và bringup.

#### Điều kiện thực thi
Cần robot_description và controller config

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_moveit moveit.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### moveit_gui.launch.py

#### Chức năng
MoveIt + robot_gui_node

#### Node được khởi tạo
move_group, static_transform_publisher, robot_gui_node; include controllers.launch.py

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Cung cấp `move_group` và config cho task manager, task executor, DRL executor, GUI và bringup.

#### Điều kiện thực thi
Cần Qt GUI dependency

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_moveit moveit_gui.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### moveit_mock.launch.py

#### Chức năng
MoveIt mock kèm RViz và task servers

#### Node được khởi tạo
move_group, rviz2, static TF; include controllers và task_servers

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Cung cấp `move_group` và config cho task manager, task executor, DRL executor, GUI và bringup.

#### Điều kiện thực thi
Phù hợp test không hardware thật

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_moveit moveit_mock.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

