    # robot_description - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | display.launch.py | Xem robot trên RViz không cần Gazebo | robot_state_publisher, joint_state_publisher_gui, rviz2 | Cần xacro hợp lệ và mesh path đúng |
| gazebo.launch.py | Khởi động Gazebo world chính, spawn robot và wood demo | ros_gz_sim, robot_state_publisher, ros_gz_sim create, ros_gz_bridge, random_wood_blocks.py | Cần Gazebo/ros_gz_sim |
| gazebo_new.launch.py | Biến thể Gazebo có spawn wood/box obstacle | robot_state_publisher, create, bridge, random_wood_blocks.py, box_obstacle.py | Cần model SDF trong worlds |

    ## 2. Chi tiết từng launch file
    ### display.launch.py

#### Chức năng
Xem robot trên RViz không cần Gazebo

#### Node được khởi tạo
robot_state_publisher, joint_state_publisher_gui, rviz2

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Là nguồn `robot_description` cho controller, MoveIt, RViz và Gazebo; các package khác không nên tự định nghĩa frame robot.

#### Điều kiện thực thi
Cần xacro hợp lệ và mesh path đúng

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_description display.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### gazebo.launch.py

#### Chức năng
Khởi động Gazebo world chính, spawn robot và wood demo

#### Node được khởi tạo
ros_gz_sim, robot_state_publisher, ros_gz_sim create, ros_gz_bridge, random_wood_blocks.py

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Là nguồn `robot_description` cho controller, MoveIt, RViz và Gazebo; các package khác không nên tự định nghĩa frame robot.

#### Điều kiện thực thi
Cần Gazebo/ros_gz_sim

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### gazebo_new.launch.py

#### Chức năng
Biến thể Gazebo có spawn wood/box obstacle

#### Node được khởi tạo
robot_state_publisher, create, bridge, random_wood_blocks.py, box_obstacle.py

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Là nguồn `robot_description` cho controller, MoveIt, RViz và Gazebo; các package khác không nên tự định nghĩa frame robot.

#### Điều kiện thực thi
Cần model SDF trong worlds

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_description gazebo_new.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

