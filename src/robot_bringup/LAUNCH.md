    # robot_bringup - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
| sim.launch.py | Simulation đầy đủ: Gazebo, MoveIt, task servers sim, joint/arm/gripper spawner | robot_gazebo/gazebo.launch.py, robot_moveit/moveit.launch.py, robot_task_manager/task_servers_sim.launch.py, controller_manager spawner | Cần Gazebo/ros_gz_sim và robot_control controllers |
| real.launch.py | Bringup real/mock hardware qua MoveIt GUI và task servers | Include robot_moveit/moveit_gui.launch.py với use_mock=false, robot_task_manager/task_servers.launch.py | Cần hardware interface/controller hoạt động nếu dùng robot thật |
| drl_test.launch.py | Test DRL planner trên simulation | Include gazebo, moveit, robot_drl_executor; chạy drl_unified_planner_node và rviz2 | Cần model DRL và service Cartesian executor |
| rl_pick_place_box_gazebo_demo.launch.py | Demo DRL pick-place hộp trong Gazebo | Include sim.launch.py, robot_drl_executor.launch.py; chạy spawn_pick_box.py, drl_unified_planner_node, demo client | Cần Gazebo world và task manager action `/drl_pickplace` |

    ## 2. Chi tiết từng launch file
    ### sim.launch.py

#### Chức năng
Simulation đầy đủ: Gazebo, MoveIt, task servers sim, joint/arm/gripper spawner

#### Node được khởi tạo
robot_gazebo/gazebo.launch.py, robot_moveit/moveit.launch.py, robot_task_manager/task_servers_sim.launch.py, controller_manager spawner

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Đứng ở tầng orchestration: include `robot_description`, `robot_control`, `robot_moveit`, `robot_task_manager`, `robot_drl`, `robot_drl_executor`, RViz/Gazebo tùy mode.

#### Điều kiện thực thi
Cần Gazebo/ros_gz_sim và robot_control controllers

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_bringup sim.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### real.launch.py

#### Chức năng
Bringup real/mock hardware qua MoveIt GUI và task servers

#### Node được khởi tạo
Include robot_moveit/moveit_gui.launch.py với use_mock=false, robot_task_manager/task_servers.launch.py

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Đứng ở tầng orchestration: include `robot_description`, `robot_control`, `robot_moveit`, `robot_task_manager`, `robot_drl`, `robot_drl_executor`, RViz/Gazebo tùy mode.

#### Điều kiện thực thi
Cần hardware interface/controller hoạt động nếu dùng robot thật

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_bringup real.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### drl_test.launch.py

#### Chức năng
Test DRL planner trên simulation

#### Node được khởi tạo
Include gazebo, moveit, robot_drl_executor; chạy drl_unified_planner_node và rviz2

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Đứng ở tầng orchestration: include `robot_description`, `robot_control`, `robot_moveit`, `robot_task_manager`, `robot_drl`, `robot_drl_executor`, RViz/Gazebo tùy mode.

#### Điều kiện thực thi
Cần model DRL và service Cartesian executor

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_bringup drl_test.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### rl_pick_place_box_gazebo_demo.launch.py

#### Chức năng
Demo DRL pick-place hộp trong Gazebo

#### Node được khởi tạo
Include sim.launch.py, robot_drl_executor.launch.py; chạy robot_gazebo/spawn_pick_box.py, drl_unified_planner_node, demo client

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Đứng ở tầng orchestration: include `robot_description`, `robot_control`, `robot_moveit`, `robot_task_manager`, `robot_drl`, `robot_drl_executor`, RViz/Gazebo tùy mode.

#### Điều kiện thực thi
Cần Gazebo world và task manager action `/drl_pickplace`

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
