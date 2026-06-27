    # robot_drl - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | main.launch.py | Pipeline DRL tổng hợp với simulation và tùy chọn vision/mock | Include sim, mock_environment hoặc vision_full_pipeline; chạy drl_unified_planner_node | Cần model DRL và các package bringup/vision |
| drl_unified_planner.launch.py | Chạy planner trên sim bringup | Include robot_bringup/sim.launch.py; chạy drl_unified_planner_node | Cần task servers và executor sẵn sàng |
| mock_environment.launch.py | Publish target/box mock | mock_environment_node | Không cần camera thật |
| mock_drl.launch.py | Planner + mock target không RViz | mock_environment_node, drl_unified_planner_node | Cần executor nếu auto execute |
| mock_drl_rviz.launch.py | Mock DRL kèm RViz marker | mock_environment_node, drl_unified_planner_node, rviz2 | Cần rviz config |
| drl_mock_hw.launch.py | Mock hardware + MoveIt + DRL executor | Include robot_moveit/moveit_mock, robot_drl_executor; chạy mock env/planner | Cần MoveIt mock |
| drl_gazebo.launch.py | Gazebo + DRL executor + planner | Include robot_bringup/sim, robot_drl_executor | Cần Gazebo |
| rl_sim_rviz.launch.py | Simulation DRL có RViz | Include sim, executor, rviz | Cần Gazebo/MoveIt |
| drl_mock_hw_obstacle_test.launch.py | Test obstacle trên mock hardware | Include drl_mock_hw; chạy mock_hw_obstacle_test | Cần service `/drl/*` |
| drl_gazebo_obstacle_test.launch.py | Test obstacle trên Gazebo | Include drl_gazebo; chạy gazebo_obstacle_test | Cần Gazebo ổn định |
| rviz_drl.launch.py | Chỉ mở RViz DRL | rviz2 | Cần topic marker nếu muốn xem dữ liệu |

    ## 2. Chi tiết từng launch file
    ### main.launch.py

#### Chức năng
Pipeline DRL tổng hợp với simulation và tùy chọn vision/mock

#### Node được khởi tạo
Include sim, mock_environment hoặc vision_full_pipeline; chạy drl_unified_planner_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần model DRL và các package bringup/vision

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl main.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### drl_unified_planner.launch.py

#### Chức năng
Chạy planner trên sim bringup

#### Node được khởi tạo
Include robot_bringup/sim.launch.py; chạy drl_unified_planner_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần task servers và executor sẵn sàng

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl drl_unified_planner.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### mock_environment.launch.py

#### Chức năng
Publish target/box mock

#### Node được khởi tạo
mock_environment_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Không cần camera thật

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl mock_environment.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### mock_drl.launch.py

#### Chức năng
Planner + mock target không RViz

#### Node được khởi tạo
mock_environment_node, drl_unified_planner_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần executor nếu auto execute

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl mock_drl.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### mock_drl_rviz.launch.py

#### Chức năng
Mock DRL kèm RViz marker

#### Node được khởi tạo
mock_environment_node, drl_unified_planner_node, rviz2

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần rviz config

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl mock_drl_rviz.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### drl_mock_hw.launch.py

#### Chức năng
Mock hardware + MoveIt + DRL executor

#### Node được khởi tạo
Include robot_moveit/moveit_mock, robot_drl_executor; chạy mock env/planner

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần MoveIt mock

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl drl_mock_hw.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### drl_gazebo.launch.py

#### Chức năng
Gazebo + DRL executor + planner

#### Node được khởi tạo
Include robot_bringup/sim, robot_drl_executor

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần Gazebo

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl drl_gazebo.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### rl_sim_rviz.launch.py

#### Chức năng
Simulation DRL có RViz

#### Node được khởi tạo
Include sim, executor, rviz

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần Gazebo/MoveIt

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl rl_sim_rviz.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### drl_mock_hw_obstacle_test.launch.py

#### Chức năng
Test obstacle trên mock hardware

#### Node được khởi tạo
Include drl_mock_hw; chạy mock_hw_obstacle_test

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần service `/drl/*`

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl drl_mock_hw_obstacle_test.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### drl_gazebo_obstacle_test.launch.py

#### Chức năng
Test obstacle trên Gazebo

#### Node được khởi tạo
Include drl_gazebo; chạy gazebo_obstacle_test

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần Gazebo ổn định

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl drl_gazebo_obstacle_test.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.
### rviz_drl.launch.py

#### Chức năng
Chỉ mở RViz DRL

#### Node được khởi tạo
rviz2

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Nằm giữa perception/mock target và executor MoveIt. Nhận target/box từ mock hoặc vision, gọi service planning scene/IK/Cartesian executor, và được `robot_task_manager` gọi qua `/drl/*`.

#### Điều kiện thực thi
Cần topic marker nếu muốn xem dữ liệu

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl rviz_drl.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

