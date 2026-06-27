    # robot_task_executor - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | task_executor.launch.py | Chạy service executor MoveIt | task_executor_node | Cần move_group, robot_description, controller và YAML waypoint hợp lệ |

    ## 2. Chi tiết từng launch file
    ### task_executor.launch.py

#### Chức năng
Chạy service executor MoveIt

#### Node được khởi tạo
task_executor_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Backend MoveIt service độc lập cho client hoặc DRL đời cũ; song song với `robot_drl_executor` và `robot_task_manager` action servers.

#### Điều kiện thực thi
Cần move_group, robot_description, controller và YAML waypoint hợp lệ

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_task_executor task_executor.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

