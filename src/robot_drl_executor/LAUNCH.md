    # robot_drl_executor - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | robot_drl_executor.launch.py | Chạy Cartesian executor cho DRL | robot_drl_executor_node | Cần `move_group` đang chạy và robot model/controller sẵn sàng |

    ## 2. Chi tiết từng launch file
    ### robot_drl_executor.launch.py

#### Chức năng
Chạy Cartesian executor cho DRL

#### Node được khởi tạo
robot_drl_executor_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Backend MoveIt cho `robot_drl`; cũng có thể được gọi trực tiếp bởi client qua service `/move_cartesian_pose_sequence`.

#### Điều kiện thực thi
Cần `move_group` đang chạy và robot model/controller sẵn sàng

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_drl_executor robot_drl_executor.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

