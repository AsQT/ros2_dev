    # robot_hardware_interface - Launch Guide

    ## 1. Danh sách launch file
    | Launch file | Mục đích | Node được chạy | Điều kiện cần |
    |---|---|---|---|
    | hardware_interface.launch.py | Chạy hardware node standalone | robot_hw_node | Cần cấu hình IP/port hoặc transport trong params; robot thật phải online |

    ## 2. Chi tiết từng launch file
    ### hardware_interface.launch.py

#### Chức năng
Chạy hardware node standalone

#### Node được khởi tạo
robot_hw_node

#### Argument
Xem `PARAMETERS.md`; các argument được scan từ `DeclareLaunchArgument` trong source.

#### Parameter truyền vào node
Xem `PARAMETERS.md` và YAML/config liên quan nếu có.

#### Package phụ thuộc
Tầng sát phần cứng. Nhận lệnh từ GUI/controller_manager, publish joint state/status và cung cấp service connect/servo/home/jog/run/stop.

#### Điều kiện thực thi
Cần cấu hình IP/port hoặc transport trong params; robot thật phải online

#### Lệnh chạy
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_hardware_interface hardware_interface.launch.py
```

#### Lỗi thường gặp
- Chưa `source install/setup.bash` sau khi build.
- Package include hoặc node phụ thuộc chưa được build/cài.
- Với MoveIt/Gazebo/hardware: controller, `move_group`, Gazebo hoặc robot thật chưa sẵn sàng.

