# Gazebo Migration Report

## 1. Gazebo cũ trong `robot_description`

| Nhóm | Vị trí cũ |
|---|---|
| Launch Gazebo | `robot_description/launch/gazebo.launch.py`, `robot_description/launch/gazebo_new.launch.py` |
| Script spawn/random object | `robot_description/gazebo/random_wood_blocks.py`, `wood_blocks_3.py`, `checker_board.py`, `box_obstacle.py`, `spawn_pick_box.py` |
| World/model/SDF/assets | `robot_description/worlds/table`, `worlds/wood_block`, `worlds/checker_board`, `worlds/box`, `worlds/pick_box_3cm` |
| Robot URDF/xacro | `robot_description/urdf/*.xacro` |

Các file cũ trong `robot_description` được giữ nguyên để backward compatibility.

## 2. Package mới `robot_gazebo`

Đã tạo package ROS2 độc lập `robot_gazebo` dùng `ament_cmake`.

| File/thư mục | Nội dung |
|---|---|
| `CMakeLists.txt` | Cài `launch`, `worlds`, `models`, `config` và script Python |
| `package.xml` | Khai báo dependency một chiều sang `robot_description`, `ros_gz_sim`, `ros_gz_bridge`, `launch_ros`, `xacro`, `robot_state_publisher`, `rclpy` |
| `launch/gazebo.launch.py` | Port launch Gazebo chính |
| `launch/gazebo_new.launch.py` | Port launch Gazebo biến thể có box obstacle |
| `gazebo/*.py` | Script spawn/random object đã đổi package share sang `robot_gazebo` |
| `worlds/` | Copy world/model/SDF/assets từ `robot_description/worlds` |
| `models/`, `config/` | Thư mục mở rộng cho asset/config Gazebo sau này |
| `README.md` | Hướng dẫn package mới |

## 3. File đã copy/port từ `robot_description`

- `launch/gazebo.launch.py`
- `launch/gazebo_new.launch.py`
- `gazebo/random_wood_blocks.py`
- `gazebo/wood_blocks_3.py`
- `gazebo/checker_board.py`
- `gazebo/box_obstacle.py`
- `gazebo/spawn_pick_box.py`
- Toàn bộ `worlds/`

## 4. File vẫn tham chiếu từ `robot_description`

- `urdf/robot.urdf.xacro`
- Các xacro được include bởi robot model như `gazebo.xacro`, `ros2_control.xacro`, `sensors.xacro`, `materials.xacro`
- Mesh/URDF robot trong `robot_description`

`robot_gazebo` phụ thuộc vào `robot_description`; `robot_description` không phụ thuộc ngược lại `robot_gazebo`.

## 5. Cập nhật `robot_bringup`

| File | Thay đổi |
|---|---|
| `robot_bringup/launch/sim.launch.py` | Include `robot_gazebo/launch/gazebo.launch.py` thay vì `robot_description/launch/gazebo.launch.py` |
| `robot_bringup/launch/drl_test.launch.py` | Include Gazebo từ `robot_gazebo` |
| `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py` | Chạy `spawn_pick_box.py` từ package `robot_gazebo` |
| `robot_bringup/package.xml` | Thêm dependency `robot_gazebo` |
| `robot_bringup/README.md`, `LAUNCH.md`, `PARAMETERS.md` | Cập nhật mô tả package Gazebo mới |

## 6. Lệnh build đã chạy

```bash
cd ~/ros2_dev
colcon build --packages-select robot_description robot_gazebo robot_bringup --symlink-install
```

Kết quả: thành công.

```text
Summary: 3 packages finished
```

## 7. Lệnh kiểm tra package

```bash
source install/setup.bash
ros2 pkg list | grep robot_gazebo
ros2 pkg prefix robot_gazebo
```

Kết quả:

```text
robot_gazebo
/home/minhquang/ros2_dev/install/robot_gazebo
```

## 8. Lệnh launch cần test

```bash
ros2 launch robot_gazebo gazebo.launch.py
ros2 launch robot_bringup sim.launch.py
```

Kết quả:

- `ros2 launch robot_gazebo gazebo.launch.py`: Gazebo, `robot_state_publisher`, `ros_gz_bridge`, robot spawn và `random_wood_blocks.py` khởi động được; object SDF được lấy từ `install/robot_gazebo/share/robot_gazebo/worlds/...`.
- `ros2 launch robot_bringup sim.launch.py`: entrypoint chính include Gazebo từ `robot_gazebo`; Gazebo/MoveIt/task servers khởi động; `joint_state_broadcaster`, `arm_controller`, `gripper_controller` đều configured/activated sau khi tăng `--switch-timeout`.

## 9. Lỗi còn tồn tại

- RViz/MoveIt có log `Action server: /recognize_objects not available` khi chạy `robot_bringup sim.launch.py`. Đây là lỗi/cảnh báo runtime từ RViz perception plugin khi không có object recognition action server, không phải lỗi path/package Gazebo.
- URDF parser vẫn cảnh báo material `gripper_l`/`gripper_r` undefined như trước migration.

## 10. Danh sách file đã sửa/tạo

- Tạo mới package `robot_gazebo/`
- Sửa `robot_bringup/launch/sim.launch.py`
- Sửa `robot_bringup/launch/drl_test.launch.py`
- Sửa `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`
- Sửa `robot_bringup/package.xml`
- Sửa `robot_bringup/launch/sim.launch.py` để include `robot_gazebo` và tăng `--switch-timeout` cho controller spawner
- Cập nhật tài liệu ngắn trong `robot_bringup` và `robot_description`
