# robot_gazebo

Package `robot_gazebo` chứa phần mô phỏng Gazebo đã được tách ra từ `robot_description`: launch Gazebo, world/model SDF và các script spawn object.

## Nội dung chính

| Thành phần | Vai trò |
|---|---|
| `launch/gazebo.launch.py` | Launch Gazebo chính, spawn robot, bridge camera/clock, spawn demo wood blocks tùy argument |
| `launch/gazebo_new.launch.py` | Biến thể launch có spawn wood và box obstacle |
| `gazebo/*.py` | Script spawn/random object trong Gazebo |
| `worlds/` | Table world, wood block, checker board, pick box và box obstacle SDF/assets |
| `models/`, `config/` | Thư mục dành cho Gazebo model/config bổ sung |

## Quan hệ với `robot_description`

Robot URDF/xacro vẫn nằm trong `robot_description` và được launch này tham chiếu bằng `get_package_share_directory("robot_description")`. World, model object và script spawn đã được copy sang package này và được tham chiếu bằng `get_package_share_directory("robot_gazebo")`.

Các file Gazebo cũ trong `robot_description` được giữ lại tạm thời để backward compatibility và sẽ chỉ nên xóa sau khi validation riêng.

## Build

```bash
cd ~/ros2_dev
colcon build --packages-select robot_description robot_gazebo robot_bringup --symlink-install
source install/setup.bash
```

## Chạy nhanh

```bash
ros2 launch robot_gazebo gazebo.launch.py
ros2 launch robot_bringup sim.launch.py
```
