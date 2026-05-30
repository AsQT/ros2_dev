# robot_reachability

ROS 2 + MoveIt 2 package dùng để quét vùng robot có thể tiếp cận.

Package này quét nhiều điểm `x, y, z` trong workspace, tại mỗi điểm thử nhiều hướng TCP:

- `roll`: xoay quanh trục X, mặc định từ `-45°` đến `45°`
- `pitch`: xoay quanh trục Y, mặc định từ `-45°` đến `45°`
- `yaw`: mặc định cố định `0°`

Sau đó node gọi MoveIt:

- `/compute_ik`
- `/check_state_validity`

Nếu pose có nghiệm IK và hợp lệ thì điểm đó được đánh dấu là reachable.

---

## 1. Cài vào workspace

Giải nén package vào `~/ros2/src`:

```bash
cd ~/ros2/src
unzip robot_reachability.zip
```

Build:

```bash
cd ~/ros2
colcon build --packages-select robot_reachability --symlink-install
source install/setup.bash
```

---

## 2. Chạy MoveIt trước

Trước khi chạy package này, phải launch robot + MoveIt của bạn sao cho có service:

```bash
ros2 service list | grep compute_ik
ros2 service list | grep check_state_validity
```

Cần thấy:

```text
/compute_ik
/check_state_validity
```

---

## 3. Chạy quét reachability

```bash
ros2 launch robot_reachability reachability_scan.launch.py
```

Hoặc chạy trực tiếp:

```bash
ros2 run robot_reachability reachability_scan_node --ros-args --params-file ~/ros2/src/robot_reachability/config/reachability_scan.yaml
```

---

## 4. Xem kết quả CSV

Mặc định file kết quả lưu ở:

```bash
/tmp/reachability_map.csv
```

Xem nhanh:

```bash
head /tmp/reachability_map.csv
```

Mở bằng LibreOffice:

```bash
libreoffice /tmp/reachability_map.csv
```

Các cột chính:

```text
x_m,y_m,z_m,valid_orientation_count,first_valid_roll_deg,first_valid_pitch_deg,yaw_deg,valid_roll_pitch_pairs_deg
```

Ý nghĩa:

- `x_m, y_m, z_m`: vị trí TCP trong `base_frame`
- `valid_orientation_count`: số cặp roll/pitch hợp lệ tại điểm đó
- `first_valid_roll_deg`, `first_valid_pitch_deg`: một hướng hợp lệ đầu tiên tìm được
- `valid_roll_pitch_pairs_deg`: danh sách các cặp `roll:pitch` hợp lệ

---

## 5. Xem điểm reachable trong RViz

Mở RViz:

```bash
rviz2
```

Thêm display:

```text
Add -> MarkerArray
```

Topic:

```text
/reachability_markers
```

Fixed Frame nên chọn giống `base_frame`, mặc định là:

```text
world
```

---

## 6. Chỉnh vùng quét

Mở file:

```bash
nano ~/ros2/src/robot_reachability/config/reachability_scan.yaml
```

Các thông số quan trọng:

```yaml
base_frame: "world"
group_name: "arm"
ik_link_name: "tcp_link"

x_min: 0.20
x_max: 0.75
y_min: -0.35
y_max: 0.35
z_min: 0.05
z_max: 0.55
xyz_step: 0.05

roll_min_deg: -45.0
roll_max_deg: 45.0
pitch_min_deg: -45.0
pitch_max_deg: 45.0
angle_step_deg: 15.0
yaw_deg: 0.0
```

Nếu muốn dùng cho pick/place, nên bật kiểm tra pose approach:

```yaml
check_approach: true
approach_offset_z: 0.10
```

Khi bật tùy chọn này, một điểm chỉ được xem là reachable nếu cả pose tại `z` và pose tại `z + 0.10 m` đều IK được.

---

## 7. Gợi ý cho robot gắp vật

Ban đầu nên quét thô:

```yaml
xyz_step: 0.05
angle_step_deg: 15.0
```

Sau khi xác định vùng tốt, quét mịn hơn:

```yaml
xyz_step: 0.025
angle_step_deg: 10.0
```

Điểm nào có `valid_orientation_count` càng lớn thì robot càng dễ tiếp cận, ít sát giới hạn khớp và ít lỗi IK hơn.
