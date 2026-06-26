# Task Servers Launch Fix Report

Ngày thực hiện: 2026-06-26

> Ghi chú cập nhật: report này ghi lại yêu cầu cũ khi default tạm thời là
> `enable_drl_backend=false`. Theo nhiệm vụ mới trong
> `move_pose_rl_wait_services_fix_report.md`, default hiện tại đã đổi lại thành
> `enable_drl_backend=true` để `/move_pose_rl` có backend DRL ngay khi chạy
> `task_servers.launch.py`.

## Nguyên nhân lỗi

`task_servers.launch.py` có sử dụng `LaunchConfiguration("enable_drl_backend")` trong launch description, nhưng thứ tự khai báo khiến launch có thể resolve biến này trước khi argument được đăng ký. Ngoài ra default trước đó đang bật DRL backend, không phù hợp với mục tiêu chạy task server cơ bản để test GUI/mock hardware.

Lỗi quan sát:

```text
launch configuration 'enable_drl_backend' does not exist
```

## File đã sửa

```text
src/robot_task_manager/launch/task_servers.launch.py
```

## Argument đã cập nhật

```python
DeclareLaunchArgument(
    "enable_drl_backend",
    default_value="false",
    ...
)
```

`enable_drl_backend_arg`, `planner_node_name_arg` và `drl_calibrated_start_tcp_arg` được đưa lên trước các action dùng `LaunchConfiguration` trong `LaunchDescription`.

## Default sau sửa

```text
enable_drl_backend=false
```

DRL backend chỉ bật khi chạy:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=true
```

## Kết quả build

Lệnh:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_task_manager --symlink-install
```

Kết quả:

```text
Summary: 1 package finished
```

Ghi chú: build có cảnh báo deprecation cũ từ `rosidl_target_interfaces()` trong `CMakeLists.txt`, không phải lỗi launch.

## Test 1: chạy không truyền argument

Lệnh:

```bash
ROS_DOMAIN_ID=41 ros2 launch robot_task_manager task_servers.launch.py
```

Kết quả:

```text
[task_servers] Starting action servers; enable_drl_backend=false
```

Launch chạy được, không còn lỗi thiếu launch configuration.

## Test 2: chạy explicit false

Lệnh:

```bash
ROS_DOMAIN_ID=42 ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

Kết quả:

```text
[task_servers] Starting action servers; enable_drl_backend=false
```

Launch chạy được, không lỗi.

## Action server xuất hiện

Sau khi chạy launch, `ros2 action list` thấy:

```text
/drl_pickplace
/gohome
/move_action
/move_checker_board
/move_gripper
/move_pose_rl
/move_to_pose
/move_to_pose_cartesian
/pickplace
/repeatability_test
```

Các action cơ bản yêu cầu đều có:

```text
/move_to_pose
/move_to_pose_cartesian
/move_gripper
/pickplace
/move_checker_board
/repeatability_test
```

## Ghi chú DRL

`/drl_pickplace` và `/move_pose_rl` vẫn được launch để GUI/action list đầy đủ, nhưng với default `enable_drl_backend=false` thì backend DRL không chạy.

Kiểm tra:

```bash
ros2 service info /drl/plan
```

Kết quả khi backend tắt:

```text
Type: std_srvs/srv/Trigger
Clients count: 2
Services count: 0
```

Muốn test DRL thật cần bật backend:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=true
```

## Ghi chú shutdown

Khi dừng launch bằng Ctrl-C, một số node MoveIt hiện có in `RCLError` trong destructor do ROS context đã invalid. Hiện tượng này xảy ra ở pha shutdown và không liên quan tới lỗi launch configuration đã sửa.
