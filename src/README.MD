# ROS2 Workspace (src)

Tài liệu tóm tắt workspace ROS2 hiện tại nằm trong thư mục `src`. README này mô tả cấu trúc các package chính, cách build và các lệnh khởi chạy phổ biến.

## Tổng quan

Workspace chứa nhiều package cho robot, perception, launch/bringup, và công cụ phát triển. Nội dung chính gồm:
- camera drivers (ví dụ `astra_camera`)
- message packages (`astra_camera_msgs`, ...)
- vision / perception pipelines (`gp7_vision_pipeline`, `robot_vision_pipeline`)
- robot description, bringup, controllers (`robot_description`, `robot_bringup`, `robot_hardware_interface`, `robot_control`)
- GUI và công cụ (ví dụ `robot_gui`)

Mục tiêu: dễ build với `colcon`, dễ launch simulation hoặc phần cứng tùy package.

## Các package/ thư mục đáng chú ý

- `astra_camera/` — driver và hỗ trợ cho camera Astra (launch, config, libs)
- `astra_camera_msgs/` — định nghĩa message cho camera Astra
- `calibrationdata/` — file calibration và dữ liệu liên quan
- `gp7_vision_pipeline/` — pipeline vision (YOLO, mapper, nodes, models)
- `robot_bringup/` — launch và cấu hình để khởi động robot (sim/real)
- `robot_control/` — cấu hình ros2_control, controller YAML
- `robot_description/` — URDF/meshes, gazebo assets, rviz
- `robot_gui/` — GUI (PySide6), launcher và scripts
- `robot_hardware_interface/` — interface tới phần cứng, plugin
- `robot_moveit/` — (nếu có) cấu hình MoveIt cho robot
- `robot_vision_pipeline/` — bổ sung pipelines cho perception
- `robot_task_manager/`, `robot_rl/`, `robot_vision_pipeline/` — các công cụ tác vụ, RL, testing

Lưu ý: Một số package có tên nội bộ tương tự nhưng khác chức năng; kiểm tra thư mục tương ứng để biết chi tiết.

## Cấu trúc thư mục (tóm tắt)

Root của workspace (tương ứng với `src` ở workspace hiện tại):

```
/src/
├── robot_description/          # URDF, xacro, mesh, camera mount
├── robot_moveit_config/        # SRDF, MoveIt config, controllers config
├── robot_hardware_interface/   # ros2_control hardware plugin, RS485, Ezi-Servo
├── robot_interfaces/           # msg/srv/action dùng chung
├── robot_vision_pipeline/      # YOLO, ArUco, depth, pose estimation
├── robot_task_manager/         # action server: move_to_pose, gripper, pick_place
├── robot_bringup/              # launch tổng hợp toàn hệ thống
├── robot_gui/                  # GUI điều khiển, hiển thị camera, gọi action
├── robot_calibration/          # calib camera, hand-eye, lưu ma trận
└── robot_rl/                   # train/test SAC, policy, environment
```



## Build (quick start)

1. Thiết lập môi trường ROS2 ():

```bash
source /opt/ros/jazzy/setup.bash
```

2. Build bằng `colcon` từ thư mục chứa `src` (lưu ý: chạy tại workspace root nếu có):

```bash
cd <workspace_root>
colcon build --symlink-install
source install/setup.bash
```

Ghi chú: nếu chỉ muốn build một package cụ thể, dùng `--packages-select <pkg_name>`.

## Một số lệnh launch thường dùng

- Launch bringup (kiểm tra package `robot_bringup` để biết tên launch cụ thể):

```bash
ros2 launch robot_bringup main.launch.py
```

- Khởi simulation nếu có package gazebo tương ứng (ví dụ `robot_gazebo` / `gp7_gazebo_classic`):

```bash
ros2 launch gp7_gazebo_classic sim_moveit.launch.py
```

- Start camera / vision pipelines (ví dụ `astra_camera`, `gp7_vision_pipeline`):

```bash
ros2 launch astra_camera astra.launch.xml
ros2 launch gp7_vision_pipeline vision_full_pipeline.launch.py
```

Kiểm tra file `launch/` trong từng package để biết tham số `mode:=sim|real` hoặc tên launch chính xác.

## Hướng dẫn nhanh kiểm tra nội dung package

- Liệt kê package trong `src`:

```bash
ls -1
```

- Kiểm tra `package.xml` hoặc `CMakeLists.txt` để biết dependencies và entry points.

## Ghi chú và next steps

- Nếu bạn muốn, tôi có thể:
  - tạo README chi tiết hơn cho từng package (theo danh sách package),
  - thêm hướng dẫn build riêng cho phần hardware vs simulation,
  - hoặc tự động sinh một file `workspace_index.md` liệt kê tất cả package và mô tả ngắn.

File README đã được cập nhật tại [README.MD](README.MD).


| Parameter | File | Sim | Real |
|-----------|------|-----|------|
| `max_velocity_scale` | `motion_scaling.yaml` | `0.5` | `0.1` |
| `max_accel_scale` | `motion_scaling.yaml` | `0.5` | `0.1` |
| `FRAME_Z_OFFSET` | `config.py` | `0.330` m | `0.330` m |
| `ACTION_STEP` | `config.py` | `0.01` m | `0.01` m |

### Unit Conventions

| Quantity | Unit | File |
|----------|------|------|
| URDF joint limits | **radians** | `arm.xacro` |
| `initial_positions.yaml` | **radians** | `gp7_description/config/` |
| `joint_waypoints.yaml` | **degrees** | `gp7_task_executor/config/` |
| `pose_waypoints.yaml` | meters + **radians** | `gp7_task_executor/config/` |
| `cartesian_points.yaml` | meters | `gp7_task_executor/config/` |

## 14. Troubleshooting

### Gazebo spawn fails or robot jitter
**Cause:** `initial_positions.yaml` contains degree values instead of **radians**.
**Fix:** Convert all values. Example: `-43.32°` → `-43.32 × π/180 = -0.7561 rad`.

### MoveIt planning fails silently
Verify `planning_pipelines` only contains `ompl`:
```bash
ros2 param get /move_group planning_pipelines
```

### DRL trajectory offset from robot
**Cause:** Mixing BASE and WORLD/DRL coordinate frames. The 0.330 m Z offset must be applied before model inference and removed before execution.
**Fix:** Audit all `_base` vs `_drl` variable suffixes.

### `/move_to_named_target` fails with "target not found"
The `target_name` field is a plain `string`, not a nested message:
```
Wrong:  '{target_name: {data: start}}'
Right: "'{target_name: \\'start\\'}'"
```

### Real robot not responding
```bash
echo $ROS_DOMAIN_ID   # must be 1
ros2 topic list | grep robot1  # should list /robot1/joint_states
```

### Vision markers not visible in RViz
- Set Fixed Frame to `base_link` (not `world`)
- Verify: `ros2 topic echo /vision/detection_markers --once`
- Check publisher: `ros2 topic info /vision/detection_markers -v`
  Expected: `/vision_detection_marker_node`

## 15. Safety Notes for Real Robot Operation

> **Physical robot motion will occur. Ensure all personnel are clear of the robot workspace before running any launch file.**

1. **Start in teach mode** — ensure MotoROS2 `servo_on1` has been run before launching.
2. **Velocity scaling** — real robot uses `0.1` (10% of max). Never increase beyond `0.3` without supervisor approval.
3. **ROS_DOMAIN_ID** — must match the MotoROS2 configuration (default `1`). A mismatch causes the robot to appear frozen.
4. **Emergency stop** — keep the physical e-stop within reach. ROS 2 has no real-time safety guarantees.
5. **No direct joint streaming** — all motion must go through MoveIt 2. Do not bypass MoveIt by publishing to `arm_controller` or `/robot1/follow_joint_trajectory`.
6. **DRL execution** — always execute trajectories in a supervised manner. Use plan-only mode first.

## 16. Development Notes

### Coordinate Frames

| Frame | Description |
|-------|-------------|
| `base_link` | Kinematic root of the robot. Z=0 is at the robot base mounting surface. |
| `tool0` | End-effector frame at the TCP. TCP offset from `link_tool`: +0.088 m along Z. |
| `WORLD/DRL` | PyBullet training frame. Z=0 is at the table surface. BASE Z = WORLD/DRL Z + 0.330 m. |

### TCP Calibration (calibrated at "start" waypoint)

```
base_link → tool0: Translation = (0.350, -0.330, 0.060) m
             RPY = (-3.142, 0.0, 0.0) rad
```

### URDF Joint Names

`joint_1`, `joint_2`, `joint_3`, `joint_4`, `joint_5`, `joint_6`

### Git Commit Style

```
feat(gp7_task_executor): add move_cartesian_pose_sequence service
fix(gp7_vision_pipeline): correct homography matrix units
docs(gp7_gazebo_classic): document sim_camera launch args
test(gp7_drl_inference): add trajectory execution test
```

## Quick Reference

```bash
# Build
colcon build --symlink-install && source install/setup.bash

# Simulation
ros2 launch gp7_bringup main.launch.py mode:=sim

# Real robot
export ROS_DOMAIN_ID=1
ros2 launch gp7_bringup main.launch.py mode:=real

# Task executor
ros2 launch gp7_task_executor task_executor.launch.py
ros2 service call /move_to_named_target gp7_task_executor_msgs/srv/MoveToNamedTarget \
  "{target_name: 'start', execute: true}"

# DRL
ros2 launch gp7_drl_inference main.launch.py mode:=sim vision:=mock
ros2 service call /drl/execute_forward std_srvs/srv/Trigger {}

# GUI
ros2 launch gp7_gui gui.launch.py

# Inspect
ros2 topic list
ros2 service list
ros2 run tf2_ros tf2_echo base_link tool0
ros2 control list_controllers
```