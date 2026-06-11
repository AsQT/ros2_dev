# Robot DRL — Mock Hardware Testing Guide

Hướng dẫn chạy package `robot_drl` với **mock hardware** (không cần robot thật, không cần Gazebo).

ros2 launch robot_drl drl_mock_hw.launch.py

---

## Mục lục

1. [Tổng quan](#1-tổng-quan)
2. [Kiểm tra môi trường](#2-kiểm-tra-môi-trường)
3. [Build package](#3-build-package)
4. [Cách chạy nhanh](#4-cách-chạy-nhanh)
5. [Các tùy chọn chạy](#5-các-tùy-chọn-chạy)
6. [Visualize kết quả trong RViz2](#6-visualize-kết-quả-trong-rviz2)
7. [Các services & topics của DRL node](#7-các-services--topics-của-drl-node)
8. [Xử lý lỗi thường gặp](#8-xử-lý-lỗi-thường-gặp)

---

## 1. Tổng quan

Stack mock chạy **2 node**:

| Node | Mô tả |
|------|--------|
| `mock_environment_node` | Publish synthetic target pose + box detection ở 10 Hz (thay thế camera thật) |
| `drl_unified_planner_node` | Load model DRL, chạy inference, publish trajectory markers |

**Không có**: Gazebo, MoveIt, robot_bringup, hay bất kỳ hardware thật nào.

Model được dùng: **TD3** (10 triệu steps), đặt tại `models/run/model/best_model.zip`.

---

## 2. Kiểm tra môi trường

### 2.1 Kiểm tra các package đã được build

```bash
ls /home/minhquang/ros2/install/robot_drl/share/robot_drl/models/
```

Output mong đợi:
```
DEPLOYMENT_GUIDE.md  README.md  run  run1
```

### 2.2 Kiểm tra virtual environment

Package `robot_drl` cần các thư viện Python từ venv `ros_rl`:

```bash
/home/minhquang/venvs/ros_rl/bin/python -c "import gymnasium; import stable_baselines3; print('OK')"
```

Output mong đợi:
```
OK
```

Nếu lỗi `ModuleNotFoundError`, đảm bảo `PYTHONPATH` trỏ đúng vào venv.

### 2.3 Kiểm tra ROS 2 environment

```bash
source /home/minhquang/ros2/install/setup.bash
ros2 pkg list | grep -E "robot_drl|robot_task_executor_msgs|robot_vision_pipeline"
```

Output mong đợi:
```
robot_drl
robot_task_executor_msgs
robot_vision_pipeline
```

---

## 3. Build package

Mỗi khi thay đổi code hoặc thêm file launch mới, rebuild:

```bash
cd /home/minhquang/ros2
source install/setup.bash
colcon build --packages-select robot_drl --symlink-install
```

Build thành công sẽ hiển thị:
```
Summary: 1 package finished [X.XXs]
```

---

## 4. Cách chạy nhanh

### Cách 1 — Một command duy nhất + RViz (khuyên dùng)

```bash
cd /home/minhquang/ros2
source install/setup.bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl mock_drl_rviz.launch.py \
    input_mode:=manual \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false
```

Launch này khởi động đồng thời **3 thành phần**:
1. `mock_environment_node` — publish synthetic target pose ở 10 Hz
2. `drl_unified_planner_node` (sau 3s) — DRL planning
3. `rviz2` (sau 5s) — visualize trajectory markers

RViz2 sẽ tự động mở với config `drl_markers.rviz` đã cấu hình sẵn.

### Cách 2 — Không mở RViz (chạy node thô)

```bash
cd /home/minhquang/ros2
source install/setup.bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl mock_drl.launch.py \
    input_mode:=manual \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false
```

Dùng cách này khi muốn mở RViz thủ công ở terminal riêng.

---

## 5. Các tùy chọn chạy

### 5.1 Target tùy chỉnh

```bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl mock_drl_rviz.launch.py \
    input_mode:=manual \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    target_x:=0.600 \
    target_y:=-0.100 \
    target_z:=0.200
```

### 5.2 Target class khác

```bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl mock_drl_rviz.launch.py \
    input_mode:=manual \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    target_class_name:=target
```

### 5.3 Vision mode (đọc từ mock env)

```bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl mock_drl_rviz.launch.py \
    input_mode:=vision \
    auto_plan_on_start:=true
```

Ở chế độ `vision`, node sẽ đọc target từ topic `/vision/target_position` (do mock_environment publish).

### 5.4 Start position khác

```bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl mock_drl_rviz.launch.py \
    input_mode:=manual \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    calibrated_start_tcp_base:="[0.600, 0.100, 0.300]"
```

### 5.5 Chạy từng node riêng lẻ (không dùng launch file)

```bash
# Terminal 1: mock environment
source install/setup.bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 run robot_drl mock_environment_node --ros-args \
    -p target_x:=0.575 \
    -p target_y:=0.050 \
    -p target_z:=0.120 \
    -p target_class_name:=box

# Terminal 2: DRL planner (manual, no auto-plan)
source install/setup.bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 run robot_drl drl_unified_planner_node --ros-args \
    -p input_mode:=manual \
    -p auto_plan_on_start:=false

# Terminal 3: RViz (mở thủ công)
rviz2 -d /home/minhquang/ros2/install/robot_drl/share/robot_drl/rviz/drl_markers.rviz
```

---

## 6. Visualize kết quả trong RViz2

### 6.1 Mở RViz2 với config có sẵn

Nếu dùng `mock_drl_rviz.launch.py`, RViz2 đã được khởi động tự động.

Nếu dùng `mock_drl.launch.py` (không kèm RViz), mở thủ công:

```bash
source /home/minhquang/ros2/install/setup.bash
rviz2 -d /home/minhquang/ros2/install/robot_drl/share/robot_drl/rviz/drl_markers.rviz
```

### 6.2 Cấu hình RViz thủ công (nếu không dùng file config)

Trong RViz2, thực hiện:

1. **Global Options** → **Fixed Frame**: `base_link`
2. **Add** → **By topic** → **`/drl/forward_trajectory_marker`** (MarkerArray) → OK
3. **Add** → **By topic** → **`/drl/backward_trajectory_marker`** (MarkerArray) → OK
4. **Add** → **By topic** → **`/drl/forward_trajectory_poses`** (PoseArray) → OK

### 6.3 Nội dung hiển thị

RViz config `drl_markers.rviz` đã cấu hình sẵn:

| Display | Topic | Nội dung |
|---------|-------|-----------|
| Grid | — | Lưới XY plane |
| TF | — | Hiển thị các reference frames |
| RobotModel | `/robot_description` | Mô hình robot (sẽ ẩn nếu không có robot_description) |
| **DRL Forward Trajectory** | `/drl/forward_trajectory_marker` | Đường xanh dương + quả cầu xanh lục (start) + đỏ (target) |
| **DRL Backward Trajectory** | `/drl/backward_trajectory_marker` | Đường tím + quả cầu cam (target) + xanh lục (start) |
| **DRL Forward Poses** | `/drl/forward_trajectory_poses` | Mũi tên từng waypoint |

### 6.4 Chú thích màu sắc

| Element | Màu | Ý nghĩa |
|---------|------|---------|
| Quả cầu xanh lục | `(0,1,0)` | Start TCP |
| Quả cầu đỏ | `(1,0,0)` | Target position |
| Đường xanh dương | `(0,0.7,1)` | Forward trajectory |
| Quả cầu cam | `(1,0.6,0)` | Target (backward start) |
| Đường tím | `(0.8,0.2,0.8)` | Backward trajectory |
| Khối vàng nhạt | `(1,0.4,0, 0.4 alpha)` | Obstacle |
| Cylinder xanh dương | `(0,0.2,1)` | Target block marker |

### 6.5 Screenshot ví dụ kết quả

Sau khi plan thành công, bạn sẽ thấy trong RViz:

```
┌─────────────────────────────────────────────────────┐
│  RViz2 — drl_markers.rviz                          │
│                                                     │
│     [Green ●]────────→[Red ●]                      │
│      Start TCP    Forward     Target                 │
│                    (blue line)                      │
│                                                     │
│     [Orange ●]←───────[Green ●]                    │
│     Backward           Backward                     │
│     (purple line)      Start                       │
│                                                     │
│  Displays:                                         │
│    ✓ DRL Forward Trajectory   /drl/forward...      │
│    ✓ DRL Backward Trajectory  /drl/backward...     │
│    ✓ DRL Forward Poses        /drl/forward_poses   │
│    ✓ Grid                      XY Plane             │
│    ✓ TF                                             │
│  Fixed Frame: base_link                             │
└─────────────────────────────────────────────────────┘
```

### 6.6 Lưu / Load cấu hình RViz

Sau khi cấu hình xong, lưu lại để lần sau không phải setup lại:

- **File** → **Save Config As**
- Lưu vào thư mục workspace, ví dụ: `/home/minhquang/ros2/rviz/mock_drl.rviz`

Chạy lại với config đã lưu:
```bash
rviz2 -d /home/minhquang/ros2/rviz/mock_drl.rviz
```

---

## 7. Các Services & Topics của DRL Node

### 7.1 Services

| Service | Kiểu | Mô tả |
|---------|------|--------|
| `/drl/plan` | `std_srvs/Trigger` | Trigger planning (manual: đọc target từ terminal; vision: dùng latest vision data) |
| `/drl/replan` | `std_srvs/Trigger` | Alias cho `/drl/plan` |
| `/drl/execute_forward` | `std_srvs/Trigger` | Execute trajectory forward (sẽ fail nếu không có robot thật) |
| `/drl/execute_backward` | `std_srvs/Trigger` | Execute trajectory backward |
| `/drl/execute_trajectory` | `std_srvs/Trigger` | Alias cho `execute_forward` |
| `/drl/clear_trajectory` | `std_srvs/Trigger` | Xóa trajectory hiện tại |
| `/drl/get_execution_status` | `std_srvs/Trigger` | Kiểm tra trạng thái execution |

Gọi service từ terminal:
```bash
# Re-plan
ros2 service call /drl/plan std_srvs/srv/Trigger

# Clear trajectory
ros2 service call /drl/clear_trajectory std_srvs/srv/Trigger

# Get status
ros2 service call /drl/get_execution_status std_srvs/srv/Trigger
```

### 7.2 Topics

| Topic | Kiểu | Mô tả |
|-------|------|--------|
| `/drl/forward_trajectory_marker` | `MarkerArray` | Trajectory forward: đường + spheres |
| `/drl/backward_trajectory_marker` | `MarkerArray` | Trajectory backward: đường + spheres |
| `/drl/forward_trajectory_poses` | `PoseArray` | Trajectory forward dạng PoseArray |
| `/drl/backward_trajectory_poses` | `PoseArray` | Trajectory backward dạng PoseArray |
| `/drl/execution_status` | `String` | Trạng thái execution (2 Hz) |
| `/drl/next_pose` | `PoseStamped` | Waypoint đang execute |
| `/detected_object/pose` | `PoseStamped` | Target pose từ mock env |
| `/vision/box_detection` | `BoxDetection` | Bounding box từ mock env |

Kiểm tra topics:
```bash
# List all DRL topics
ros2 topic list | grep drl

# Echo forward trajectory markers
ros2 topic echo /drl/forward_trajectory_marker --once

# Echo execution status
ros2 topic echo /drl/execution_status
```

### 7.3 Parameters của DRL Node

| Parameter | Default | Mô tả |
|-----------|---------|--------|
| `input_mode` | `manual` | `manual` hoặc `vision` |
| `auto_plan_on_start` | `true` | Tự động plan khi node khởi động |
| `manual_prompt_on_start` | `true` | Prompt terminal khi auto-plan |
| `auto_execute_after_plan` | `false` | Tự động execute sau khi plan |
| `use_sim_time` | `false` | Dùng simulation time |
| `calibrated_start_tcp_base` | `[0.5241, 0.0, 0.315]` | Vị trí start TCP (m) |
| `manual_default_target` | `[0.575, 0.050, 0.120]` | Target mặc định |
| `manual_default_obstacle_center` | `[0.550, 0.0, 0.120]` | Obstacle center mặc định |
| `manual_default_obstacle_size` | `[0.1, 0.1, 0.1]` | Obstacle size mặc định |

---

## 8. Xử lý lỗi thường gặp

### Lỗi 1: `ModuleNotFoundError: No module named 'gymnasium'`

**Nguyên nhân**: Python không tìm thấy thư viện.

**Khắc phục**: Thêm `PYTHONPATH` vào venv:

```bash
export PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH
```

Hoặc thêm vào `~/.bashrc` để không phải gõ lại mỗi lần:

```bash
echo 'export PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### Lỗi 2: `VecNormalize: file not found`

**Nguyên nhân**: File `vec_normalize_stats.pkl` không tồn tại trong `models/run/model/`.

**Khắc phục**: Hoạt động bình thường — node sẽ dùng raw observations thay vì normalized. Model vẫn chạy được.

### Lỗi 3: `/move_cartesian_pose_sequence service not available`

**Nguyên nhân**: Service này cần `robot_task_executor` đang chạy (cần robot thật hoặc MoveIt).

**Khắc phục**: Đây là **hành vi bình thường** khi chạy mock-only. Planning vẫn hoạt động, chỉ có execute là không có robot để thực thi. Xem kết quả trong RViz.

### Lỗi 4: Trajectory không hiện trong RViz

**Khắc phục**:
1. Kiểm tra Fixed Frame = `base_link`
2. Kiểm tra topics đã được subscribe đúng: `/drl/forward_trajectory_marker`
3. Chạy `ros2 topic list | grep drl` để xác nhận topics có dữ liệu

### Lỗi 5: Model loading failed

**Kiểm tra** model file tồn tại:
```bash
ls /home/minhquang/ros2/install/robot_drl/share/robot_drl/models/run/model/best_model.zip
```

Nếu không tồn tại, copy từ source:
```bash
ls /home/minhquang/ros2/src/robot_drl/models/run/model/best_model.zip
# Sau đó rebuild:
colcon build --packages-select robot_drl --symlink-install
```

### Lỗi 6: Node bị crash ngay khi khởi động

**Kiểm tra** log:
```bash
tail -100 /root/.ros/log/$(ls -t /root/.ros/log/ | head -1)/ros2.log
```

---

## 9. Full Simulation Test (Gazebo + MoveIt + DRL + Robot Execution)

For a complete end-to-end test with a robot model, motion planning, and physical execution in Gazebo.

### 9.1 Prerequisites

All required packages must be built:

```bash
cd /home/minhquang/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Verify packages
ros2 pkg list | grep -E "robot_bringup|robot_drl|robot_task_executor|robot_moveit|robot_description"
```

### 9.2 Quick Start

```bash
cd /home/minhquang/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_bringup drl_test.launch.py \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    auto_execute_after_plan:=true
```

### 9.3 What Gets Launched

The `drl_test.launch.py` starts **5 components in sequence**:

| Step | Component | Delay | Description |
|------|-----------|-------|-------------|
| 1 | Gazebo + gz_ros2_control + bridges | 0s | Robot sim, controller manager, camera bridges |
| 2 | MoveIt (move_group) | 5s | Motion planning via OMPL |
| 3 | Task executor node | 9s | `/move_cartesian_pose_sequence` service |
| 4 | DRL planner node | 13s | DRL inference + trajectory visualization |
| 5 | RViz | 15s | Trajectory markers + robot model |

### 9.4 Key Services

The full stack provides:

| Service | From | Description |
|---------|------|-------------|
| `/move_cartesian_pose_sequence` | `robot_task_executor` | Execute Cartesian pose sequences via MoveIt |
| `/drl/execute_forward` | `drl_unified_planner_node` | Execute forward trajectory |
| `/drl/execute_backward` | `drl_unified_planner_node` | Execute backward trajectory |
| `/drl/plan` | `drl_unified_planner_node` | Trigger DRL planning |
| `/drl/clear_trajectory` | `drl_unified_planner_node` | Clear trajectory markers |

### 9.5 Launch Arguments

```bash
# Auto-plan + auto-execute (recommended for testing)
ros2 launch robot_bringup drl_test.launch.py \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    auto_execute_after_plan:=true

# Plan only (execute manually via service call)
ros2 launch robot_bringup drl_test.launch.py \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    auto_execute_after_plan:=false

# Then execute manually:
ros2 service call /drl/execute_forward std_srvs/srv/Trigger

# Custom start position and target
ros2 launch robot_bringup drl_test.launch.py \
    calibrated_start_tcp_base:="[0.5241, 0.000, 0.315]" \
    target_x:=0.575 target_y:=0.050 target_z:=0.120
```

### 9.6 Expected Log Output

Successful execution shows:

```
[drl_unified_planner_node]: [manual] Trajectory planning complete | converged=True | planning_time=0.0654s
[drl_unified_planner_node]: [execute_forward] starting background execution
[task_executor_node]: /move_cartesian_pose_sequence: 21 poses, execute=1
[task_executor_node]: [/move_cartesian_pose_sequence] computeCartesianPath fraction=0.xxx
[move_group]: Completed trajectory execution with status SUCCEEDED
[drl_unified_planner_node]: [forward] background execution finished | status=SUCCEEDED_FORWARD
```

### 9.7 Known Issues

**TF time jump warnings**: The log will show many `[tf2_buffer]: Detected jump back in time` warnings. This is cosmetic — caused by the Gazebo simulation clock resetting during startup. The system still works correctly.

---

## Thông tin thêm

- **Model algorithm**: TD3 (Twin Delayed DDPG)
- **Training steps**: 10,000,000
- **Action step**: 0.01 m mỗi step
- **Success threshold**: 0.02 m
- **Workspace**: x=[0.425, 0.675], y=[-0.2, 0.2], z=[0.02, 0.6] m
- **Planning time**: ~12-50 ms (tùy target/obstacle)
- **Default target**: (0.575, 0.050, 0.120) m
- **Default start TCP**: (0.5241, 0.000, 0.315) m

Để xem model details:
```bash
cat /home/minhquang/ros2/install/robot_drl/share/robot_drl/models/run/config.json
```
