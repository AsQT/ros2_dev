# Báo cáo lỗi: drl_mock_hw.launch.py

**File**: `src/robot_drl/launch/drl_mock_hw.launch.py`
**Mục tiêu**: Chạy full stack mock hardware (robot model + MoveIt + DRL planning + execution) mà không cần Gazebo
**Trạng thái**: CÓ 3 LỖI NGHIÊM TRỌNG chưa sửa được

---

## Lỗi 1: `arm_controller` spawner FAIL

### Triệu chứng
```
[ERROR] [controller_manager]: The 'type' param was not defined for 'arm_controller'.
[FATAL] [spawner_arm_controller]: Failed loading controller arm_controller
[ERROR] [spawner_arm_controller]: process has died [pid XXXXX, exit code 1]
```

### Nguyên nhân gốc
- `joint_state_broadcaster` và `gripper_controller` spawn THÀNH CÔNG
- Chỉ `arm_controller` FAIL
- `joint_state_broadcaster` spawner dùng chung `robot_controllers.yaml` (chứa 3 controllers) → spawn OK
- `arm_controller` spawner cũng dùng `robot_controllers.yaml` → spawn FAIL vì `type` bị override
- `gripper_controller` spawner dùng chung file → spawn OK (không hiểu tại sao)

### Cách đã thử
1. Dùng `controllers.launch.py` đã test trước đó → `arm_controller` VẪN FAIL
2. Tạo file riêng `arm_controller.yaml` và `gripper_controller.yaml` → **CHƯA test**

### Hướng giải quyết tiếp theo
- Cần đổi spawner arguments để KHÔNG dùng `--param-file` mà chỉ dùng `--controller-manager`
- Hoặc truyền param `type` trực tiếp qua command line
- Hoặc kiểm tra `GenericSystem` có joint nào không có state interface

---

## Lỗi 2: `use_sim_time` type mismatch

### Triệu chứng
```
rclcpp::exceptions::InvalidParameterTypeException:
parameter 'use_sim_time' has invalid type: Wrong parameter type,
parameter {use_sim_time} is of type {bool}, setting it to {string} is not allowed.
```

### Nguyên nhân gốc
- `MoveItConfigsBuilder.robot_description()` thêm `use_sim_time` vào parameter dict với giá trị string `"false"` (từ xacro mapping)
- ROS 2 rclcpp yêu cầu `bool`, không nhận `string`
- Node bị ABORT ngay khi khởi động

### Đã sửa (chưa test)
- Thêm hàm `_make_params()` strip `use_sim_time` string và prepend `bool` ở đầu param list
- **CHƯA xác nhận** có hoạt động đúng sau khi rebuild

---

## Lỗi 3: `manual_default_target` type mismatch

### Triệu chứng
```
rclpy.exceptions.InvalidParameterTypeException:
Trying to set parameter 'manual_default_target' to '0.5750.0500.120'
of type 'STRING', expecting type 'DOUBLE_ARRAY'
```

### Nguyên nhân gốc
- `drl_unified_planner_node` khai báo `manual_default_target` là `float[]` (double array)
- Launch file truyền `LaunchConfiguration("target_x") + LaunchConfiguration("target_y") + ...` → thành string
- String bị merge: `"0.575" + "0.050" + "0.120"` = `"0.5750.0500.120"`

### Đã sửa
- Bỏ `manual_default_target` khỏi launch params
- DRL node sẽ dùng default trong code: `[0.575, 0.050, 0.120]`
- Hoặc dùng vision mode để đọc target từ `mock_environment_node`

---

## Tình trạng sau sửa

| Component | Trước sửa | Sau sửa | Test? |
|-----------|-----------|---------|-------|
| `use_sim_time` bool fix | FAIL | Sửa rồi | CHƯA |
| `manual_default_target` type | FAIL | Sửa rồi | CHƯA |
| `arm_controller` spawner | FAIL | Đã tạo arm_controller.yaml | CHƯA |
| `gripper_controller` spawner | OK | OK | RỒI |
| `joint_state_broadcaster` | OK | OK | RỒI |
| `move_group` MoveIt | OK | OK | RỒI |
| `task_executor_node` | OK | OK | RỒI |
| `mock_environment_node` | OK | OK | RỒI |
| `drl_unified_planner_node` | FAIL | Sửa rồi | CHƯA |
| `rviz2` | OK | OK | RỒI |

---

## Các file đã tạo/sửa

```
src/robot_drl/launch/drl_mock_hw.launch.py  (viết lại hoàn toàn)
src/robot_control/config/arm_controller.yaml  (mới)
src/robot_control/config/gripper_controller.yaml  (mới)
```

---

## Cách test tiếp theo

```bash
# 1. Rebuild cả robot_control và robot_drl
cd /home/minhquang/ros2
source install/setup.bash
colcon build --packages-select robot_control robot_drl robot_task_executor --symlink-install

# 2. Kill tất cả process cũ
pkill -9 -f "ros2_control\|spawner\|robot_state\|move_group\|task_executor\|drl\|mock\|rviz" 2>/dev/null

# 3. Chạy
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
ros2 launch robot_drl drl_mock_hw.launch.py \
    auto_plan_on_start:=true \
    manual_prompt_on_start:=false \
    input_mode:=manual
```

---

## Câu hỏi cần trả lời

1. Tại sao `arm_controller` spawn fail nhưng `gripper_controller` spawn OK dùùng cùng file YAML và cùng controller type?
2. Hàm `_make_params()` có thực sự fix được `use_sim_time` bool không?
3. `GenericSystem` mock hardware có hỗ trợ `velocity` command interface không?
