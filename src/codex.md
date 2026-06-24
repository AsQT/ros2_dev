Hãy kiểm tra toàn bộ package `robot_drl` để xác định các phần liên quan đến DRL planning, reward, inference runtime, path generation và path execution.

Bối cảnh lỗi hiện tại:

* RL planner có hiện tượng sinh quỹ đạo đi xuống thấp hơn `target_pos.z` rồi mới đi lên lại.
* Quỹ đạo có xu hướng gấp khúc / đổi hướng không mượt.
* Cần phân biệt rõ:

  * Reward chỉ dùng trong training/evaluation.
  * ROS runtime/action server không nên viết lại reward để điều khiển.
  * ROS runtime cần có path safety validator để chặn path xấu trước khi execute.

Mục tiêu kiểm tra:

1. Xác định trong `robot_drl` có phần training environment hay không.
2. Xác định reward function hiện đang nằm ở đâu, có được dùng trong ROS runtime không.
3. Xác định ROS action/server/service nào đang load SAC model và gọi `policy.predict()`.
4. Xác định path/waypoint được sinh ra ở đâu.
5. Xác định path được execute ở đâu.
6. Kiểm tra hiện tại có safety validator cho path chưa.
7. Kiểm tra hiện tại có kiểm tra Z thấp hơn target hoặc Z an toàn chưa.
8. Kiểm tra hiện tại có kiểm tra gấp khúc / sharp turn / smoothness chưa.
9. Kiểm tra có fallback/replan khi path không hợp lệ chưa.
10. Kiểm tra log/debug hiện tại có đủ để biết path bị tụt Z hay gấp khúc chưa.

Các file/thư mục cần rà soát:

* Toàn bộ `robot_drl`
* Các file Python node/action/server liên quan đến DRL planner
* Các file launch liên quan đến DRL
* Các file config YAML liên quan đến model, workspace, planner, obstacle, reward nếu có
* Các test hiện có trong package
* `package.xml`, `setup.py`, `setup.cfg`, `CMakeLists.txt` nếu có
* README hoặc tài liệu liên quan nếu có

Yêu cầu kiểm tra chi tiết:

## 1. Phân loại architecture hiện tại

Hãy lập sơ đồ luồng xử lý hiện tại của `robot_drl`:

```text
ROS action/service input
        ↓
build observation
        ↓
load SAC model / policy.predict()
        ↓
generate action / waypoint / path
        ↓
validate path nếu có
        ↓
execute path / gọi MoveToPoseCartesian / controller / action khác
```

Cần ghi rõ:

* File nào thực hiện từng bước.
* Class/function nào chịu trách nhiệm.
* Input/output chính của từng bước.
* Có dùng reward trong runtime không.

Kết luận rõ:

* Reward hiện tại có nằm trong `robot_drl` không?
* Reward có bị dùng nhầm trong ROS runtime không?
* Nếu không có training environment trong `robot_drl`, không được thêm reward vào runtime chỉ để sửa đường đi.

## 2. Kiểm tra vấn đề Z target

Hãy tìm toàn bộ nơi dùng:

* `target_pos`
* `current_pos`
* `z`
* `target_z`
* `workspace`
* `workspace_z_min`
* `min_z`
* `waypoint`
* `path`
* `trajectory`

Cần kiểm tra:

* Khi policy sinh action/path, có ràng buộc `waypoint.z >= z_min_allowed` không.
* `z_min_allowed` hiện có được tính từ `target_pos.z`, workspace, table height, object height, hay không có.
* Có trường hợp RL được phép sinh waypoint thấp hơn target không.
* Có clamp Z hoặc reject path khi tụt Z không.
* Có log `min_z_in_path` không.

Đề xuất logic đúng cho runtime:

```python
z_min_allowed = target_pos[2] - z_guard_margin

for i, waypoint in enumerate(path):
    if waypoint[2] < z_min_allowed:
        reject path
```

Không hard-code hệ số, nên đưa vào config nếu cần:

```yaml
planner:
  safety_check_enabled: true
  z_guard_enabled: true
  z_guard_mode: "target"
  z_guard_margin: 0.005
  reject_z_violation_path: true
```

## 3. Kiểm tra vấn đề quỹ đạo gấp khúc

Hãy kiểm tra path hiện tại có được làm mượt hoặc validate hình học chưa.

Cần tìm:

* Có kiểm tra góc giữa 2 đoạn liên tiếp không.
* Có kiểm tra khoảng cách giữa 2 waypoint liên tiếp không.
* Có interpolate/smoothing không.
* Có giới hạn action delta không.
* Có lọc waypoint bất thường không.

Đề xuất runtime validator:

```python
for i in range(1, len(path) - 1):
    v0 = path[i] - path[i - 1]
    v1 = path[i + 1] - path[i]

    if norm(v0) < eps or norm(v1) < eps:
        continue

    cos_angle = dot(v0, v1) / (norm(v0) * norm(v1))
    angle_deg = arccos(clamp(cos_angle, -1.0, 1.0)) * 180.0 / pi

    if angle_deg > max_turn_angle_deg:
        reject path
```

Config đề xuất:

```yaml
planner:
  sharp_turn_check_enabled: true
  max_turn_angle_deg: 120.0
  reject_sharp_turn_path: true
  max_waypoint_step: 0.05
```

## 4. Kiểm tra reward/training nếu có trong package

Nếu trong `robot_drl` có training environment hoặc reward function, hãy kiểm tra reward hiện tại có các component sau chưa:

```text
R_success
R_collision
R_distance
R_workspace
R_episode
R_time
R_shake
R_z_guard
R_smooth_action
R_path_curvature
```

Nếu thiếu thì đề xuất bổ sung trong training env/reward, không đưa reward vào ROS runtime.

Reward Z guard đề xuất:

```python
z_error = max(0.0, z_min_allowed - current_pos[2])
R_z_guard = 0.0
if z_error > 0.0:
    R_z_guard -= z_violation_penalty
    R_z_guard -= z_guard_scale * (z_error ** 2)
```

Smooth action reward đề xuất:

```python
action_delta = action - prev_action
R_smooth_action = -smooth_action_scale * np.linalg.norm(action_delta) ** 2
```

Path curvature reward đề xuất nếu có position history:

```python
v1 = current_pos - prev_pos
v0 = prev_pos - prev_prev_pos
curvature = np.linalg.norm(v1 - v0)
R_path_curvature = -path_curvature_scale * curvature ** 2
```

Nhưng chỉ sửa reward nếu `robot_drl` thực sự chứa training/evaluation environment. Nếu package chỉ chạy inference trong ROS thì không thêm reward vào runtime.

## 5. Thêm hoặc đề xuất `PathSafetyValidator`

Nếu hiện tại chưa có safety validator, hãy implement tối thiểu một module/class riêng, ví dụ:

```text
robot_drl/path_safety_validator.py
```

Chức năng:

* Kiểm tra Z violation.
* Kiểm tra workspace violation nếu có workspace config.
* Kiểm tra sharp turn.
* Kiểm tra max waypoint step.
* Trả về kết quả rõ ràng gồm:

  * `is_valid`
  * `reason`
  * `invalid_index`
  * `min_z_in_path`
  * `z_min_allowed`
  * `max_turn_angle`
  * `max_step_distance`

Không trộn validator trực tiếp vào logic action server nếu có thể tách riêng.

## 6. Tích hợp validator vào runtime

Tìm nơi path được sinh ra ngay trước khi execute. Thêm luồng:

```text
RL planner sinh path
        ↓
PathSafetyValidator.validate(path, target_pos, workspace)
        ↓
Nếu valid:
    execute
Nếu invalid:
    không execute
    log lỗi rõ ràng
    trả result failed cho action/service
    hoặc fallback/replan nếu project đã có cơ chế này
```

Không được execute path khi:

* Có waypoint thấp hơn Z target trừ margin.
* Có góc gấp quá giới hạn.
* Có waypoint ngoài workspace.
* Có đoạn nhảy waypoint quá dài.

## 7. Bổ sung log/debug

Khi planning xong, log các giá trị:

```text
number_of_waypoints
start_pos
target_pos
min_z_in_path
z_min_allowed
z_guard_margin
is_z_violation
z_violation_index
max_turn_angle
sharp_turn_index
max_step_distance
path_valid
reject_reason
```

Nếu có action result hoặc service response phù hợp, trả thêm reason khi path bị reject.

## 8. Test bắt buộc

Tạo hoặc cập nhật test cho `robot_drl`:

### Test Z validator

* Path hợp lệ khi mọi waypoint.z >= target.z - margin.
* Path invalid khi có waypoint.z < target.z - margin.
* Kết quả phải trả đúng invalid index và reason.

### Test sharp turn validator

* Path thẳng hoặc cong nhẹ hợp lệ.
* Path có góc gấp lớn hơn `max_turn_angle_deg` bị reject.

### Test max step distance

* Path có đoạn waypoint nhảy quá dài bị reject.

### Test integration

* Mock một path xấu từ planner.
* Đảm bảo runtime không gọi execute khi validator fail.
* Đảm bảo runtime log reason rõ ràng.

## 9. Build/test

Sau khi kiểm tra hoặc sửa:

* Chạy test Python nếu có.
* Chạy `colcon build` cho package liên quan.
* Chạy launch/action test bằng mock_hw nếu project có.
* Không phá các action/service API hiện có.
* Không đổi observation/action space nếu không thật sự bắt buộc.
* Không rewrite toàn bộ package.
* Không hard-code hệ số trong code nếu có thể đưa vào YAML/config.

## 10. Báo cáo kết quả

Tạo file:

```text
robot_drl_safety_reward_audit.md
```

Nội dung bắt buộc:

1. Sơ đồ luồng xử lý hiện tại của `robot_drl`.
2. Danh sách file/class/function liên quan.
3. Kết luận reward nằm ở đâu và có được dùng trong ROS runtime không.
4. Kết luận có cần sửa reward trong `robot_drl` không.
5. Kết luận hiện có path safety validator chưa.
6. Các lỗi/rủi ro tìm thấy:

   * Z tụt dưới target.
   * Gấp khúc path.
   * Thiếu workspace check.
   * Thiếu fallback/replan.
   * Thiếu log.
7. Các thay đổi đã thực hiện nếu có.
8. Các config mới nếu có.
9. Kết quả test/build.
10. Việc còn lại nếu chưa thể xử lý hết.

Kết luận kỹ thuật mong muốn:

* Nếu `robot_drl` chỉ chạy inference ROS runtime: không thêm reward runtime, chỉ thêm safety validator.
* Nếu `robot_drl` có training env: thêm reward Z/smoothness trong training env, sau đó cần train/fine-tune lại model.
* Dù model đã train tốt, ROS runtime vẫn phải có validator để không execute path nguy hiểm.
