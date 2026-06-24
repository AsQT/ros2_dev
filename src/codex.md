Hãy kiểm tra package `robot_drl` và đối chiếu với file `struc.txt` mà tôi đã cung cấp. Nhiệm vụ hiện tại chỉ là **kiểm tra/audit và tạo báo cáo**, tuyệt đối **không chỉnh sửa code**, **không thêm reward mới**, **không đổi observation**, **không đổi action space**, **không đổi API/action/service**, **không rewrite package**.

Bối cảnh:

* Khi chạy thử model trong môi trường test/training thì không thấy hiện tượng Z đi xuống thấp hơn target rồi mới đi lên.
* Nhưng khi chạy qua ROS runtime/planning trong `robot_drl`, path có hiện tượng đi xuống thấp hơn `target_pos.z` rồi mới đi lên lại.
* Hiện chưa áp dụng reward mới. Việc thêm `R_z_guard`, `R_smooth_action`, `R_path_curvature` để sau.
* File `struc.txt` mô tả toàn bộ cấu trúc lúc train:

  * Training project là `FRAME_ONLY`, không phải robot joint/MoveIt/Gazebo.
  * Observation shape = 15.
  * Action shape = 3.
  * Action là delta Cartesian chuẩn hóa trong `[-1, 1]`.
  * Physical delta = `action * 0.01 m`.
  * `next_pos = current_pos + delta`.
  * Observation gồm:

    * index 0-2: `current_pos`
    * index 3-5: `target_pos`
    * index 6-8: `err = target_pos - current_pos`
    * index 9-11: `rel_obs`
    * index 12-14: `obs_size`
  * Không dùng `VecNormalize`.
  * Một phần observation là mét raw, obstacle feature được normalize theo workspace range.
  * Reward training hiện tại chưa có Z guard và chưa có path curvature reward.
  * Evaluation/predict dùng `deterministic=True` mặc định.
  * Target random Z trong training có ghi chú bị hard-code về `0.10`.

Mục tiêu kiểm tra:
Tìm nguyên nhân vì sao test model không tụt Z nhưng ROS runtime/planning lại có hiện tượng tụt Z. Cần xác định lỗi có khả năng nằm ở:

1. ROS runtime build observation sai thứ tự.
2. Sai đơn vị m/mm.
3. Sai frame tọa độ.
4. Sai `target_z`, ví dụ ROS dùng target pick thật thay vì pre-pick/pre-place.
5. Sai action scale so với training `0.01 m/step`.
6. Đảo dấu trục Z khi cộng action.
7. ROS runtime dùng `deterministic=False`.
8. ROS runtime load sai model.
9. ROS runtime xử lý normalization khác training.
10. Path sau policy đúng nhưng sau interpolation/smoothing/execution lại bị tụt Z.
11. Thiếu path safety validator trong runtime.

Yêu cầu quan trọng:

* Chỉ kiểm tra và báo cáo.
* Không sửa file code.
* Không thêm reward mới.
* Không thêm validator trong task này.
* Không thay đổi config.
* Nếu cần thêm log để kiểm tra, chỉ đề xuất vị trí cần log trong báo cáo, không tự sửa.
* Nếu muốn chạy lệnh build/test thì được, nhưng không thay đổi source.
* Không xóa, rename, format lại file.
* Không commit thay đổi.

## 1. Đọc và tóm tắt `struc.txt`

Trước tiên hãy đọc kỹ file `struc.txt` và rút ra các thông tin training quan trọng để đối chiếu với `robot_drl`:

```text
observation_dim
observation order
action_dim
action scale
next_pos update rule
deterministic mode khi evaluation
normalization/VecNormalize
target_z behavior
reward hiện tại
workspace range
model/checkpoint path nếu có
```

Trong báo cáo phải có mục:

```text
Training reference from struc.txt
```

Ghi rõ các điểm dùng để so sánh với ROS runtime.

## 2. Audit luồng ROS runtime trong `robot_drl`

Hãy kiểm tra toàn bộ package `robot_drl` và xác định luồng xử lý thực tế:

```text
ROS action/service input
        ↓
parse start/target/obstacle
        ↓
build observation
        ↓
load SAC model
        ↓
model.predict()
        ↓
scale action
        ↓
update current_pos / generate waypoint
        ↓
create policy path
        ↓
interpolate/smooth/resample nếu có
        ↓
execute / publish / gọi action khác
```

Trong báo cáo cần ghi rõ:

* File nào xử lý từng bước.
* Class/function nào xử lý từng bước.
* Input/output của từng bước.
* Có dùng reward trong ROS runtime không.
* Có gọi `model.predict(..., deterministic=True)` không.
* Có path validator không.
* Có interpolation/smoothing không.
* Có clamp Z/workspace không.

## 3. Kiểm tra model và inference mode

Kiểm tra:

* ROS runtime đang load model `.zip` nào.
* Test/evaluation model trong training dùng model nào.
* Có nguy cơ ROS load nhầm model cũ không.
* Có dùng `VecNormalize`/normalization file nào không.
* Vì `struc.txt` ghi training không dùng VecNormalize, kiểm tra ROS có đang normalize sai hoặc thiếu/nhầm normalize không.
* `model.predict()` trong ROS dùng `deterministic=True` hay `False`.

Báo cáo cần có bảng:

```text
Item | Training/Test reference | robot_drl runtime | Match? | Note
model path | ... | ... | yes/no | ...
VecNormalize | disabled | ... | yes/no | ...
deterministic | true | ... | yes/no | ...
```

Nếu ROS dùng `deterministic=False`, đánh dấu là rủi ro cao.

## 4. Kiểm tra observation build

Dựa theo `struc.txt`, observation đúng lúc training là:

```text
obs[0:3]   = current_pos
obs[3:6]   = target_pos
obs[6:9]   = target_pos - current_pos
obs[9:12]  = rel_obs = (obstacle_center - current_pos) / workspace_range
obs[12:15] = obs_size = obstacle_half_extent / workspace_range
```

Hãy kiểm tra trong `robot_drl`:

* Observation có đúng shape 15 không.
* Thứ tự index có giống training không.
* `current_pos`, `target_pos`, `err`, `rel_obs`, `obs_size` có đúng không.
* `obs_size` đang dùng full size hay half extent.
* Workspace range có giống training không.
* Đơn vị có phải mét không.
* Obstacle position có đúng frame không.
* Có nhầm index Z không.
* Có đưa target pick thật thay vì target approach không.

Báo cáo cần có bảng:

```text
Observation index | Training meaning | robot_drl runtime meaning | Match? | Risk
0-2 | current_pos | ... | yes/no | ...
3-5 | target_pos | ... | yes/no | ...
6-8 | target-current | ... | yes/no | ...
9-11 | rel_obs | ... | yes/no | ...
12-14 | obs_size half extent normalized | ... | yes/no | ...
```

Nếu không xác định được bằng code, ghi rõ “chưa xác định được” và chỉ ra file/function cần log thêm.

## 5. Kiểm tra action scale và update rule

Theo `struc.txt`, training dùng:

```python
delta = action * 0.01
next_pos = current_pos + delta
```

Hãy kiểm tra trong `robot_drl`:

* Có clip action về `[-1, 1]` không.
* Action scale có đúng `0.01 m/step` không.
* Scale theo Z có khác X/Y không.
* Có dùng đơn vị mm không.
* Có đảo dấu Z không.
* Có công thức sai kiểu:

```python
next_z = current_z - action_z * scale
```

thay vì:

```python
next_z = current_z + action_z * scale
```

Báo cáo cần kết luận rõ:

* Nếu `raw_action_z` đúng nhưng `next_z` sai: lỗi nằm ở scale/dấu/frame/update rule.
* Nếu `raw_action_z` đã khác test: lỗi nằm ở observation/model/target/deterministic.

## 6. Kiểm tra target Z

Hãy kiểm tra target truyền vào DRL planner trong `robot_drl` là:

* target pick/place thật, hay
* pre-pick/pre-place approach target.

Với pick-place, DRL runtime nên plan tới pose an toàn:

```text
pre_pick.z  = target_pick.z  + approach_height
pre_place.z = target_place.z + approach_height
```

Sau đó mới Cartesian đi thẳng xuống điểm pick/place thật.

Cần kiểm tra:

* Trong test model, target_z thường là bao nhiêu.
* Trong ROS runtime, target_z là bao nhiêu.
* Có offset approach height không.
* Có nhầm giữa target thật và target approach không.
* Có trường hợp ROS target_z thấp hơn đáng kể so với test không.
* Training từng có target_z hard-code 0.10, ROS có dùng target_z khác phân phối này không.

Báo cáo cần ghi rõ rủi ro nếu ROS runtime đưa target_z ngoài phân phối training.

## 7. Kiểm tra path policy, processed path và executed path

Phải phân biệt 3 loại path:

```text
policy_path:
    waypoint trực tiếp từ policy/action loop

processed_path:
    path sau interpolate/smooth/resample

executed_path:
    path cuối cùng gửi cho controller/action khác
```

Hãy kiểm tra:

* Policy path có tụt Z không.
* Nếu policy path không tụt Z, processed/executed path có tụt Z không.
* Có dùng spline/cubic/polynomial smoothing gây overshoot không.
* Có nội suy làm tạo waypoint dưới target_z không.
* Có convert frame gây đổi Z không.

Báo cáo cần có mục:

```text
Path stage comparison
```

với các metric cần tìm hoặc đề xuất log:

```text
policy_path_min_z
processed_path_min_z
executed_path_min_z
target_z
z_min_allowed = target_z - margin
min_z_index
```

Nếu hiện chưa có log để tính, ghi rõ cần thêm log ở file/function nào.

## 8. Kiểm tra sharp turn/gấp khúc

Chỉ kiểm tra, không sửa.

Hãy kiểm tra path hiện tại có thể tính được:

* `max_turn_angle_deg`
* `sharp_turn_index`
* `max_step_distance`
* `max_step_index`
* waypoint duplicate hoặc segment quá ngắn

Công thức góc:

```python
v0 = path[i] - path[i - 1]
v1 = path[i + 1] - path[i]

if norm(v0) > eps and norm(v1) > eps:
    cos_angle = dot(v0, v1) / (norm(v0) * norm(v1))
    angle_deg = arccos(clamp(cos_angle, -1.0, 1.0)) * 180.0 / pi
```

Nếu path không được lưu/log, báo cáo phải đề xuất vị trí cần log để tính các metric này.

## 9. Kiểm tra runtime safety validator hiện có hay chưa

Chỉ kiểm tra, không thêm.

Tìm xem `robot_drl` hiện có kiểm tra trước khi execute không:

* waypoint.z >= target_z - margin
* waypoint nằm trong workspace
* max step distance
* sharp turn angle
* obstacle collision
* fallback/replan khi path invalid

Báo cáo cần kết luận:

* Có validator chưa.
* Validator nằm ở file/function nào.
* Validator kiểm tra những gì.
* Thiếu kiểm tra gì.
* Nếu không có validator, ghi rõ đây là rủi ro runtime.

## 10. Kiểm tra reward trong ROS runtime

Yêu cầu xác định:

* ROS runtime có dùng reward không.
* Nếu có, reward dùng để làm gì.
* Có dùng reward để sửa action/path không.
* Có reward training copy sang runtime không.

Kết luận mong muốn:

* Nếu ROS chỉ inference thì không nên dùng reward để điều khiển runtime.
* Reward mới để sau, thuộc training/evaluation, không áp dụng trong task này.

## 11. Chạy kiểm tra nếu có thể

Có thể chạy các lệnh không làm thay đổi source:

```bash
colcon build --packages-select robot_drl
```

Nếu có test sẵn:

```bash
colcon test --packages-select robot_drl
colcon test-result --verbose
```

Nếu có script inference/test không ghi đè file, có thể chạy để lấy log. Nhưng không được train lại, không được ghi đè model.

Nếu chạy được case runtime/mock_hw, hãy ghi lệnh và kết quả. Nếu không chạy được, ghi rõ lý do.

## 12. Tạo báo cáo

Tạo file:

```text
robot_drl_runtime_z_audit_report.md
```

Nội dung bắt buộc:

### 1. Scope

* Chỉ audit/kiểm tra.
* Không sửa code.
* Không thêm reward.
* Không đổi observation/action.

### 2. Training reference from `struc.txt`

Tóm tắt các thông tin training dùng để đối chiếu:

* observation order
* action scale
* update rule
* deterministic inference
* no VecNormalize
* target_z behavior
* reward hiện tại

### 3. robot_drl runtime flow

Sơ đồ:

```text
input → observation → model.predict → action scale → waypoint/path → post-process → execute
```

Ghi file/function tương ứng.

### 4. Test/training vs ROS runtime comparison

Bảng:

```text
Item | Training/Test | robot_drl runtime | Match? | Risk/Note
```

Tối thiểu gồm:

* model path
* deterministic mode
* VecNormalize
* observation_dim
* observation order
* unit
* frame
* target_z
* action_scale
* Z sign
* policy_path
* processed_path
* executed_path
* runtime validator

### 5. Z issue analysis

Kết luận theo logic:

```text
Nếu raw_action_z test và ROS khác nhau:
    nghi observation/normalization/target/deterministic/model.

Nếu raw_action_z giống nhưng next_z khác:
    nghi action scale/dấu Z/frame/update rule.

Nếu policy path không tụt nhưng processed/executed path tụt:
    nghi interpolation/smoothing/path conversion.

Nếu mọi thứ khớp nhưng vẫn tụt:
    lúc đó mới xem reward/training.
```

### 6. Sharp turn/path smoothness audit

* Có tính được max turn angle không.
* Có tính được max step distance không.
* Có path log không.
* Nếu chưa có, cần log ở đâu.

### 7. Runtime safety audit

* Có validator chưa.
* Thiếu check nào.
* Rủi ro nếu execute path không validate.

### 8. Findings

Liệt kê theo mức độ:

* High risk
* Medium risk
* Low risk

### 9. Recommendations for next step

Chỉ đề xuất, không sửa:

* Log thêm gì.
* Chỗ nào cần đối chiếu raw_action/next_pos.
* Có nên thêm validator không.
* Có cần chỉnh reward/training không.
* Có cần train/fine-tune lại không.

### 10. Commands run

* Lệnh đã chạy.
* Kết quả pass/fail.
* Lỗi nếu có.

Ràng buộc cuối:

* Không chỉnh sửa source trong task này.
* File duy nhất được phép tạo là `robot_drl_runtime_z_audit_report.md`.
* Nếu cần ghi chú thêm, ghi trong file báo cáo.
* Không train lại model.
* Không ghi đè checkpoint/model.
* Không thay đổi config.
