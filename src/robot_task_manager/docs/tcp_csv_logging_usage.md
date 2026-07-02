# TCP CSV Logging Usage Guide

## 1. Mục tiêu logging

Hệ thống logging này thu dữ liệu TCP set-pose (mục tiêu) vs actual-pose (TF thực tế đo qua
`base_frame → tcp_frame`) theo thời gian, cho từng lần gọi action, để phục vụ:

- đánh giá mô hình robot / bộ điều khiển (độ chính xác bám quỹ đạo TCP)
- đánh giá `/move_to_pose` (PTP)
- đánh giá `/move_to_pose_cartesian` (Cartesian)
- đánh giá `/repeatability_test` (lặp lại vị trí)
- đánh giá `/move_checker_board` (quét lưới hiệu chuẩn)
- đánh giá `/pickplace` (chuỗi pick-place hoàn chỉnh)

Mỗi lần gọi action tạo ra **một file CSV riêng** chứa toàn bộ set/actual pose, sai số, và các
mốc sự kiện (event/sample/summary) của đúng lần gọi đó — không trộn lẫn với các lần gọi khác.

## 2. Kiến trúc logging hiện tại

Có 2 logger "per-call" (mỗi lần gọi action → 1 file CSV) là trọng tâm của tài liệu này:

### PickPlaceTcpLogger
- Định nghĩa trong `robot_task_manager/src/pickplace_server.cpp`, dùng riêng cho `/pickplace`.
- Mỗi lần gọi `/pickplace` tạo 1 file CSV riêng (`pickplace_NNNN_*.csv`) — **không cần** đặt
  `goal.enable_tcp_log` vì `PickPlace.action` không có field này.
- Chỉ phụ thuộc tham số node-level `enable_executor_logging` (xem mục 3).
- Schema riêng 33 cột, có thêm các cột pick/place pose và `velocity_scale` (xem mục 6.2).
- Không dùng chung với `PerCallTcpLogger` — cố ý tách riêng vì ra đời trước, có schema cố định
  khác, và đã được kiểm chứng độc lập.

### PerCallTcpLogger
- Định nghĩa trong `robot_task_manager/include/robot_task_manager/per_call_tcp_logger.hpp`,
  dùng chung (generic) cho 4 action:
  - `/move_to_pose`
  - `/move_to_pose_cartesian`
  - `/repeatability_test`
  - `/move_checker_board`
- Chỉ tạo file CSV riêng khi **cả hai** điều kiện đều đúng:
  1. Node được launch với tham số `enable_executor_logging:=true`.
  2. Goal của lần gọi đó có `enable_tcp_log: true`.
- Nếu action được gọi làm action con (ví dụ `/repeatability_test` tự gọi `/move_to_pose_cartesian`
  bên trong), caller luôn để `enable_tcp_log` ở giá trị mặc định `false` nên **không** sinh thêm
  file log con.
- Schema chung 27 cột, có cột `metadata_json` để mỗi action nhét dữ liệu goal riêng của mình
  (xem mục 6.1).

Ngoài 2 logger trên, package còn có `ExecutorExperimentLogger`
(`robot_task_executor/include/robot_task_executor/executor_experiment_logger.hpp`) — đây **không
phải** logger per-call TCP. Nó là logger CSV dùng chung, một instance gắn với cả vòng đời của một
node/process, ghi hai loại dữ liệu:
1. Dữ liệu thực thi cấp thấp (`actual_sample`, `joint_command`, `ref_waypoint`,
   `execute_start`, `execute_summary`) — dùng bởi `task_executor_node`
   (`/move_cartesian_pose_sequence`), bởi `MoveItExecutor` (nền tảng của `gohome`,
   `move_to_pose`, `move_to_pose_cartesian`), và độc lập bởi `robot_drl_executor_node` (demo
   Gazebo/real-hardware, launch stack khác, không liên quan tới `mock.launch.py`).
2. Dữ liệu vòng đời action (`action_goal_received`, `action_goal_rejected`,
   `action_stage_failed`, `action_result`, ghi qua `log_lifecycle_event()`).

`ExecutorExperimentLogger` vẫn đang được dùng, không bị xóa; nhưng nó **không** phải công cụ để
đánh giá TCP set-vs-actual per-call — dùng cho mục đích đó thì dùng `PickPlaceTcpLogger` hoặc
`PerCallTcpLogger` như mô tả ở trên.

## 3. Vì sao `enable_tcp_log` mặc định `false`

- Tránh action con (ví dụ `/move_to_pose_cartesian` được `/repeatability_test` gọi nội bộ) sinh
  thêm file CSV phụ không mong muốn mỗi lần lặp.
- Tránh log của `/pickplace` (vốn tự log riêng, luôn bật) bị vẽ chồng thêm dữ liệu từ các action
  con `/move_to_pose`/`/move_to_pose_cartesian` mà nó gọi bên trong.
- Khi cần đánh giá riêng một action cụ thể, gọi trực tiếp action đó (không qua action cha) và đặt
  `enable_tcp_log: true` trên goal.

Tương tự, `enable_executor_logging` (tham số node, không phải field goal) cũng mặc định `false`
ở cấp launch — nếu không bật tham số này thì dù goal có `enable_tcp_log: true`, logger vẫn không
được khởi tạo và sẽ không có file CSV nào được ghi.

## 4. Thư mục output

```text
Report/executor_logs/
└── run_YYYYMMDD_HHMMSS_PID[_NNN]/
    ├── index.csv
    ├── move_to_pose_0001_YYYYMMDD_HHMMSS.csv
    ├── move_to_pose_cartesian_0001_YYYYMMDD_HHMMSS.csv
    ├── repeatability_test_0001_YYYYMMDD_HHMMSS.csv
    ├── checker_board_0001_YYYYMMDD_HHMMSS.csv
    └── pickplace_0001_YYYYMMDD_HHMMSS.csv
```

- Mỗi action server (`move_to_pose_server`, `move_to_pose_cartesian_server`,
  `repeatability_test_server`, `checker_board_server`, `pickplace_server`) là một **process
  riêng**, nên mỗi node tự tạo `run_<timestamp>_<pid>[_NNN]/` riêng của nó ngay khi khởi động (nếu
  `enable_executor_logging=true`) — không dùng chung một run folder giữa các node.
- `[_NNN]` (001, 002, ...) chỉ xuất hiện nếu bị trùng timestamp+PID trong cùng thư mục
  `executor_log_dir`, để tránh ghi đè.
- `index.csv` nằm trong từng run folder, liệt kê tất cả các lần gọi action mà node đó xử lý trong
  suốt vòng đời của nó (một dòng mỗi lần gọi).
- Tên file CSV của từng lần gọi: `<prefix>_<call_index 4 chữ số>_<timestamp>.csv`, với `prefix`
  là `move_to_pose`, `move_to_pose_cartesian`, `repeatability_test`, `checker_board`, hoặc
  `pickplace` tuỳ action.

## 5. Cách bật logging cho từng action

Trước tiên, node phải được launch với `enable_executor_logging:=true`, ví dụ:

```bash
source install/setup.bash
ros2 launch robot_bringup mock.launch.py enable_executor_logging:=true
```

### 5.1. PickPlace

- `/pickplace` tự log riêng qua `PickPlaceTcpLogger`, **không cần** field `enable_tcp_log` (action
  `PickPlace.action` không có field này).
- Chỉ cần `enable_executor_logging:=true` lúc launch, mỗi lần gọi `/pickplace` sẽ tự tạo
  `pickplace_*.csv`.

```bash
ros2 action send_goal /pickplace robot_task_manager_msgs/action/PickPlace \
  "{pose_pick: {position: {x: 0.4, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, \
    pose_place: {position: {x: 0.4, y: 0.2, z: 0.05}, orientation: {w: 1.0}}, \
    velocity_scale: 0.2, execute: true}"
```

### 5.2. MoveToPose

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose \
  "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {w: 1.0}}, \
    velocity_scale: 0.2, execute: true, enable_tcp_log: true}"
```

File sinh ra: `move_to_pose_NNNN_*.csv` trong run folder của `move_to_pose_server`.

### 5.3. MoveToPoseCartesian

```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian \
  "{target_pose: {position: {x: 0.4, y: 0.1, z: 0.25}, orientation: {w: 1.0}}, \
    velocity_scale: 0.2, execute: true, enable_tcp_log: true}"
```

File sinh ra: `move_to_pose_cartesian_NNNN_*.csv`.

### 5.4. RepeatabilityTest

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.4, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}, \
    disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: 0.05, z: 0.3}, orientation: {w: 1.0}}}, \
    axis: 1, meas_offset: 0.02, repeat_count: 3, velocity_scale: 0.2, execute: true, \
    enable_tcp_log: true}"
```

- 1 lần gọi `/repeatability_test` = 1 file CSV (`repeatability_test_NNNN_*.csv`), dù bên trong nó
  gọi Cartesian/PTP nhiều lần cho mỗi vòng lặp.
- Các action con (`/move_to_pose`, `/move_to_pose_cartesian`) mà `repeatability_test_server` dùng
  nội bộ để di chuyển **không** tạo file phụ, vì server này tự thực hiện motion qua
  `MoveItExecutor`/`PlannerUtils` trực tiếp chứ không gọi qua action client với goal riêng.
- Dùng cột `stage` dạng `loop_N_...` để phân tích từng lần lặp riêng biệt (xem mục 8).

### 5.5. CheckerBoard

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.05, velocity_scale: 0.2, execute: true, enable_tcp_log: true}"
```

Giới hạn hiện tại:
- Set-pose của mỗi target trong lưới checkerboard **chưa** được truyền chính xác vào logger —
  cột `stage` được gắn từ chuỗi feedback (`feedback_cb`) của `MoveItExecutor::checkerBoard()`,
  còn set-pose vẫn giữ nguyên giá trị được seed lúc bắt đầu call.
- Actual TCP pose và stage vẫn được log đầy đủ theo thời gian, chỉ có sai số set-vs-actual theo
  từng target là chưa chính xác.
- Muốn có sai số chính xác từng target thì cần sửa `MoveItExecutor::checkerBoard()` để expose
  target pose ra ngoài qua callback (hiện tại `feedback_cb` chỉ truyền `(stage, progress)`).

## 6. Schema CSV

### 6.1. Schema PerCallTcpLogger (27 cột)

```text
time_sec, stage, row_type,
set_x, set_y, set_z, set_qx, set_qy, set_qz, set_qw,
actual_x, actual_y, actual_z, actual_qx, actual_qy, actual_qz, actual_qw,
error_x, error_y, error_z, error_pos_norm, error_ori_rad,
status, success, message, call_index, metadata_json
```

Ý nghĩa từng nhóm:
- `time_sec`: thời gian tương đối (giây) kể từ lúc bắt đầu call.
- `stage`: nhãn giai đoạn hiện tại (xem mục 8).
- `row_type`: `event` / `sample` / `summary` (xem mục 7).
- `set_x..set_qw`: pose mục tiêu (vị trí + quaternion) tại thời điểm ghi dòng; rỗng nếu chưa có
  set-pose nào được set cho stage hiện tại.
- `actual_x..actual_qw`: pose TCP thực tế đọc từ TF `base_frame → tcp_frame`; rỗng nếu TF lookup
  thất bại (không crash, chỉ để trống + ghi cảnh báo vào `message`).
- `error_x, error_y, error_z, error_pos_norm`: sai số vị trí (m) và độ lớn vector sai số, chỉ có
  khi cả set và actual đều có giá trị.
- `error_ori_rad`: sai số góc quay (rad) giữa quaternion set và actual.
- `status`: nhãn trạng thái tự do (`received`, `stage_start`, `stage_end`, `stage_failed`,
  `completed`, `aborted`, ...).
- `success`: `true`/`false`, chỉ có ở dòng `summary`.
- `message`: thông điệp mô tả (kèm cảnh báo TF nếu có).
- `call_index`: số thứ tự lần gọi trong vòng đời node đó (khớp với `index.csv`).
- `metadata_json`: chuỗi JSON do action tự build lúc `startCall()`, chứa các field goal riêng
  của action đó (ví dụ `velocity_scale`, `axis`, `repeat_count`, ...) — giữ nguyên suốt cả call.

### 6.2. Schema PickPlaceTcpLogger (33 cột)

```text
time_sec, stage, row_type,
set_x, set_y, set_z, set_qx, set_qy, set_qz, set_qw,
actual_x, actual_y, actual_z, actual_qx, actual_qy, actual_qz, actual_qw,
error_x, error_y, error_z, error_pos_norm, error_ori_rad,
status, success, message,
call_index, velocity_scale, pick_x, pick_y, pick_z, place_x, place_y, place_z
```

22 cột đầu giống hệt `PerCallTcpLogger` (`time_sec` .. `message`). 8 cột cuối là phần mở rộng
riêng của PickPlace, thay thế cho `metadata_json` chung:
- `call_index`: số thứ tự lần gọi `/pickplace`.
- `velocity_scale`: velocity scale của goal.
- `pick_x/y/z`, `place_x/y/z`: toạ độ pick pose và place pose của goal.

PickPlace dùng schema cột cố định riêng (thay vì `metadata_json` chung như `PerCallTcpLogger`)
vì logger này ra đời trước `PerCallTcpLogger`, đã được kiểm chứng độc lập, và cột cố định giúp
đọc/phân tích bằng pandas dễ hơn cho một action có cấu trúc goal cố định (luôn có pick/place
pose) thay vì JSON tự do.

## 7. Row type

- `event`: một mốc sự kiện rời rạc — ví dụ `stage_start` (bắt đầu một giai đoạn), `stage_end`
  (hoàn thành giai đoạn), `stage_failed` (giai đoạn lỗi), `received` (goal vừa được chấp nhận).
- `sample`: dòng lấy mẫu TCP theo thời gian trong lúc thực thi (background thread, tần số theo
  `executor_sample_rate_hz`), dùng để vẽ quỹ đạo/đường sai số theo thời gian.
- `summary`: dòng cuối cùng của call, ghi kết quả tổng (`success`, `message`) khi action kết
  thúc (thành công hoặc lỗi).

## 8. Stage naming

### PickPlace
```text
pickplace_start
open_gripper
move_to_pre_pick
approach_pick
close_gripper
move_to_pre_place
approach_place
open_gripper_release
pickplace_end
```

### MoveToPose
```text
move_to_pose_start
planning
execution
move_to_pose_end
```

### MoveToPoseCartesian
```text
cartesian_start
cartesian_planning
cartesian_execution
cartesian_end
```

### RepeatabilityTest
```text
repeatability_start
move_to_retract
loop_N_move_to_meas_1
loop_N_wait_meas_1
loop_N_move_to_retract_1
loop_N_move_to_disturb
loop_N_move_to_retract_2
loop_N_move_to_meas_2
loop_N_wait_meas_2
loop_N_move_to_retract_3
...
repeatability_end
```
(`N` là chỉ số vòng lặp, bắt đầu từ 1, lặp lại theo `goal.repeat_count`.)

### CheckerBoard
Stage được gắn trực tiếp từ chuỗi `stage` trong feedback của
`MoveItExecutor::checkerBoard()` (feedback callback), cộng thêm `checker_board_start` lúc nhận
goal và `checker_board_end` lúc kết thúc. Nội dung `stage` cụ thể phụ thuộc cách
`MoveItExecutor::checkerBoard()` đặt tên từng bước quét lưới hiện tại (xem
`robot_task_manager/src/moveit_executor.cpp`).

## 9. Cách đọc bằng pandas

```python
import pandas as pd
from pathlib import Path

run_dir = Path("Report/executor_logs/run_xxx")

index_df = pd.read_csv(run_dir / "index.csv")
print(index_df)

for path in sorted(run_dir.glob("*.csv")):
    if path.name == "index.csv":
        continue
    df = pd.read_csv(path)
    print(path.name, df.shape)
    print(df["row_type"].value_counts(dropna=False))
    print(df["stage"].value_counts(dropna=False))
```

## 10. Chỉ số đánh giá đề xuất

Tính trên các dòng `row_type == "sample"` có đủ `error_pos_norm`/`error_ori_rad` (set và actual
đều có giá trị) của một file CSV (một lần gọi action):

- `max_error_pos_norm`
- `mean_error_pos_norm`
- `rmse_error_pos_norm`
- `final_error_pos_norm` (giá trị `error_pos_norm` ở dòng cuối cùng, hoặc dòng `summary`)
- `max_error_ori_rad`
- `mean_error_ori_rad`
- `rmse_error_ori_rad`
- `duration_sec` (`time_sec` lớn nhất trong file, hoặc lấy từ `index.csv`)
- `sample_count` (số dòng có `row_type == "sample"`)

Công thức:
```text
rmse = sqrt(mean(error^2))
```

## 11. Lưu ý khi dùng

- Nếu gọi action trực tiếp để đánh giá thì đặt `enable_tcp_log: true` trên goal.
- Nếu gọi action như action con của action khác (ví dụ bên trong `/repeatability_test`), để
  `enable_tcp_log` ở giá trị mặc định `false`.
- Không bật `enable_tcp_log` cho các action con bên trong `/pickplace` nếu không muốn sinh file
  phụ — bản thân `/pickplace` không có field `enable_tcp_log` nên vấn đề này chỉ áp dụng nếu bạn
  tự thêm logic gọi các action khác từ ngoài.
- Cần cả `enable_executor_logging:=true` (tham số node/launch) **và** `enable_tcp_log: true`
  (field goal) thì mới có file CSV — chỉ bật một trong hai sẽ không sinh file.
- `/move_checker_board` hiện chưa có set-pose chính xác từng target trong lưới quét (xem mục
  5.5).
- Nếu TF (`base_frame → tcp_frame`) chưa sẵn sàng tại thời điểm ghi dòng, logger không crash —
  chỉ để trống các cột `actual_*`/`error_*` và ghi cảnh báo vào `message`.
- Nếu action fail giữa chừng, vẫn luôn có dòng `summary` cuối cùng với `success=false` và
  `message` mô tả lỗi.

## 12. Troubleshooting

**Không thấy file CSV nào được tạo:**
- Kiểm tra `enable_executor_logging` đã được set `true` lúc launch chưa:
  ```bash
  ros2 param get /move_to_pose_action_server enable_executor_logging
  ```
- Kiểm tra goal gửi đi có `enable_tcp_log: true` chưa (chỉ áp dụng cho 4 action dùng
  `PerCallTcpLogger`; `/pickplace` không cần field này).
- Kiểm tra đúng run folder — mỗi node có run folder riêng, tìm bằng:
  ```bash
  find Report/executor_logs -name "*.csv" -newer /tmp -printf "%T@ %p\n" | sort -n
  ```

**Tìm nhầm run folder:**
- Mỗi lần restart node sẽ tạo run folder mới (timestamp + PID khác). Dùng `index.csv` trong từng
  run folder, hoặc sort theo thời gian sửa đổi, để xác định đúng lần chạy.

**Node chưa được launch lại sau build / interface action đã đổi nhưng chưa `source`:**
```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager --symlink-install
source install/setup.bash
```
Nếu không source lại sau khi build interface (`.action`) thay đổi, client cũ có thể không có
field `enable_tcp_log` khi gọi action, hoặc server sẽ báo lỗi type mismatch.

**pandas đọc lỗi do mở nhầm file đang ghi:**
- File CSV chỉ hoàn chỉnh (đủ dòng `summary`, đã `flush()`+`close()`) sau khi action kết thúc.
  Nếu đọc trong lúc action đang chạy, `pd.read_csv` có thể đọc phải dòng dở. Đợi action kết thúc
  rồi mới đọc, hoặc bọc trong `try/except`.

**Kiểm tra nhanh interface hiện tại:**
```bash
ros2 action list
ros2 interface show robot_task_manager/action/MoveToPose
ros2 param get /move_to_pose_action_server enable_executor_logging
find Report/executor_logs -name "*.csv"
```
