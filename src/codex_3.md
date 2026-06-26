Yêu cầu kiểm tra và cập nhật launch backend RL trong `robot_task_manager`.

Hiện tượng lỗi:

```text
[DrlPickPlace] WAIT_FOR_SERVERS
DrlPickPlace failed at WAIT_FOR_SERVERS: Planner parameter service not available: /drl_unified_planner_node/set_parameters
```

Nguyên nhân:
Các action RL như `/drl_pickplace` hoặc `/move_pose_rl` cần các service/node từ `robot_drl`, nhưng `robot_task_manager/launch/task_servers.launch.py` hiện chỉ launch action server, chưa launch đủ DRL backend.

Mục tiêu:
Kiểm tra toàn bộ action RL đang có, xác định chính xác các service/topic/node cần thiết, sau đó cập nhật `task_servers.launch.py` và `task_servers_sim.launch.py` để launch luôn backend RL cần thiết.

Không được đoán tên executable. Phải đọc code thực tế trong `robot_drl` và `robot_task_manager`.

============================================================

1. KIỂM TRA CÁC ACTION RL HIỆN CÓ
   ============================================================

Kiểm tra trong `robot_task_manager`:

* `drl_pickplace_server`
* `move_pose_rl_server`
* các file action:

  * `DrlPickPlace.action`
  * `MovePoseRl.action`
* các client hoặc GUI backend gọi RL nếu có.

Cần xác định action RL nào đang phụ thuộc DRL backend:

```text
/drl_pickplace
/move_pose_rl
```

Nếu còn action RL khác thì cũng phải đưa vào báo cáo.

============================================================
2. XÁC ĐỊNH SERVICE/TOPIC MÀ ACTION RL CẦN
==========================================

Từ code thực tế, liệt kê đầy đủ các service/topic/action mà RL action server đang wait hoặc gọi.

Tối thiểu cần kiểm tra các dependency sau:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
/drl/forward_trajectory_poses
```

Phân loại rõ:

* Service bắt buộc cho plan-only `execute=false`.
* Service bắt buộc cho execute thật `execute=true`.
* Topic chỉ phục vụ visualization/feedback.
* Parameter service của planner node.

Ví dụ mong muốn:

```text
execute=false cần:
- /drl_unified_planner_node/set_parameters
- /drl/plan
- /drl/clear_trajectory

execute=true cần thêm:
- /drl/execute_forward
- /drl/get_execution_status

Visualization/monitor:
- /drl/forward_trajectory_poses
```

Nhưng không được ghi theo ví dụ nếu code thực tế khác.

============================================================
3. TÌM NODE/EXECUTABLE TẠO RA CÁC SERVICE NÀY
=============================================

Trong package `robot_drl`, kiểm tra:

* `setup.py`
* `package.xml`
* thư mục `robot_drl/`
* các launch file hiện có
* các node Python/C++ tạo service:

  * `/drl/plan`
  * `/drl/clear_trajectory`
  * `/drl/execute_forward`
  * `/drl/get_execution_status`
  * `/drl_unified_planner_node/set_parameters`

Phải xác định chính xác:

* package name
* executable name
* node name
* parameters cần truyền
* có cần `use_sim_time` không
* có cần model path không
* có cần config path không
* có cần activate venv không, nếu project đang dùng Python environment riêng

Không được hard-code sai executable.

============================================================
4. CẬP NHẬT task_servers.launch.py
==================================

File cần sửa:

```text
robot_task_manager/launch/task_servers.launch.py
```

Yêu cầu:
Launch luôn backend RL cần thiết để khi chạy:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

thì các service RL phải có sẵn.

Sau launch, các lệnh sau phải thấy service:

```bash
ros2 service list | grep drl
ros2 service list | grep drl_unified_planner_node
```

Phải thấy tối thiểu các service mà action RL yêu cầu.

Nếu backend RL nặng hoặc có thể muốn tắt, thêm launch argument:

```python
DeclareLaunchArgument(
    "enable_drl_backend",
    default_value="true",
    description="Launch DRL planner backend required by RL actions"
)
```

Nhưng mặc định phải là `true`, vì yêu cầu hiện tại là task_servers launch lên phải dùng được RL action ngay.

Nếu cần model path/config path, thêm launch arguments rõ ràng:

```text
drl_model_path
drl_config_path
planner_node_name
```

và default phải lấy từ package path, không dùng absolute path `/home/...` nếu có thể tránh.

============================================================
5. CẬP NHẬT task_servers_sim.launch.py
======================================

File cần sửa:

```text
robot_task_manager/launch/task_servers_sim.launch.py
```

Yêu cầu:

* Cũng launch backend RL.
* Truyền `use_sim_time:=true` cho các node RL backend nếu node hỗ trợ.
* Không phá các server đang chạy trong sim.

Sau khi chạy:

```bash
ros2 launch robot_task_manager task_servers_sim.launch.py
```

phải có đủ service RL như bản real/mock.

============================================================
6. TRÁNH LAUNCH TRÙNG NODE
==========================

Nếu hệ thống đã có launch riêng của `robot_drl`, cần kiểm tra có bị launch trùng node `/drl_unified_planner_node` không.

Yêu cầu:

* Nếu include launch file từ `robot_drl`, đảm bảo không duplicate node name.
* Nếu tạo node trực tiếp trong `task_servers.launch.py`, đảm bảo action server đang trỏ đúng `planner_node_name`.
* Nếu user vẫn tự chạy launch `robot_drl` riêng, phải ghi rõ trong README là không nên chạy trùng hoặc thêm option `enable_drl_backend:=false`.

Ví dụ:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

chỉ dùng khi DRL backend đã được launch ở nơi khác.

============================================================
7. CẬP NHẬT PARAMETER planner_node_name
=======================================

Trong các server:

* `drl_pickplace_server`
* `move_pose_rl_server`

Kiểm tra parameter:

```yaml
planner_node_name: /drl_unified_planner_node
```

Yêu cầu:

* Tên này phải khớp đúng node được launch.
* Nếu node launch không có dấu `/` hoặc namespace khác, sửa cho thống nhất.
* Không được để action server wait service sai tên.

Service parameter cuối cùng phải đúng:

```text
/drl_unified_planner_node/set_parameters
```

hoặc tên thực tế tương ứng nếu code đổi, nhưng phải thống nhất toàn bộ.

============================================================
8. CẬP NHẬT GUI BACKEND NẾU CẦN
===============================

Nếu GUI đang kiểm tra service backend RL, cập nhật danh sách service theo kết quả audit thực tế.

GUI phải báo rõ:

* thiếu `/move_pose_rl` action server
* thiếu `/drl_pickplace` action server
* thiếu `/drl_unified_planner_node/set_parameters`
* thiếu `/drl/plan`
* thiếu `/drl/execute_forward`

Nhưng nếu `task_servers.launch.py` đã launch đủ backend, GUI không nên báo thiếu.

============================================================
9. TEST BẮT BUỘC
================

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager robot_drl robot_gui
source install/setup.bash
```

Launch task servers:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Kiểm tra action:

```bash
ros2 action list | grep drl_pickplace
ros2 action list | grep move_pose_rl
```

Kiểm tra service:

```bash
ros2 service list | grep drl
ros2 service list | grep drl_unified_planner_node
```

Phải thấy đầy đủ service mà action RL yêu cầu.

Test `/move_pose_rl execute=false`:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: false}" \
  --feedback
```

Kỳ vọng:

* Không còn lỗi thiếu `/drl_unified_planner_node/set_parameters`.
* Action đi qua stage WAIT_FOR_SERVERS thành công.
* Plan-only không làm robot di chuyển.

Test `/drl_pickplace execute=false` nếu action có field execute:

```bash
ros2 action send_goal /drl_pickplace robot_task_manager/action/DrlPickPlace \
  "{target_pick: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.40, y: 0.05, z: 0.08}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, target_place: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.34, y: -0.10, z: 0.08}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, gripper_close_width_m: 0.028, execute: false}" \
  --feedback
```

Nếu `DrlPickPlace.action` chưa có `execute`, dùng interface thực tế sau khi kiểm tra bằng:

```bash
ros2 interface show robot_task_manager/action/DrlPickPlace
```

Test sim launch:

```bash
ros2 launch robot_task_manager task_servers_sim.launch.py
```

Kiểm tra service RL tương tự.

============================================================
10. CẬP NHẬT Call_action.md / README
====================================

Cập nhật tài liệu:

```text
robot_task_manager/Call_action.md
robot_task_manager/README.md
```

Nội dung cần bổ sung:

* `task_servers.launch.py` hiện đã launch luôn DRL backend.
* Các service mà RL action cần.
* Cách kiểm tra backend:

```bash
ros2 service list | grep drl
ros2 service list | grep drl_unified_planner_node
```

* Nếu muốn tự launch DRL backend riêng, dùng:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

nếu đã thêm argument này.

============================================================
11. BÁO CÁO
===========

Tạo file:

```text
drl_backend_launch_audit_report.md
```

Nội dung bắt buộc:

1. Danh sách action RL đã kiểm tra.
2. Service/topic từng action RL cần.
3. Node/executable nào tạo ra từng service.
4. File launch đã sửa.
5. Launch argument mới nếu có.
6. Cách tránh launch trùng DRL backend.
7. Kết quả build.
8. Kết quả `ros2 action list`.
9. Kết quả `ros2 service list`.
10. Kết quả test `/move_pose_rl`.
11. Kết quả test `/drl_pickplace`.
12. Xác nhận lỗi thiếu `/drl_unified_planner_node/set_parameters` đã hết.
13. Xác nhận không ảnh hưởng các action non-RL:

    * `/move_to_pose`
    * `/move_to_pose_cartesian`
    * `/move_checker_board`
    * `/pickplace`
    * `/repeatability_test`
    * `/gohome`

Tiêu chí hoàn thành:

* Chạy `ros2 launch robot_task_manager task_servers.launch.py` là đủ backend cho action RL.
* Không còn lỗi:

```text
Planner parameter service not available: /drl_unified_planner_node/set_parameters
```

* `/move_pose_rl` gọi được.
* `/drl_pickplace` gọi được.
* Có option tắt DRL backend nếu cần tránh launch trùng.
