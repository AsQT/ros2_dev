Bạn đang làm việc trong workspace ROS 2 của robot. Hiện tại DRL/RL path planning trên `mock_hw` đã hoạt động ổn định. Hãy trực tiếp kiểm tra source code hiện tại, triển khai, build, chạy thử, sửa lỗi và tiếp tục thực hiện cho đến khi action pick-and-place dựa trên RL chạy thành công thực tế.

Không chỉ phân tích hoặc viết báo cáo. Phải sửa source code, build workspace, chạy launch, gửi action goal, đọc log, chẩn đoán lỗi và lặp lại cho đến khi hoàn thành.

# 1. Mục tiêu

Tạo một ROS 2 action mới để thực hiện chu trình pick-and-place, trong đó:

* Các chuyển động dài sử dụng DRL/RL planner hiện có.
* Các chuyển động thẳng đứng ngắn tại vị trí pick sử dụng action Cartesian hiện có.
* Điều khiển gripper sử dụng action `MoveGripper` trong package `robot_task_manager`.
* Có thể tham khảo cách tổ chức, gọi sub-action, feedback, result và xử lý lỗi của action `PickPlace` hiện có trong `robot_task_manager`.

Ưu tiên đặt action mới trong `robot_task_manager` nếu phù hợp với kiến trúc hiện tại. Không tạo package mới nếu không cần thiết.

# 2. Kiểm tra kiến trúc hiện có trước khi sửa

Trước khi triển khai, hãy tự đọc và xác định chính xác:

* Action `PickPlace` hiện tại.
* Action `MoveGripper`.
* Action `MoveToPoseCartasian` hoặc tên thực tế tương ứng trong source code.
* Cách gọi DRL planner hiện tại.
* Cách DRL planner trả về và thực thi trajectory.
* Frame tọa độ đang sử dụng, ví dụ `base_link`.
* Cách lấy pose hiện tại của end-effector.
* Tên end-effector link.
* Các launch file khởi động `mock_hw`, controller, MoveIt, task manager và DRL planner.

Lưu ý: tên `MoveToPoseCartasian` có thể đang bị viết khác trong source code. Phải sử dụng đúng action type và action name thực tế đang tồn tại. Không tự ý đổi tên interface cũ làm hỏng các node khác.

Không được tạo planner giả, không bypass DRL bằng MoveIt hoặc nội suy thẳng cho các bước được yêu cầu sử dụng RL.

# 3. Action interface mới

Tạo action mới với tên phù hợp quy ước hiện tại, ưu tiên:

```text
DrlPickPlace.action
```

Goal tối thiểu phải có:

```text
geometry_msgs/PoseStamped target_pick
geometry_msgs/PoseStamped target_place
float64 gripper_close_width_m
```

Trong đó:

* `target_pick` là pose của end-effector tại vị trí kẹp vật.
* `target_place` là pose của end-effector tại vị trí nhả vật.
* `gripper_close_width_m` là độ mở của gripper sau khi đóng, đơn vị mét.
* Giá trị mặc định mong muốn là `0.028 m`, tương đương `28 mm`.
* Độ mở hoàn toàn của gripper là `0.05 m`, tương đương `5 cm`.

ROS action definition không hỗ trợ khai báo giá trị mặc định trực tiếp. Vì vậy:

* Test client/launch phải mặc định gửi `0.028`.
* Action server phải kiểm tra giá trị đầu vào.
* Nếu `gripper_close_width_m` không hữu hạn, nhỏ hơn hoặc bằng 0, hoặc không hợp lệ thì sử dụng `0.028 m`.
* Giới hạn hợp lệ phải nằm trong giới hạn thực của gripper, tối đa không vượt quá `0.05 m`.

Result tối thiểu:

```text
bool success
string message
string failed_stage
```

Feedback tối thiểu:

```text
string current_stage
float32 progress
geometry_msgs/PoseStamped current_pose
```

Có thể điều chỉnh message definition để phù hợp với convention hiện có, nhưng phải giữ đủ thông tin tương đương.

Sau khi thêm action phải cập nhật đầy đủ:

* `CMakeLists.txt`
* `package.xml`
* ROS interface generation
* Dependency giữa các target
* Install Python script hoặc executable nếu server/client được viết bằng Python

# 4. Chuỗi thực thi bắt buộc

Action server phải thực hiện đúng thứ tự sau.

## Bước 1: Kiểm tra hệ thống

* Kiểm tra goal hợp lệ.
* Transform `target_pick` và `target_place` về planning frame nếu cần.
* Kiểm tra toàn bộ sub-action server đã sẵn sàng.
* Lấy pose hiện tại thật của end-effector từ TF, robot state hoặc cơ chế đang được project sử dụng.
* Không giả định robot đang ở một pose cố định.

## Bước 2: Mở gripper

Gọi action `MoveGripper` trong `robot_task_manager`:

```text
gripper width = 0.05 m
```

Chỉ chuyển sang bước tiếp theo khi action trả về thành công.

## Bước 3: DRL plan đến pre-pick

Tạo pose trung gian:

```text
pre_pick.x = target_pick.x
pre_pick.y = target_pick.y
pre_pick.z = target_pick.z + 0.05
```

Orientation của `pre_pick` phải giống `target_pick`.

Từ pose hiện tại, gọi DRL/RL planner hiện có để lập kế hoạch và thực thi chuyển động đến `pre_pick`.

Phải chờ đến khi trajectory được thực thi thành công và kiểm tra pose cuối nằm trong tolerance cho phép.

Không dùng Cartesian planner hoặc MoveIt planner để thay thế DRL ở bước này.

## Bước 4: Hạ xuống vị trí pick

Gọi action `MoveToPoseCartasian` trong `robot_task_manager` để dịch end-effector từ `pre_pick` xuống đúng `target_pick`.

Chuyển động phải chủ yếu theo trục Z và giữ nguyên:

* X
* Y
* Orientation

Chỉ tiếp tục khi action Cartesian trả về thành công.

## Bước 5: Đóng gripper

Gọi action `MoveGripper` với:

```text
width = gripper_close_width_m
```

Mặc định:

```text
0.028 m
```

Không nhầm đơn vị giữa mét và milimét.

## Bước 6: Nâng vật lên

Tạo pose nâng:

```text
lift_pose.x = target_pick.x
lift_pose.y = target_pick.y
lift_pose.z = target_pick.z + 0.05
```

Orientation giữ giống `target_pick`.

Gọi action `MoveToPoseCartasian` để nâng end-effector thẳng lên `0.05 m`.

## Bước 7: DRL plan đến target_place

Từ `lift_pose`, gọi DRL planner để lập kế hoạch và thực thi đến chính xác `target_place`.

`target_place` được định nghĩa là pose nhả vật của end-effector, không phải tọa độ tâm vật hoặc mặt bàn. Không tự cộng hoặc trừ offset Z ngoài yêu cầu này.

Không thay DRL planner bằng planner khác.

## Bước 8: Nhả vật

Gọi `MoveGripper`:

```text
width = 0.05 m
```

Khi gripper mở thành công, action trả về:

```text
success = true
current_stage = DONE
```

# 5. State machine thực thi

Tổ chức action server theo state machine rõ ràng, ví dụ:

```text
VALIDATE_GOAL
WAIT_FOR_SERVERS
OPEN_GRIPPER
PLAN_TO_PRE_PICK
DESCEND_TO_PICK
CLOSE_GRIPPER
LIFT_FROM_PICK
PLAN_TO_PLACE
OPEN_GRIPPER_AT_PLACE
DONE
FAILED
CANCELLED
```

Mỗi stage phải:

* Publish feedback.
* Có timeout riêng hoặc timeout cấu hình được.
* Kiểm tra kết quả sub-action.
* Ghi log rõ stage bắt đầu, thành công hoặc thất bại.
* Khi lỗi phải trả về chính xác `failed_stage`.
* Không tiếp tục chuỗi nếu bước trước thất bại.

Khi client cancel action:

* Cancel sub-action đang chạy.
* Dừng trajectory hoặc chuyển robot về trạng thái an toàn theo cơ chế hiện có.
* Không để action con tiếp tục chạy ngầm.
* Trả về trạng thái cancelled đúng chuẩn ROS 2.

Không sử dụng `time.sleep()` dài làm block executor. Dùng callback, future hoặc cơ chế async phù hợp kiến trúc hiện tại.

# 6. Kiểm tra pose và điều kiện thành công

Không coi action thành công chỉ vì sub-action trả `success`.

Sau các bước chuyển động, kiểm tra pose end-effector thực tế:

* Sai số vị trí phải nằm trong tolerance cấu hình được.
* Sai số orientation phải nằm trong tolerance cấu hình được.
* Log pose yêu cầu, pose thực tế và sai số.

Tạo các parameter, có thể dùng giá trị mặc định:

```text
position_tolerance_m: 0.01
orientation_tolerance_rad: 0.10
sub_action_timeout_sec: 60.0
drl_timeout_sec: 120.0
gripper_open_width_m: 0.05
gripper_default_close_width_m: 0.028
pick_approach_height_m: 0.05
```

Phải sử dụng tolerance phù hợp với mock hardware và planner hiện tại, không đặt tolerance lớn đến mức che giấu lỗi.

# 7. Launch test tự động

Tạo launch test hoặc launch file kiểm thử riêng, tên gợi ý:

```text
drl_pick_place_random_test.launch.py
```

Launch file phải khởi động đầy đủ các thành phần cần thiết dựa trên launch mock hardware hiện có:

* Robot description
* `mock_hw`
* Controller manager
* Joint state broadcaster
* Arm controller
* MoveIt hoặc task manager nếu action Cartesian phụ thuộc vào chúng
* DRL planner
* Action server mới
* Random obstacle spawner
* Random test client

Không copy lại toàn bộ launch hiện có nếu có thể include launch file cũ.

# 8. Random test client

Viết test client gọi action nhiều lần liên tiếp.

Parameter mặc định:

```text
number_of_trials: 20
random_seed: 0
gripper_close_width_m: 0.028
```

Mỗi trial phải random:

1. Vị trí start của robot.
2. Vị trí vật cản.
3. Kích thước vật cản.
4. Orientation vật cản nếu mock environment hỗ trợ.
5. `target_pick`.
6. `target_place`.

Các giá trị random phải thỏa mãn:

* Nằm trong workspace robot có thể với tới.
* Nằm trong joint limit.
* Pose start không collision.
* Target pick không nằm bên trong vật cản.
* Target place không nằm bên trong vật cản.
* Vật cản không spawn xuyên robot ở pose start.
* Có khoảng cách tối thiểu hợp lý giữa pick và place.
* Có tình huống vật cản nằm gần hoặc chắn đường thẳng để thực sự kiểm tra khả năng tránh vật cản của DRL.
* Không tạo toàn bộ test case quá dễ hoặc không có tác dụng kiểm tra obstacle avoidance.

Phải log đầy đủ cho mỗi trial:

```text
trial_id
random_seed
start joint position hoặc start pose
obstacle pose
obstacle dimensions
target_pick
target_place
gripper_close_width
action result
failed_stage
elapsed time
final pose error
```

Seed phải được lưu để có thể chạy lại chính xác trial bị lỗi.

# 9. Tiêu chí pass/fail của test

Một trial chỉ được PASS khi:

* Action server nhận goal thành công.
* Mở gripper thành công.
* DRL đến pre-pick thành công.
* Cartesian xuống target_pick thành công.
* Đóng gripper thành công.
* Cartesian nâng lên thành công.
* DRL đến target_place thành công.
* Mở gripper tại target_place thành công.
* Pose cuối nằm trong tolerance.
* Không có collision được báo bởi môi trường hoặc planning scene.
* Không có node quan trọng bị crash.
* Action result trả `success = true`.

Sau 20 trial, in bảng tổng kết:

```text
Total trials
Passed
Failed
Success rate
Average execution time
Maximum pose error
Failed seeds
Failed stages
```

Yêu cầu cuối cùng:

```text
20/20 trial phải PASS.
```

Nếu có bất kỳ trial nào lỗi:

* Không được kết luận hoàn thành.
* Dùng seed của trial lỗi để tái hiện.
* Đọc log và sửa nguyên nhân gốc.
* Build lại.
* Chạy lại trial lỗi.
* Sau khi sửa phải chạy lại toàn bộ 20 trial từ đầu.
* Tiếp tục đến khi đạt 20/20.

Không được sửa test theo hướng loại bỏ các case khó chỉ để đạt PASS.

# 10. Build và chạy thực tế

Sau khi sửa source:

1. Build đúng package và dependency:

```bash
colcon build --symlink-install --packages-up-to robot_task_manager
```

Nếu action server nằm ở package khác, thêm đúng package đó và các package DRL liên quan.

2. Source workspace:

```bash
source install/setup.bash
```

3. Kiểm tra action interface:

```bash
ros2 interface show <package_name>/action/DrlPickPlace
```

4. Kiểm tra action list:

```bash
ros2 action list -t
```

5. Chạy random test launch.

6. Theo dõi:

```bash
ros2 node list
ros2 action list -t
ros2 topic list
ros2 control list_controllers
```

7. Kiểm tra controller ở trạng thái `active`.

8. Gửi ít nhất một goal thủ công bằng `ros2 action send_goal` để xác nhận interface hoạt động độc lập với test client.

Không được chỉ build thành công rồi dừng. Phải chạy action thật.

# 11. Không làm hỏng chức năng hiện có

* Không thay đổi logic DRL đang chạy ổn nếu không thật sự cần.
* Không thay đổi interface của `MoveGripper`, `MoveToPoseCartasian` hoặc `PickPlace` hiện có.
* Không đổi tên topic/action/service đang được package khác sử dụng.
* Không xóa code cũ.
* Không hard-code pose chỉ để test pass.
* Không dùng obstacle data giả khác với obstacle đã spawn trong môi trường.
* Không bỏ qua lỗi controller hoặc TF.
* Không trả success khi trajectory chưa thực thi xong.
* Không thực hiện thay đổi ngoài phạm vi task nếu không cần thiết.

# 12. Báo cáo kết quả cuối cùng

Khi đã đạt 20/20 trial, cung cấp báo cáo ngắn nhưng có bằng chứng:

* Danh sách file đã tạo.
* Danh sách file đã sửa.
* Tên action mới.
* Action goal/result/feedback definition.
* Chuỗi sub-action thực tế.
* Command build.
* Command launch test.
* Command gửi goal thủ công.
* Kết quả 20 trial.
* Các lỗi đã gặp và nguyên nhân gốc.
* Các thay đổi quan trọng đã thực hiện.
* `git diff --stat`.
* Các parameter có thể cấu hình.

Không chỉ nói “đã hoàn thành”; phải đưa log hoặc output chứng minh.

# 13. Hẹn giờ tắt máy

Sau khi:

* Source code đã được lưu.
* Build và test đã kết thúc.
* Báo cáo và log đã được ghi ra file.
* Không còn tiến trình build/test quan trọng đang chạy.

Hãy hẹn giờ tắt máy vào đúng:

```text
03:00 sáng ngày 18/06/2026
Timezone: Asia/Ho_Chi_Minh
```

Ưu tiên tạo systemd transient timer có thời điểm tuyệt đối:

```bash
sudo systemd-run \
  --unit=poweroff-at-20260618-0300 \
  --on-calendar='2026-06-18 03:00:00 Asia/Ho_Chi_Minh' \
  /usr/bin/systemctl poweroff
```

Sau đó xác minh timer đã tồn tại:

```bash
systemctl list-timers --all | grep poweroff-at-20260618-0300
```

Có thể kiểm tra thêm:

```bash
systemctl status poweroff-at-20260618-0300.timer
```

Nếu môi trường không hỗ trợ systemd transient timer, dùng:

```bash
sudo shutdown -h 03:00
```

và xác minh:

```bash
shutdown --show
```

Không được báo đã hẹn giờ thành công nếu command bị lỗi hoặc thiếu quyền `sudo`.

Nếu không có quyền sudo không tương tác:

* Ghi rõ lỗi thực tế.
* In chính xác command cần người dùng chạy.
* Không giả lập kết quả thành công.
