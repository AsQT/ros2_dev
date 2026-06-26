Yêu cầu cập nhật pkg `robot_gui` để các nút trong các tab `TaskControlPanel` gọi đúng các ROS2 action tương ứng.

## 1. Bối cảnh

Layout `robot_gui.ui` đã có khu vực `TaskControlPanel` gồm:

* `cbModeControl`: chọn mode/task.
* `taskModeTabs`: chứa các tab chức năng.
* Các tab dự kiến:

```text
0. Move Pose
1. Move Pose RL
2. Gripper
3. Pick Place
4. Pick Place Vision
5. Pick Place RL
6. Check Board
7. Repeatability Test
```

Tài liệu hướng dẫn gọi action nằm trong file:

```text
Call_action.md
```

Cần đọc kỹ file này trước khi code.

Hiện tại đã làm phần chuyển tab bằng `cbModeControl`. Bước này cần cập nhật chức năng các nút trong từng tab để gọi action.

## 2. Mục tiêu chính

Cập nhật `robot_gui` để:

* Nút `Start` gọi action tương ứng với `execute=true`.
* Nút `Plan` gọi action tương ứng với `execute=false`.
* Nút `Stop` hiện tại chỉ cần log/cancel nếu đã có cancel action client; nếu chưa có cancel thì log rõ “cancel chưa implement”.
* Khi gọi action, hiển thị feedback/result/status vào ô thông tin phía dưới màn hình.
* Chỉ tập trung gọi action từ GUI, không sửa server action.
* Phải test được trên `mock_hardware`.

## 3. Khuyến nghị tổ chức code

Có thể viết riêng file mới để dễ chỉnh sửa, ví dụ:

```text
robot_gui/include/robot_gui/task_action_controller.hpp
robot_gui/src/task_action_controller.cpp
```

Class đề xuất:

```cpp
class TaskActionController : public QObject
{
    Q_OBJECT
public:
    TaskActionController(rclcpp::Node::SharedPtr node, Ui::MainWindow *ui, QObject *parent = nullptr);

    void connectUiSignals();

private:
    void sendMovePose(bool execute);
    void sendMovePoseCartesian(bool execute);
    void sendGripper(bool execute);
    void sendPickPlace(bool execute);
    void sendDrlPickPlace(bool execute);
    void sendCheckerBoard(bool execute);
    void sendRepeatabilityTest(bool execute);

    void appendActionLog(const QString &msg);
};
```

Nếu project hiện tại không dùng `Ui::MainWindow` trực tiếp thì điều chỉnh theo class GUI hiện có, nhưng vẫn nên tách logic action ra file riêng.

## 4. Action cần hỗ trợ

Theo `Call_action.md`, cần dùng các action sau:

```text
/move_to_pose
robot_task_manager/action/MoveToPose

/move_to_pose_cartesian
robot_task_manager/action/MoveToPoseCartesian

/move_gripper
robot_task_manager/action/MoveGripper

/pickplace
robot_task_manager/action/PickPlace

/drl_pickplace
robot_task_manager/action/DrlPickPlace

/move_checker_board
robot_task_manager/action/CheckerBoard

/repeatability_test
robot_task_manager/action/RepeatabilityTest
```

Nếu tab `Move Pose RL` hiện chưa có action riêng trong backend thì chưa tự chế action mới. Chỉ log rõ:

```text
Move Pose RL action chưa có mapping backend, chưa gửi goal.
```

Không sửa `robot_task_manager` ở bước này.

## 5. Quy tắc Plan / Start

Tất cả action đã có field `execute`.

Quy tắc bắt buộc:

```text
Nút Plan  -> execute = false
Nút Start -> execute = true
```

Không được dùng nút `Plan` để chạy thật robot.

Khi `execute=false`, action server vẫn phải planning/validate và đường plan phải hiển thị trên RViz nếu backend MoveIt đang publish trajectory như hiện tại.

## 6. Tab Move Pose

Tab `Move Pose` có ô tick chọn Cartesian.

Yêu cầu sửa đúng logic:

* Nếu checkbox **không tick**:

  * `Plan` gọi `/move_to_pose` với `execute=false`.
  * `Start` gọi `/move_to_pose` với `execute=true`.

* Nếu checkbox **có tick**:

  * `Plan` gọi `/move_to_pose_cartesian` với `execute=false`.
  * `Start` gọi `/move_to_pose_cartesian` với `execute=true`.

Checkbox nên có text rõ:

```text
Move Pose Cartesian
```

Goal:

```yaml
target_pose:
  position:
    x: <x nhập từ GUI>
    y: <y nhập từ GUI>
    z: <z nhập từ GUI>
  orientation:
    x: <quaternion.x>
    y: <quaternion.y>
    z: <quaternion.z>
    w: <quaternion.w>
velocity_scale: <velocity nhập từ GUI hoặc default>
execute: true/false
```

Nếu chưa có ô velocity trong tab, dùng hằng số default:

```cpp
static constexpr double DEFAULT_VELOCITY_SCALE = 0.5;
```

## 7. Xử lý Orientation

Các ô nhập hướng trong GUI là dạng Euler:

```text
Roll
Pitch
Yaw
```

Quy tắc:

* Nếu người dùng có nhập Roll/Pitch/Yaw:

  * Parse sang `double`.
  * Quy ước đơn vị phải rõ ràng.
  * Nếu UI đang ghi `deg`, chuyển degree sang radian trước.
  * Nếu UI không ghi đơn vị, mặc định dùng degree và ghi rõ trong report.
  * Convert RPY sang quaternion trước khi gửi action.

* Nếu người dùng không nhập orientation:

  * Dùng orientation mặc định dạng hằng số dễ chỉnh:

```cpp
static constexpr double DEFAULT_ORI_X = 1.0;
static constexpr double DEFAULT_ORI_Y = 1.0;
static constexpr double DEFAULT_ORI_Z = 0.0;
static constexpr double DEFAULT_ORI_W = 0.0;
```

Lưu ý: giữ đúng default trên theo yêu cầu hiện tại, không tự đổi sang `{0,0,0,1}`.

Nên gom logic thành hàm dùng chung:

```cpp
geometry_msgs::msg::Quaternion makeQuaternionFromUi(
    QLineEdit *rollEdit,
    QLineEdit *pitchEdit,
    QLineEdit *yawEdit);
```

hoặc hàm tương đương.

## 8. Tab Gripper

Tab `Gripper` gọi action:

```text
/move_gripper
robot_task_manager/action/MoveGripper
```

Goal:

```yaml
position: <width/position nhập từ GUI>
execute: true/false
```

Nếu chỉ có nút Open/Close:

* Open dùng default:

```cpp
static constexpr double DEFAULT_GRIPPER_OPEN = 0.048;
```

* Close dùng default hoặc ô nhập:

```cpp
static constexpr double DEFAULT_GRIPPER_CLOSE = 0.028;
```

Nếu tab có nút Plan thì `execute=false`. Nếu không có Plan thì chỉ cần Start/Open/Close execute=true theo UI hiện có.

## 9. Tab Pick Place

Tab `Pick Place` gọi:

```text
/pickplace
robot_task_manager/action/PickPlace
```

Goal:

```yaml
pose_pick:
  position: {x, y, z}
  orientation: <quaternion>
pose_place:
  position: {x, y, z}
  orientation: <quaternion>
gripper: <gripper width>
velocity_scale: <velocity>
execute: true/false
```

Quy tắc:

* `Plan` -> `execute=false`.
* `Start` -> `execute=true`.
* Nếu orientation pick/place không nhập thì dùng default quaternion hằng số ở mục 7.
* Nếu chỉ có một cụm orientation dùng chung thì dùng chung cho cả pick và place.
* Nếu có 2 cụm orientation riêng thì parse riêng.

Default nếu ô trống:

```cpp
static constexpr double DEFAULT_PICK_X = 0.40;
static constexpr double DEFAULT_PICK_Y = 0.10;
static constexpr double DEFAULT_PICK_Z = 0.25;

static constexpr double DEFAULT_PLACE_X = 0.30;
static constexpr double DEFAULT_PLACE_Y = 0.00;
static constexpr double DEFAULT_PLACE_Z = 0.25;

static constexpr double DEFAULT_PICKPLACE_GRIPPER = 0.01;
static constexpr double DEFAULT_PICKPLACE_VELOCITY = 0.5;
```

## 10. Tab Pick Place Vision

Tab `Pick Place Vision` hiện có khung ảnh/preview.

Ở bước này:

* Không cần implement xử lý vision.
* Nếu nút Start/Plan cần gọi action mà chưa có mapping rõ thì có thể gọi `/pickplace` bằng tọa độ đang nhập tay.
* Nếu tọa độ lấy từ vision chưa có topic/data thì log rõ:

```text
Vision pose chưa có dữ liệu, dùng pose nhập tay hoặc không gửi goal.
```

Không tự thêm pipeline YOLO/vision mới.

## 11. Tab Pick Place RL

Tab `Pick Place RL` gọi:

```text
/drl_pickplace
robot_task_manager/action/DrlPickPlace
```

Goal:

```yaml
target_pick:
  header:
    frame_id: <frame_id>
  pose:
    position: {x, y, z}
    orientation: <quaternion>
target_place:
  header:
    frame_id: <frame_id>
  pose:
    position: {x, y, z}
    orientation: <quaternion>
gripper_close_width_m: <width>
execute: true/false
```

Default:

```cpp
static const QString DEFAULT_DRL_FRAME_ID = "base_link";
static constexpr double DEFAULT_DRL_GRIPPER_CLOSE = 0.028;
```

Quy tắc:

* `Plan` -> `execute=false`.
* `Start` -> `execute=true`.

Ngoài ra, cập nhật layout:

* Sao chép y nguyên khung ảnh/preview từ tab `PickPlaceVision` sang tab `PickPlace_RL`.
* Hai tab cùng thể hiện chung nội dung ảnh/preview.
* Không làm mất objectName hiện có ở tab `PickPlaceVision`.
* Với tab `PickPlace_RL`, objectName mới nên có suffix `_RL` để tránh duplicate objectName.

Ví dụ:

```text
yoloPreviewWidget      -> yoloPreviewWidget_RL
rawImageFrame          -> rawImageFrame_RL
detectionImageFrame    -> detectionImageFrame_RL
```

Nếu code hiện tại đang publish ảnh vào widget cũ, chưa cần nối ảnh cho tab RL ở bước này; chỉ cần layout có khung ảnh giống nhau và không làm hỏng tab Vision.

## 12. Tab Check Board

Tab `Check Board` gọi:

```text
/move_checker_board
robot_task_manager/action/CheckerBoard
```

Goal:

```yaml
step: <step nhập từ GUI>
velocity_scale: <velocity>
execute: true/false
```

Yêu cầu layout:

* Thêm nút `Plan` cho tab `Check Board`.
* `Plan` gọi `/move_checker_board` với `execute=false`.
* `Start` gọi `/move_checker_board` với `execute=true`.

Default:

```cpp
static constexpr double DEFAULT_CHECKER_STEP = 0.10;
static constexpr double DEFAULT_CHECKER_VELOCITY = 0.5;
```

## 13. Tab Repeatability Test

Tab `Repeatability Test` gọi:

```text
/repeatability_test
robot_task_manager/action/RepeatabilityTest
```

Goal:

```yaml
retract_pose:
  header:
    frame_id: <frame_id>
  pose:
    position: {x, y, z}
    orientation: <quaternion>

disturb_pose_1:
  header:
    frame_id: <frame_id>
  pose:
    position: {x, y, z}
    orientation: <quaternion>

disturb_pose_2:
  header:
    frame_id: <frame_id>
  pose:
    position: {x, y, z}
    orientation: <quaternion>

axis: 0/1/2
meas_offset: <offset>
repeat_count: <N>
velocity_scale: <velocity chậm lúc đo>
execute: true/false
```

Axis mapping:

```text
X -> 0
Y -> 1
Z -> 2
```

Default:

```cpp
static const QString DEFAULT_REPEAT_FRAME_ID = "world";
static constexpr double DEFAULT_REPEAT_RETRACT_X = 0.40;
static constexpr double DEFAULT_REPEAT_RETRACT_Y = 0.00;
static constexpr double DEFAULT_REPEAT_RETRACT_Z = 0.18;

static constexpr double DEFAULT_REPEAT_DISTURB1_X = 0.35;
static constexpr double DEFAULT_REPEAT_DISTURB1_Y = -0.08;
static constexpr double DEFAULT_REPEAT_DISTURB1_Z = 0.18;

static constexpr double DEFAULT_REPEAT_DISTURB2_X = 0.45;
static constexpr double DEFAULT_REPEAT_DISTURB2_Y = 0.08;
static constexpr double DEFAULT_REPEAT_DISTURB2_Z = 0.18;

static constexpr double DEFAULT_REPEAT_MEAS_OFFSET = 0.02;
static constexpr int DEFAULT_REPEAT_COUNT = 3;
static constexpr double DEFAULT_REPEAT_VELOCITY = 0.25;
```

Quy tắc:

* `Plan` -> `execute=false`.
* `Start` -> `execute=true`.

## 14. Ô thông tin action phía dưới màn hình

Khu vực ô dưới cùng, nằm bên dưới vùng RViz và các tab, hiện chưa dùng.

Yêu cầu:

* Dùng khu vực này để hiển thị trạng thái action.
* Nếu đã có widget dạng `QTextEdit`, `QPlainTextEdit`, `QLabel`, `QFrame` thì tận dụng.
* Nếu chưa có widget text bên trong, thêm `QPlainTextEdit` hoặc `QTextEdit` vào đúng khu vực đó.
* Không làm thay đổi layout các phần khác.

ObjectName đề xuất:

```text
txtActionLog
```

Nội dung cần hiển thị:

* Khi bấm Plan/Start:

```text
[Move Pose] Sending goal to /move_to_pose, execute=false
```

* Khi action server available/unavailable.
* Feedback action.
* Result success/fail.
* Message result.
* Lỗi parse input.
* Lỗi timeout.
* Lỗi cancel nếu có.

Log nên append, không ghi đè toàn bộ.

## 15. Action client implementation

Dùng `rclcpp_action`.

Cần đảm bảo GUI không bị block.

Không được dùng kiểu chờ blocking dài trong thread UI.

Nếu cần chờ action server, dùng timeout ngắn và log lỗi:

```cpp
if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    appendActionLog("Action server not available: /move_to_pose");
    return;
}
```

Feedback callback phải append log an toàn với Qt thread. Nếu callback chạy ngoài UI thread, dùng:

```cpp
QMetaObject::invokeMethod(...)
```

hoặc signal/slot queued connection.

## 16. Validate input

Tất cả ô nhập số phải parse an toàn:

* Nếu rỗng thì dùng default.
* Nếu nhập sai kiểu số thì log lỗi và không gửi goal.
* `velocity_scale` phải nằm trong `(0, 1]`.
* `gripper` không âm.
* `repeat_count > 0`.
* `axis` chỉ là X/Y/Z.
* `meas_offset` không được bằng 0.

Không để GUI crash vì input sai.

## 17. Không được làm

Không sửa action server.

Không sửa `robot_task_manager` nếu không bắt buộc.

Không sửa `mock_hardware`.

Không đổi tên action.

Không đổi type action.

Không đổi objectName cũ nếu code đang dùng.

Không phá chức năng chuyển tab `cbModeControl -> taskModeTabs`.

Không tự thêm thuật toán vision hoặc RL mới.

Không làm blocking GUI khi action đang chạy.

## 18. Build và test bắt buộc trên mock_hardware

Phải build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_gui robot_task_manager --symlink-install
source install/setup.bash
```

Chạy mock stack/action server theo project hiện có, ví dụ:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Nếu cần mock hardware/bringup thì dùng launch mock hiện có của repo.

Kiểm tra action server:

```bash
ros2 action list
```

Phải thấy tối thiểu:

```text
/move_to_pose
/move_to_pose_cartesian
/move_gripper
/pickplace
/move_checker_board
/repeatability_test
```

Nếu test `/drl_pickplace` cần chạy đủ DRL planner/mock stack tương ứng. Nếu chưa đủ dependency thì báo cáo rõ điều kiện còn thiếu, không được giả vờ pass.

## 19. Checklist test thủ công

### Move Pose

* Nhập X/Y/Z.
* Không tick Cartesian.
* Bấm Plan -> gửi `/move_to_pose`, `execute=false`.
* Bấm Start -> gửi `/move_to_pose`, `execute=true`.
* Tick Cartesian.
* Bấm Plan -> gửi `/move_to_pose_cartesian`, `execute=false`.
* Bấm Start -> gửi `/move_to_pose_cartesian`, `execute=true`.

### Gripper

* Nhập width.
* Bấm Start/Open/Close.
* Xác nhận gọi `/move_gripper`.

### Pick Place

* Nhập pick/place pose.
* Bấm Plan -> `/pickplace`, `execute=false`.
* Bấm Start -> `/pickplace`, `execute=true`.

### Pick Place RL

* Xác nhận có khung ảnh giống Pick Place Vision.
* Bấm Plan -> `/drl_pickplace`, `execute=false`, nếu server đủ.
* Bấm Start -> `/drl_pickplace`, `execute=true`, nếu server đủ.

### Check Board

* Xác nhận đã có nút Plan.
* Bấm Plan -> `/move_checker_board`, `execute=false`.
* Bấm Start -> `/move_checker_board`, `execute=true`.

### Repeatability Test

* Chọn X/Y/Z.
* Nhập retract/disturb poses, offset, repeat count, velocity.
* Bấm Plan -> `/repeatability_test`, `execute=false`.
* Bấm Start -> `/repeatability_test`, `execute=true`.

### Action log

* Mọi lần bấm nút đều có log ở ô dưới cùng.
* Feedback/result hiện rõ.
* Input sai phải hiện lỗi, không crash.

## 20. Báo cáo sau khi hoàn thành

Tạo file:

```text
robot_gui/task_action_gui_report.md
```

Nội dung gồm:

* File đã sửa/thêm.
* Mapping từng nút sang action.
* Quy tắc Plan/Start execute false/true.
* Cách xử lý orientation Euler -> quaternion.
* Hằng số orientation mặc định đang dùng.
* Widget log action đang dùng.
* Kết quả build.
* Kết quả test trên mock_hardware.
* Action nào chưa test được và lý do cụ thể nếu có.

Chỉ xem là hoàn thành khi build thành công và các nút Plan/Start trong các tab gọi đúng action tương ứng trên mock_hardware.
