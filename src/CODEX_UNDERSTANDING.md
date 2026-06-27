# Mình hiểu dự án này như thế nào (để bạn đối chiếu)

> File này là **cách hiểu của Codex** về dự án, viết theo kiểu diễn giải để bạn đọc và
> chỉ ra chỗ nào mình hiểu sai. Tài liệu tham chiếu chi tiết (bảng package, interface)
> nằm ở [CODEX_PROJECT_CONTEXT.md](CODEX_PROJECT_CONTEXT.md). File này tập trung vào
> **bức tranh tổng thể + những điểm mình chưa chắc**.
>
> Ngày: 2026-06-27. Mọi giá trị cụ thể bên dưới đều đã được mình mở file thật để kiểm tra.

---

## 1. Mình hiểu đây là dự án gì

Đây là **workspace ROS 2 Jazzy** điều khiển một **cánh tay robot 6 bậc tự do + gripper 2 ngón**,
chạy được ở 3 chế độ:

1. **Sim (Gazebo)** — mô phỏng đầy đủ, dùng `gz_ros2_control`.
2. **Mock** — chỉ MoveIt + mock hardware, để test logic/giao diện, không có vật lý.
3. **Real** — cánh tay thật, giao tiếp qua TCP/RS485 (`robot_hardware_interface`).

Điểm đặc biệt của dự án so với một robot arm thông thường: có thêm một **bộ lập kế hoạch
quỹ đạo bằng Deep Reinforcement Learning (DRL)** để né vật cản khi gắp–thả, song song với
MoveIt truyền thống. Nói cách khác, robot có **2 "bộ não" chuyển động**:
- **MoveIt** (lập quỹ đạo cổ điển): dùng cho các tác vụ cơ bản (về home, di chuyển tới pose,
  Cartesian, gắp–thả thường).
- **DRL** (chính sách học sẵn): dùng cho `drl_pickplace` và `move_pose_rl` — sinh ra chuỗi
  waypoint Cartesian né vật cản, rồi để MoveIt thực thi đoạn Cartesian đó.

Ngoài ra có **vision (YOLO + ArUco)** để nhận diện vật thể, và một **GUI Qt** để người dùng
bấm nút điều khiển.

---

## 2. Kiến trúc — mình hình dung các tầng

Mình hiểu hệ thống chia thành các tầng rõ ràng, dữ liệu chảy 1 chiều từ trên xuống:

1. **Tầng mô tả robot** — `robot_description`
   Là "nguồn sự thật" duy nhất về hình học robot (URDF/Xacro, frame, mesh).
   File `ros2_control.xacro` là chỗ **chọn loại hardware** theo cờ:
   `use_sim` → plugin Gazebo, `use_mock` → mock, còn lại → hardware thật.
   → Mình hiểu: muốn đổi sim/mock/real thì bản chất là đổi cờ truyền vào xacro này.

2. **Tầng điều khiển** — `robot_control`
   Cấu hình `ros2_control` với 3 controller: `arm_controller` (joint_1..6),
   `gripper_controller` (joint_gl, joint_gr), `joint_state_broadcaster`. Tần số 10 Hz.

3. **Tầng lập kế hoạch** — `robot_moveit`
   `move_group` + RViz + SRDF/IK. Đây là nơi mọi chuyển động cuối cùng được lập & thực thi.

4. **Tầng môi trường** — `robot_gazebo` (sim) **hoặc** `robot_hardware_interface` (real)
   Hai cái này thay thế nhau tùy chế độ. Gazebo phát `/clock`, camera sim, và topic
   "ground truth" `/sim/pick_wood_info`, `/sim/obstacle_box_info`.

5. **Tầng tác vụ (quan trọng nhất với người dùng)** — `robot_task_manager`
   Cung cấp **9 action server**. Mỗi action đều có cờ `execute`. Đây là tầng mà GUI và
   các script demo gọi tới.

6. **Tầng DRL** — `robot_drl` (Python, lập kế hoạch) + `robot_drl_executor` (C++, thực thi
   Cartesian). `robot_drl` nạp model Stable-Baselines3 và sinh waypoint;
   `robot_drl_executor` biến waypoint thành đường Cartesian cho MoveIt.

7. **Tầng phụ trợ** — `robot_gui` (giao diện), `robot_vision_pipeline` (thị giác),
   `robot_bringup` (các launch tổng).

---

## 3. Luồng chạy mình hình dung (demo DRL pick-place)

Đây là kịch bản mình tin là "demo chính" của dự án
(`rl_pick_place_box_gazebo_demo.launch.py`):

1. Khởi động Gazebo + robot + bridge (qua `sim.launch.py`, nhưng tắt wood demo và tắt
   DRL backend mặc định để demo tự bật bản riêng).
2. Sau vài giây: MoveIt + spawn controller theo chuỗi (jsb → arm → gripper).
3. Task servers chạy.
4. `robot_drl_executor` + `drl_unified_planner_node` (bản DRL của demo) chạy.
5. `spawn_pick_wood_obstacle_box.py` spawn **khối gỗ cần gắp** + **hộp vật cản**, rồi phát
   pose của chúng (ground truth) lên `/sim/pick_wood_info` và `/sim/obstacle_box_info`
   trong frame `base_link`.
6. `drl_pick_place_wood_box_demo_client.py` nghe 2 topic đó, cộng thêm `pick_z_offset_m`
   (0.06) và `object_z_correction_m` (0.01) vào Z, rồi gửi goal tới action `drl_pickplace`.
7. `drl_pickplace_server` điều phối chuỗi sau (đây là **pipeline pick-place DRL** bạn đã
   xác nhận):
   1. Vị trí hiện tại → **mở gripper**.
   2. Dùng **DRL** di chuyển tới **vị trí gắp** (tọa độ vật thể — sau này lấy từ vision,
      hiện dùng ground truth), nhưng **Z được nâng lên ~5 cm** (qua `pick_z_offset_m`,
      mặc định 0.06 m, + `object_z_correction_m` 0.01 m).
   3. Từ điểm phía trên đó, **MoveCartesian đi thẳng xuống** đúng tọa độ gắp.
   4. **Kẹp gripper**.
   5. Dùng **DRL** di chuyển tới **tọa độ place**.
   6. **Mở gripper** (thả vật).
   (Cơ chế kỹ thuật: gọi `/drl/clear_trajectory` → `/drl/plan` → `/drl/execute_forward`
   khi `execute=true`; DRL lo phần né vật cản.)

→ Điểm mình hiểu đúng: **demo này dùng dữ liệu spawn ground truth, KHÔNG dùng camera/YOLO.**
**Mục tiêu cốt lõi của demo: thể hiện gắp–đặt THÍCH ỨNG khi vật cản đổi kích thước/vị trí.**
Bước tiếp theo của dự án là thêm **depth + camera trong sim** để tự detect tọa độ/kích thước
vật cản và khối gỗ (thay cho ground truth).

---

## 4. Những chỗ "dễ sai" mà mình đã ghi nhận

- **Cờ `execute`**: `execute=false` = chỉ PLAN, không chạy thật. Mình đã kiểm tra trong
  `drl_pickplace_server.cpp` có nhánh trả về "execution skipped". → Mình sẽ không bao giờ
  báo "thành công (đã chạy)" khi thực ra chỉ plan.
- **Z/frame**: pose vật thể nằm trong frame `base_link`; Z thật phụ thuộc offset + correction.
  Đụng vào mấy tham số này là sai độ cao gắp.
- **Controller chưa active ngay**: spawn theo chuỗi, timeout 15–20s. Phải chờ/kiểm tra
  `ros2 control list_controllers` mới ra lệnh.
- **File legacy dễ nhầm**: gazebo launch cũ trong `robot_description`, `pickplace_server_v1.cpp`,
  `moveit_mock.launch.py`, các `README_old.md`, và mọi file `*_report/*_audit/*_fix.md`
  (chỉ là lịch sử, không phải nguồn sự thật).
- **`/move_cartesian_pose_sequence` có 2 nơi cung cấp** (`robot_task_executor` và
  `robot_drl_executor`) — không chạy đồng thời cả hai.

---

## 5. ✅ Đã xác nhận với bạn (cập nhật 2026-06-27)

Trước đây mục này là các câu hỏi; nay đã được bạn trả lời:

1. **`real.launch.py` & launch chính.**
   `real.launch.py` (qua `moveit_gui.launch.py`, `use_mock:=false`) **tự bật đầy đủ**:
   controllers + `robot_hardware_interface` (plugin TCP nạp trong tiến trình ros2_control
   khi không dùng mock) + **GUI có nhúng RViz**. Đây gần như là **launch chính để điều khiển
   robot + GUI** trong 1 lệnh. Mặc định là **dùng mock** (`use_mock` default = true).
   `mock.launch.py` = cùng launch đó nhưng mock, và **không** bật vision.
   Phần YOLO/xử lý ảnh (`robot_vision_pipeline`) sau này sẽ được gộp thành một launch
   hoàn chỉnh hơn.

2. **Demo chính = gắp–đặt THÍCH ỨNG (né vật cản) trong sim.** Pipeline đã ghi ở §3.
   Mục tiêu: thể hiện gắp–đặt và **thích ứng khi vật cản đổi kích thước/vị trí**. Bản demo
   này chưa có vision; bước kế tiếp là dùng **depth + cam trong sim** để detect tọa độ/kích
   thước vật cản và khối gỗ cần gắp.

3. **Mục tiêu hiện tại:**
   - Đã chạy một số **test sai số mô hình với robot thật** qua `real.launch.py` nhưng còn
     lỗi (liên quan cả **firmware điều khiển trên robot**).
   - **Ưu tiên: hoàn thiện các demo trên sim trước**, fix lỗi hardware sau.
   - Cần **lấy lại dữ liệu đánh giá** để đưa vào báo cáo.

4. **Model DRL:** `run2` — đúng, đang deploy. Thuật toán hiện tại = **SAC**.
   Chi tiết spec (workspace, start/target/obstacle, observation 15D, action 3D) đã được
   bổ sung đầy đủ ở **Phụ lục A của [CODEX_PROJECT_CONTEXT.md](CODEX_PROJECT_CONTEXT.md)**,
   khớp với `robot_drl/models/run2/config.yaml`. Điểm quan trọng cần nhớ:
   - **Workspace** (đơn vị m, base frame): `x[0.25, 0.50] y[-0.15, 0.15] z[0.02, 0.30]` —
     **mọi vị trí ngoài vùng này có thể chạy sai.**
   - Observation 15D = `[current_pos, target_pos, err, rel_obs(chuẩn hóa), obs_size(chuẩn hóa)]`.
   - Action 3D ∈ [-1,1] = delta Cartesian chuẩn hóa; `delta = action * 0.01 m`.
   - **Hướng (orientation) được giữ cố định** trong quá trình planning; DRL chỉ đổi vị trí.
   - Vật cản phải tôn trọng `safety_margin = 0.03 m`.

5. **TCP frame:** trong **sim chỉ dùng `robot.xacro`**. Hai bản `robot_tcp_xy.xacro` /
   `robot_tcp_z.xacro` là **đổi tool để đánh giá sai số**, chỉ dùng trong **mock_hw** (xem
   trước) và **robot thật**, **không dùng trong sim Gazebo**.

6. **DRL venv:** `source ~/venvs/ros_rl/bin/activate` (có stable-baselines3/torch).

7. **Vision pipeline:** đang test, **chưa dùng thật với robot**; demo hiện chạy bằng
   ground truth.

---

## 6. Mình tự đánh giá mức độ hiểu

| Mảng | Mức tự tin | Ghi chú |
|---|---|---|
| Cấu trúc package & build | Cao | Đã kiểm tra từng `CMakeLists.txt`/`setup.py` |
| Luồng launch (sim/mock/real/demo) | Cao | Đã đọc trực tiếp 4 file launch bringup |
| Action/service/topic names | Cao | Đã grep xác minh trong source |
| Logic DRL pick-place (`execute`, `/drl/*`) | Khá cao | Đã đọc `drl_pickplace_server.cpp` |
| Nội bộ thuật toán DRL (reward/obs 15D) | Trung bình | Đọc mô tả + `state_builder.py`, chưa chạy thử |
| Hardware thật (TCP protocol) | Trung bình | Biết service/param, chưa nắm protocol chi tiết |
| Vision pipeline | Trung bình–thấp | Mới đọc lướt, chưa chắc phần nào đang dùng thật |
| GUI (Qt `.ui` ↔ C++) | Trung bình | Biết kiến trúc, chưa map hết từng objectName |

---

**Tóm lại:** mình đã nắm kiến trúc tổng thể, vai trò từng package, luồng demo DRL pick-place,
và các điểm rủi ro. Các điểm trước đây mình chưa chắc (§5) **đã được bạn xác nhận**, nên
mình đã hiểu thống nhất với bạn về: launch chính (`real`/`mock` qua `moveit_gui`), demo
trọng tâm (gắp–đặt thích ứng né vật cản trong sim), và ưu tiên hiện tại (**hoàn thiện demo
sim trước → fix hardware/firmware sau → lấy lại dữ liệu cho báo cáo**).

→ **Sẵn sàng nhận task.** Khi bạn giao việc tiếp theo (vd: thêm depth+cam vào sim để thay
ground truth, hoặc tinh chỉnh pipeline gắp–đặt), mình sẽ bám đúng cách hiểu này.
