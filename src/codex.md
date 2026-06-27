Hiện tại demo `DrlPickPlace` trên Gazebo đã chạy được plan-only với log:

```text
DrlPickPlace demo succeeded: DrlPickPlace planning success; execution skipped
```

Như vậy mới chỉ chứng minh planner chạy được. Yêu cầu tiếp theo là **bắt buộc execute được trong Gazebo**, không dừng ở plan-only.

Ngoài ra cần sửa obstacle box: **box là vật cản**, phải random kích thước lớn hơn trong khoảng:

```text
5 cm -> 15 cm
0.05 m -> 0.15 m
```

Wood vẫn là vật cần gắp.

---

## 1. Mục tiêu bắt buộc

Demo đúng phải là:

```text
wood = pick object / vật cần gắp
box  = obstacle / vật cản
camera = không dùng
pose object/obstacle = lấy từ Gazebo ground truth hoặc spawn info
```

Flow cuối cùng phải chạy:

```text
spawn wood
spawn obstacle box random size 0.05 -> 0.15 m
lấy pose wood
lấy pose box
đưa obstacle box vào DRL planner / planning scene nếu flow có dùng
gửi goal /drl_pickplace với execute=true
robot thực hiện pick wood
robot place wood
gripper mở/đóng đúng flow
action trả result success
```

Không được chỉ dừng ở:

```text
execution skipped
```

---

## 2. Kiểm tra nguyên nhân execution bị skipped

Trước tiên tìm lý do vì sao log hiện tại là:

```text
DrlPickPlace planning success; execution skipped
```

Kiểm tra các khả năng:

```text
1. Goal gửi vào /drl_pickplace đang có execute=false
2. Launch/demo node có parameter execute:=false
3. Demo node cố tình chạy plan-only
4. drl_pickplace_server.cpp có nhánh if (!execute) return success
5. robot_drl_executor hoặc /move_cartesian_pose_sequence chưa được gọi
6. Controller/Gazebo chưa sẵn sàng nên code đang fallback sang plan-only
```

Trace trong các file:

```bash
cd ~/ros2_dev

grep -R "execution skipped" -n src/
grep -R "execute=false\|execute:=false\|execute.*false" -n src/robot_bringup src/robot_task_manager src/robot_drl src/robot_drl_executor
grep -R "DrlPickPlace" -n src/robot_bringup src/robot_task_manager src/robot_drl src/robot_drl_executor
grep -R "move_cartesian_pose_sequence" -n src/robot_task_manager src/robot_drl_executor src/robot_drl src/robot_bringup
```

Sau đó sửa demo để goal/action chạy với:

```text
execute = true
```

Nếu có launch argument thì demo Gazebo chính phải default execute thật, ví dụ:

```text
execute:=true
plan_only:=false
```

Tên argument cụ thể tùy code hiện tại, nhưng không được để mặc định plan-only trong demo execute.

---

## 3. Bắt buộc execute qua Gazebo thật

Khi execute, phải kiểm tra các service/action/backend sau có hoạt động:

```text
/drl_pickplace
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
/move_cartesian_pose_sequence
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

Cần xác minh robot trong Gazebo thật sự di chuyển bằng cách kiểm tra:

```bash
ros2 topic echo /joint_states
ros2 action list -t
ros2 service list -t
ros2 control list_controllers
```

Nếu `/drl_pickplace` success nhưng joint không đổi thì chưa được coi là execute thật.

---

## 4. Box obstacle random size 5–15 cm

Sửa spawn logic của obstacle box.

Yêu cầu:

```text
box = obstacle
box không phải vật gắp
box size random trong khoảng 0.05 -> 0.15 m
```

Cần random ít nhất kích thước theo một trong hai cách:

### Cách A: random đều cả 3 chiều

```text
box_size_x = random(0.05, 0.15)
box_size_y = random(0.05, 0.15)
box_size_z = random(0.05, 0.15)
```

### Cách B: random cạnh chính, giữ tỉ lệ

```text
box_scale = random(0.05, 0.15)
box_size = [box_scale, box_scale, box_scale]
```

Chọn cách phù hợp với model/SDF hiện tại, nhưng phải ghi rõ trong report.

Không random ra box quá nhỏ dưới 5 cm.

Không random ra box quá lớn trên 15 cm.

Không để box chồng lên:

```text
robot base
wood
target place pose
table edge
```

Nếu random pose và size có thể gây chồng lấn thì phải có kiểm tra khoảng cách tối thiểu.

---

## 5. Ground truth object/obstacle

Không dùng camera.

Với Gazebo, lấy dữ liệu từ:

```text
spawn info
gazebo_msgs/srv/GetEntityState
gazebo_msgs/msg/ModelStates
hoặc API hiện workspace đang dùng
```

Phải đảm bảo:

```text
wood_pose = pose của pick_wood
box_pose = pose của obstacle_box
box_size = size random đã spawn
```

Nếu pose trả về trong `world`, còn planner/action cần `base_link`, phải transform đúng hoặc chứng minh `world` trùng frame phù hợp trong demo.

Không hard-code pose sai frame.

---

## 6. Đưa obstacle box vào planner

Vì box là vật cản, cần kiểm tra obstacle có thật sự được DRL/planning flow sử dụng không.

Phải xác nhận một trong các trường hợp:

```text
A. obstacle pose + size được đưa vào DRL observation/planner
B. obstacle được đưa vào planning scene / collision object
C. cả A và B
```

Nếu hiện tại chỉ spawn box trong Gazebo nhưng planner không biết box tồn tại, thì chưa đạt yêu cầu obstacle.

Cần trace trong:

```bash
grep -R "obstacle" -n src/robot_drl src/robot_task_manager src/robot_bringup src/robot_drl_executor
grep -R "box_size\|size_x\|size_y\|size_z\|collision" -n src/robot_drl src/robot_task_manager src/robot_bringup
```

---

## 7. Không sửa sai phạm vi

Không sửa:

```text
reward
observation dimension
trained model
network architecture
action space
```

trừ khi bắt buộc để execution chạy, và nếu phải sửa thì phải báo cáo rõ lý do.

Không dùng camera/YOLO.

Không đổi wood thành obstacle.

Không dùng box làm pick object.

Không xóa package cũ.

---

## 8. Python venv cho node RL

Node Python load model RL, đặc biệt:

```text
robot_drl/drl_unified_planner_node
```

phải chạy bằng Python interpreter:

```text
/home/minhquang/venvs/ros_rl/bin/python3
```

Chỉ prefix node Python cần RL libs.

Không prefix C++ node:

```text
robot_task_manager action servers
robot_drl_executor_node
```

---

## 9. Build bắt buộc

Sau khi sửa:

```bash
cd ~/ros2_dev
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select \
  robot_description \
  robot_drl_executor \
  robot_drl \
  robot_task_manager \
  robot_bringup

source install/setup.bash
```

Nếu thiếu package liên quan thì build thêm.

Không được dừng ở lỗi build. Phải sửa tiếp đến khi build được.

---

## 10. Test Gazebo bắt buộc

Chạy demo Gazebo:

```bash
cd ~/ros2_dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py
```

Nếu cần tạo launch mới để rõ vai trò thì đặt tên:

```text
rl_pick_place_wood_obstacle_box_execute_gazebo_demo.launch.py
```

Nhưng launch cũ cũng phải được cập nhật README/report để tránh nhầm.

Sau khi launch, kiểm tra:

```bash
ros2 node list
ros2 action list -t
ros2 service list -t
ros2 control list_controllers
```

Phải gửi goal hoặc demo node tự gửi goal với:

```text
execute=true
```

Không chấp nhận kết quả cuối là:

```text
execution skipped
```

Kết quả tối thiểu phải chứng minh:

```text
1. /drl_pickplace goal accepted
2. planner success
3. /move_cartesian_pose_sequence được gọi
4. robot joint_states thay đổi trong Gazebo
5. gripper mở/đóng
6. action result success hoặc nếu fail thì fail_stage rõ ràng sau khi đã cố execute
```

---

## 11. Report bắt buộc

Tạo report:

```text
src/robot_bringup/rl_pick_place_wood_box_gazebo_execute_report.md
```

Report phải có:

```text
# RL Pick Place Wood + Random Obstacle Box Gazebo Execute Report

## 1. Mục tiêu
- execute thật, không plan-only
- wood là vật gắp
- box là vật cản
- box random size 5 -> 15 cm

## 2. Nguyên nhân cũ bị execution skipped

## 3. File đã sửa

## 4. File đã tạo mới nếu có

## 5. Logic spawn wood
- name
- pose
- size
- role

## 6. Logic spawn obstacle box
- name
- random size range
- random pose rule
- collision avoidance rule với wood/robot

## 7. Ground truth dùng cho demo
- wood pose lấy từ đâu
- box pose lấy từ đâu
- box size lấy từ đâu
- frame xử lý

## 8. Cách obstacle được đưa vào DRL/planning
- observation/planner/planning scene

## 9. Cách bật execute=true

## 10. Lệnh build đã chạy

## 11. Kết quả build

## 12. Lệnh launch Gazebo đã chạy

## 13. Kết quả node/action/service/controller list

## 14. Kết quả /drl_pickplace execute
- goal
- accepted/rejected
- result
- failed_stage nếu có
- log chính của drl_pickplace_server
- log chính của drl_unified_planner_node
- log chính của robot_drl_executor_node
- log joint_states/controller chứng minh robot có di chuyển

## 15. Trạng thái cuối cùng
- execute được/chưa execute được
- nếu chưa được, lỗi cuối cùng còn lại là gì
```

---

## 12. Tiêu chí hoàn thành

Chỉ được coi là hoàn thành khi đạt đủ:

```text
- Gazebo launch được
- Wood spawn đúng là vật cần gắp
- Box spawn đúng là vật cản
- Box có size random 0.05 -> 0.15 m
- Không dùng camera/vision
- Pose wood lấy từ Gazebo ground truth/spawn info
- Pose box và size box lấy từ Gazebo ground truth/spawn config
- /drl_pickplace chạy với execute=true
- Không còn log "execution skipped" ở kết quả cuối
- Robot thật sự di chuyển trong Gazebo
- Gripper có mở/đóng theo flow
- Có report execute đầy đủ
```

Nếu execute fail thì phải sửa tiếp. Không được báo cáo “planning success” là hoàn thành.
