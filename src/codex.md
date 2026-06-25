Bạn hãy thực hiện nhiệm vụ sau trong workspace ROS 2 hiện tại. Mục tiêu là tạo **demo pick_place bằng RL trong Gazebo**, chưa dùng YOLO/xử lý ảnh, mà dùng **Gazebo ground truth / spawn info làm perception tạm thời**.

# 1. Bối cảnh

Hiện tại chưa build kịp phần xử lý ảnh / YOLO trong mô phỏng, nhưng cần chạy được demo:

```text
Gazebo + robot + box 3cm + RL pick_place
```

Thông tin vật cần gắp sẽ lấy từ:

```text
Gazebo ground truth
hoặc
thông tin spawn object
```

Không cần dùng camera, không cần YOLO, không cần image topic.

Các package liên quan hiện có:

```text
robot_description
robot_drl
robot_task_manager
robot_bringup
```

Hiện trong repo đã có file tham khảo:

```text
robot_description/gazebo/random_wood_block.py
```

và có launch Gazebo hiện có, ví dụ:

```text
gazebo.launch.py
sim.launch.py
```

Hãy tự tìm đúng vị trí file trong repo.

Hiện có model box tương tự model wood, nằm trong:

```text
robot_description/worlds/box
```

hoặc vị trí tương tự trong `robot_description/worlds/`.

# 2. Yêu cầu quan trọng

## 2.1. Không phá code cũ

Ưu tiên **tạo file mới**, không sửa các file đang chạy ổn.

Chỉ được sửa file cũ khi thật sự bắt buộc, ví dụ:

```text
- CMakeLists.txt để install file launch/script mới
- package.xml nếu thiếu dependency bắt buộc
- setup.py/setup.cfg nếu package Python cần install entry point mới
```

Nếu phải sửa file cũ, phải sửa tối thiểu và ghi rõ trong báo cáo:

```text
- đã sửa file nào
- lý do sửa
- nội dung sửa có ảnh hưởng gì không
```

Không được thay đổi logic hiện có của các action/task đang chạy ổn nếu không cần thiết.

# 3. Nhiệm vụ 1 — Đưa box vào Gazebo

Hiện tại box chưa spawn được vào Gazebo. Bạn phải làm cho box xuất hiện được trong Gazebo.

Hãy dựa trên logic của:

```text
robot_description/gazebo/random_wood_block.py
```

để tạo file mới, ví dụ:

```text
robot_description/gazebo/spawn_pick_box.py
```

hoặc tên hợp lý hơn.

Yêu cầu:

```text
- Spawn được box vào Gazebo.
- Box là khối lập phương kích thước 3 cm.
- Kích thước: 0.03 x 0.03 x 0.03 m.
- Có thể dùng model có sẵn trong robot_description/worlds/box nếu đúng.
- Nếu model có sẵn chưa đúng hoặc không spawn được, tạo model SDF tối thiểu mới để spawn được box 3 cm.
- Không xóa model wood hoặc các model cũ.
```

Vì robot đang đặt trên bàn trong Gazebo, tọa độ Z của box phải tính thêm chiều cao bàn.

Yêu cầu xử lý Z:

```text
box_center_z = table_height + box_size_z / 2
```

Trong đó:

```text
box_size_z = 0.03 m
```

Nếu có thể đọc được `table_height` từ world/model hiện có thì dùng giá trị đó. Nếu không đọc tự động được, hãy tạo launch parameter:

```text
table_height
```

và đặt default hợp lý theo world hiện tại. Không hard-code tùy tiện nếu có thể tránh.

# 4. Nhiệm vụ 2 — Publish ground truth / spawn info làm perception tạm

Sau khi spawn box, cần publish thông tin vật thể cho pipeline pick_place đọc.

Tạo node/topic mới, ví dụ:

```text
/sim/pick_box_info
```

hoặc:

```text
/sim/object_info
```

Nội dung tối thiểu phải có:

```text
- object name
- pose của box trong frame dùng cho robot/Gazebo
- size_x = 0.03
- size_y = 0.03
- size_z = 0.03
```

Nếu không muốn tạo custom msg mới, có thể dùng các message chuẩn có sẵn, ví dụ:

```text
geometry_msgs/PoseStamped
visualization_msgs/Marker
```

Nhưng phải đảm bảo task/action đọc được cả pose và size. Nếu dùng nhiều topic thì đặt tên rõ ràng, ví dụ:

```text
/sim/pick_box_pose
/sim/pick_box_size
```

Yêu cầu quan trọng:

```text
- Không dùng YOLO.
- Không dùng camera.
- Không cần xử lý ảnh.
- Đây là mock/sim perception lấy từ Gazebo ground truth hoặc spawn info.
```

# 5. Nhiệm vụ 3 — Tạo launch Gazebo demo mới

Tạo launch file mới, không sửa launch cũ nếu không bắt buộc.

Tên đề xuất:

```text
robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py
```

hoặc vị trí phù hợp hơn nếu repo đang tổ chức khác.

Launch mới phải dựa trên launch Gazebo hiện có, ví dụ:

```text
gazebo.launch.py
sim.launch.py
```

Yêu cầu launch mới:

```text
- Khởi động Gazebo với robot như pipeline hiện tại.
- Spawn bàn/world giống cấu hình đang dùng.
- Spawn box 3 cm lên bàn.
- Publish box info làm perception tạm.
- Khởi động các node/action cần thiết từ robot_drl và robot_task_manager để chạy demo pick_place RL.
- Có parameter để bật/tắt random vị trí box nếu cần.
- Có parameter table_height.
- Có parameter box_size mặc định 0.03.
- Có parameter gripper_close_width mặc định 0.025.
```

Không được làm hỏng các launch hiện có.

# 6. Nhiệm vụ 4 — Dùng RL hiện có để chạy pick_place trong Gazebo

Hãy kiểm tra các package:

```text
robot_drl
robot_task_manager
```

và dùng những gì đã có để thực hiện demo pick_place.

Yêu cầu luồng demo:

```text
1. Launch Gazebo.
2. Spawn box 3 cm trên bàn.
3. Lấy pose box từ /sim/pick_box_info hoặc ground truth provider.
4. Tính target_pick từ pose box.
5. Tính pre_pick = target_pick + offset Z.
6. Dùng RL planner để đi tới pre_pick.
7. Dùng MoveToPoseCartesian để hạ xuống vị trí gắp.
8. Đóng gripper.
9. Gripper close width dùng 0.025 m vì box 3 cm, để đảm bảo gắp được.
10. Nâng lên theo Z.
11. Dùng RL planner đi tới pre_place.
12. Hạ xuống vị trí đặt.
13. Mở gripper.
14. Kết thúc demo.
```

Thông số mặc định:

```text
box_size = 0.03 m
gripper_close_width = 0.025 m
pre_pick_z_offset = 0.05 m
pre_place_z_offset = 0.05 m
```

Vì robot đặt trên bàn, mọi tọa độ Z liên quan đến pick/place phải cộng đúng chiều cao bàn/world.

Ví dụ:

```text
box_center_z = table_height + 0.015
target_pick_z = table_height + 0.03 hoặc giá trị phù hợp với TCP/gripper
pre_pick_z = target_pick_z + 0.05
```

Phải kiểm tra thực tế frame robot đang dùng là `world`, `base_link`, hay frame khác. Không được đoán sai frame. Nếu cần transform thì dùng TF2.

# 7. Nếu gripper trong Gazebo không gắp vật ổn

Nếu gripper physics trong Gazebo chưa đủ ổn để giữ box 3 cm, hãy xử lý theo thứ tự ưu tiên:

```text
1. Ưu tiên dùng gripper/physics hiện có nếu gắp được.
2. Nếu vật bị rơi/trượt và demo không ổn, tạo thêm node helper mới cho mô phỏng, ví dụ:
   robot_task_manager/sim/sim_grasp_helper_node.py
```

Node helper này chỉ dùng cho Gazebo demo, không ảnh hưởng robot thật.

Logic helper nếu cần:

```text
- Khi gripper đóng và TCP gần box trong ngưỡng cho phép, coi như attach box vào end-effector.
- Trong lúc attach, cập nhật pose box theo end-effector hoặc dùng cơ chế attach phù hợp của Gazebo nếu có.
- Khi gripper mở tại vị trí đặt, detach box.
```

Yêu cầu:

```text
- Helper chỉ dùng trong launch demo mới.
- Không ảnh hưởng các action thật.
- Phải ghi rõ trong báo cáo nếu có dùng helper.
```

# 8. Ép chạy được demo

Bạn không chỉ tạo file. Bạn phải build và test đến khi chạy được demo.

Bắt buộc thực hiện:

```bash
cd ~/ros2_dev
colcon build --symlink-install
source install/setup.bash
```

Sau đó chạy launch demo mới, ví dụ:

```bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py
```

Nếu launch command thực tế khác, hãy ghi đúng command trong báo cáo.

Bạn phải kiểm tra:

```text
- Gazebo mở được.
- Robot xuất hiện đúng.
- Box 3 cm xuất hiện trên bàn.
- Topic perception tạm publish được pose/size box.
- RL/action pick_place được gọi.
- Robot di chuyển tới vị trí gắp.
- Gripper đóng 0.025 m.
- Robot nâng box hoặc ít nhất thực hiện đầy đủ chuỗi pick_place trong sim.
- Robot đi tới vị trí đặt.
- Gripper mở.
- Action trả về success hoặc log hoàn thành rõ ràng.
```

Nếu demo không chạy ngay, phải tự debug cho đến khi chạy được. Không được dừng ở mức “đã viết code”.

# 9. Không được dùng YOLO/camera cho demo này

Demo này không phụ thuộc perception thật.

Không thêm yêu cầu:

```text
- camera plugin
- image topic
- YOLO node
- depth image
- point cloud
- camera calibration
```

Trừ khi repo hiện có tự launch sẵn camera, nhưng demo này không được phụ thuộc vào nó.

# 10. Báo cáo bắt buộc

Sau khi làm xong, tạo file báo cáo mới, ví dụ:

```text
rl_pick_place_box_gazebo_demo_report.md
```

Nội dung báo cáo phải có:

```text
1. Mục tiêu demo.
2. Các file mới đã tạo.
3. Các file cũ đã sửa nếu có, kèm lý do.
4. Cách spawn box 3 cm vào Gazebo.
5. Cách tính Z khi robot/box nằm trên bàn.
6. Topic dùng làm mock perception / ground truth perception.
7. Luồng pick_place RL trong Gazebo.
8. Action/node/launch nào được dùng từ robot_drl và robot_task_manager.
9. Lệnh build.
10. Lệnh chạy demo.
11. Lệnh kiểm tra topic box pose/size.
12. Lệnh gọi action nếu cần gọi thủ công.
13. Kết quả test thực tế.
14. Các lỗi đã gặp và cách đã sửa.
15. Giới hạn hiện tại: chưa dùng YOLO/xử lý ảnh, đang dùng Gazebo ground truth.
16. Hướng thay thế sau này: thay mock perception bằng YOLO/depth perception.
```

# 11. Tiêu chí nghiệm thu

Chỉ coi là hoàn thành khi đạt các tiêu chí sau:

```text
- colcon build thành công.
- Launch demo mới chạy được.
- Box 3 cm xuất hiện trong Gazebo trên bàn.
- Pose/size box được publish qua topic hoặc provider rõ ràng.
- Robot dùng pipeline RL/task_manager hiện có để chạy chuỗi pick_place.
- Không phá các launch/action/package hiện có.
- Có báo cáo hướng dẫn chạy đầy đủ.
```

# 12. Lưu ý cuối

Mục tiêu không phải làm perception thật. Mục tiêu là chạy được demo:

```text
RL pick_place trong Gazebo bằng Gazebo ground truth / spawn info.
```

Hãy ưu tiên giải pháp đơn giản, ổn định, dễ chạy demo, không làm phức tạp hệ thống bằng YOLO/camera ở giai đoạn này.
