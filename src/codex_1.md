Hiện tại package `robot_task_executor` đang chứa nhiều service, nhưng thực tế chỉ một phần service phục vụ cho DRL/RL đang được dùng. Mục tiêu là **không rename trực tiếp ngay**, mà tạo package mới tên:

```text
robot_drl_executor
```

Sau đó migrate dần những thành phần thật sự liên quan tới DRL/RL từ `robot_task_executor` sang package mới, rồi cập nhật các package đang gọi như:

```text
robot_drl
robot_task_manager
```

để dùng package mới.

Yêu cầu làm việc nghiêm ngặt:

---

## 1. Không được phá package cũ ngay

Không xóa, không rename trực tiếp `robot_task_executor`.

Package cũ phải được giữ nguyên trong giai đoạn đầu để có thể rollback.

Không được sửa hàng loạt kiểu search/replace mù quáng từ `robot_task_executor` sang `robot_drl_executor`.

Phải audit trước rồi mới sửa.

---

## 2. Đọc toàn bộ report trước khi sửa

Trước khi chỉnh code, bắt buộc đọc các file:

```text
*report*.md
```

trong các package liên quan, đặc biệt:

```text
robot_task_executor
robot_task_manager
robot_drl
robot_bringup
robot_description
```

Mục tiêu đọc report:

* Xác định flow DRL/RL nào hiện đang chạy được.
* Xác định launch/test nào đang dùng `robot_task_executor`.
* Xác định service/action/topic nào thật sự cần cho DRL.
* Xác định những service không dùng hoặc không liên quan để không đưa sang package mới.
* Xác định các lệnh build/test đã từng chạy được.

Sau khi đọc report, tạo file báo cáo audit trước khi sửa:

```text
robot_drl_executor_migration_audit_report.md
```

Nội dung báo cáo phải có:

```text
1. Danh sách file report đã đọc
2. Flow DRL/RL hiện đang chạy được
3. Các service/action/topic đang được dùng bởi DRL
4. Các file đang import/gọi robot_task_executor
5. Các thành phần không dùng hoặc nghi ngờ không dùng
6. Kế hoạch migrate cụ thể
```

---

## 3. Tạo package mới `robot_drl_executor`

Tạo package mới song song với package cũ:

```text
src/robot_drl_executor
```

Package mới phải có cấu trúc ROS2 chuẩn, bao gồm tối thiểu:

```text
robot_drl_executor/
├── package.xml
├── CMakeLists.txt hoặc setup.py/setup.cfg tùy package gốc
├── robot_drl_executor/
│   └── ...
├── launch/
├── config/
├── README.md
└── robot_drl_executor_migration_report.md
```

Tên node, executable, namespace, parameter phải ưu tiên dùng tên mới:

```text
robot_drl_executor
robot_drl_executor_node
```

Không dùng tên mới nửa vời như `robot_task_executor_node` trong package mới, trừ khi cần giữ compatibility tạm thời và phải ghi rõ trong report.

---

## 4. Chỉ migrate thành phần DRL/RL thật sự cần

Không copy toàn bộ `robot_task_executor` sang package mới.

Chỉ đưa sang `robot_drl_executor` những phần đang phục vụ cho các action/flow DRL trong `robot_task_manager` và `robot_drl`.

Cần kiểm tra các thành phần như:

```text
service clients
service servers
action clients
action servers
launch files
config files
params
Python modules
C++ nodes nếu có
```

Phải trace dependency bằng các lệnh kiểu:

```bash
grep -R "robot_task_executor" -n src/
grep -R "task_executor" -n src/
grep -R "drl" -n src/robot_task_manager src/robot_drl src/robot_task_executor
grep -R "rl" -n src/robot_task_manager src/robot_drl src/robot_task_executor
```

Nếu một service không được gọi bởi DRL/RL flow thì không đưa sang package mới.

Nếu chưa chắc có dùng hay không, giữ ở package cũ và ghi vào report là “chưa migrate vì chưa xác nhận dependency”.

---

## 5. Cập nhật các package đang gọi

Sau khi package mới đã có đủ service/node cần thiết, cập nhật các package đang gọi từ:

```text
robot_task_executor
```

sang:

```text
robot_drl_executor
```

Tối thiểu cần kiểm tra và cập nhật trong:

```text
robot_drl
robot_task_manager
robot_bringup nếu có launch liên quan
```

Các nơi cần kiểm tra:

```text
package.xml
CMakeLists.txt
setup.py
setup.cfg
launch/*.launch.py
config/*.yaml
README.md
source code import
service client names
node executable names
```

Không được để sót dependency cũ nếu flow DRL đã chuyển sang package mới.

Tuy nhiên không được xóa `robot_task_executor` khỏi workspace nếu vẫn còn flow khác phụ thuộc.

---

## 6. Giữ nguyên hành vi chạy như cũ

Mục tiêu là đổi package executor cho DRL, không thay đổi logic DRL.

Không tự ý sửa:

```text
reward
observation
action space
MoveIt planning logic
Gazebo spawn logic
pick/place pose logic
robot trajectory logic
```

Trừ khi bắt buộc do dependency package đổi tên, và phải giải thích rõ trong report.

Flow sau migration phải chạy tương đương như trước.

---

## 7. Build bắt buộc đến khi sạch lỗi

Sau khi sửa, bắt buộc build workspace:

```bash
cd ~/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Nếu build lỗi, không được dừng lại ở báo cáo lỗi. Phải tự sửa tiếp cho đến khi build được.

Nếu build toàn workspace quá lâu hoặc lỗi do package không liên quan, phải build tối thiểu các package liên quan:

```bash
colcon build --symlink-install --packages-select robot_drl_executor robot_drl robot_task_manager robot_bringup
```

Sau đó tiếp tục xử lý dependency còn thiếu.

---

## 8. Test bắt buộc đến khi chạy được như cũ

Sau khi build, phải test lại các flow DRL/RL đã được xác định từ report.

Không được tự chọn test tùy tiện. Phải dựa trên các launch/test đã ghi trong các file `*report*.md`.

Cần kiểm tra tối thiểu:

```text
1. Package robot_drl_executor được build và source thành công
2. Node/service của robot_drl_executor chạy được
3. robot_drl gọi được executor mới
4. robot_task_manager action liên quan DRL gọi được executor mới
5. Launch DRL/RL cũ chạy được như trước
6. Không còn lỗi import robot_task_executor trong flow DRL đã migrate
```

Nếu có mock hardware thì ưu tiên test bằng mock hardware trước.

Nếu report cũ có flow Gazebo DRL đang chạy được thì test lại Gazebo sau khi mock chạy ổn.

Không được dừng giữa chừng chỉ vì gặp lỗi launch/runtime. Phải sửa đến khi chạy được tương đương trạng thái cũ.

---

## 9. Không xóa package cũ sau migration

Sau khi `robot_drl_executor` chạy được, vẫn không xóa `robot_task_executor`.

Chỉ cập nhật DRL flow sang package mới.

Nếu muốn loại bỏ package cũ thì để thành task riêng sau.

---

## 10. Cập nhật tài liệu

Sau khi hoàn tất, cập nhật hoặc tạo các file:

```text
src/robot_drl_executor/README.md
src/robot_drl_executor/robot_drl_executor_migration_report.md
```

Nếu các package khác có README liên quan tới DRL/RL executor thì cập nhật lại tên package mới:

```text
robot_drl
robot_task_manager
robot_bringup
```

README phải mô tả rõ:

```text
- Package robot_drl_executor dùng để làm gì
- Nó thay thế phần nào của robot_task_executor
- Các service/action/topic chính
- Package nào gọi nó
- Lệnh build
- Lệnh chạy/test
- Những thành phần chưa migrate và lý do
```

---

## 11. Báo cáo cuối cùng bắt buộc

Tạo file:

```text
robot_drl_executor_migration_report.md
```

Nội dung gồm:

```text
1. Mục tiêu migration
2. Các report đã đọc
3. Các file đã tạo mới
4. Các file đã sửa
5. Các service/action/topic đã migrate
6. Các dependency đã đổi từ robot_task_executor sang robot_drl_executor
7. Các thành phần cố ý chưa migrate
8. Lệnh build đã chạy
9. Kết quả build
10. Lệnh test đã chạy
11. Kết quả test
12. Lỗi gặp phải và cách đã sửa
13. Trạng thái cuối cùng: chạy được/chưa chạy được
```

Không được báo cáo chung chung. Phải ghi rõ đường dẫn file và kết quả thực tế.

---

## 12. Tiêu chí hoàn thành

Chỉ được coi là hoàn thành khi đạt đủ:

```text
- Có package mới robot_drl_executor
- robot_task_executor vẫn còn nguyên, không bị xóa
- Các thành phần DRL/RL cần thiết đã được migrate sang robot_drl_executor
- robot_drl và robot_task_manager đã gọi package mới trong flow DRL
- Workspace build được
- Flow DRL/RL chạy được như trước
- Có report audit trước khi sửa
- Có report migration sau khi sửa
- Có README cho robot_drl_executor
```

Nếu chưa đạt đủ các tiêu chí trên, phải tiếp tục sửa, build và test.
