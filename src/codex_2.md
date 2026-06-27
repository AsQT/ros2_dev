Bạn hãy thực hiện tách toàn bộ phần Gazebo hiện đang nằm trong package `robot_description` ra thành một package độc lập mới, nhưng ở bước này **không được xóa hoặc phá bất kỳ file Gazebo cũ nào trong `robot_description`**. Mục tiêu là tạo package mới hoạt động song song, sau đó cập nhật `robot_bringup` để dùng package Gazebo mới.

# 1. Nguyên tắc bắt buộc

* Không xóa file cũ trong `robot_description`.
* Không đổi logic robot description / URDF / xacro nếu không thật sự cần.
* Không phá các launch hiện tại đang chạy được.
* Không hard-code absolute path.
* Mọi đường dẫn phải dùng `get_package_share_directory(...)`.
* Phải build được sau khi sửa.
* Phải cập nhật README hoặc tài liệu ngắn mô tả package mới.
* Nếu phát hiện file Gazebo cũ phụ thuộc ngược vào `robot_description`, phải giữ dependency đúng: package Gazebo mới được phụ thuộc vào `robot_description`, không được bắt `robot_description` phụ thuộc ngược vào package Gazebo.

# 2. Bước 1 — Audit hiện trạng Gazebo trong `robot_description`

Trước khi sửa, hãy kiểm tra toàn bộ các file liên quan Gazebo trong repo, tối thiểu gồm:

```bash
find src/robot_description -iname "*gazebo*" -o -iname "*.world" -o -iname "*.sdf" -o -iname "*.urdf" -o -iname "*.xacro"
find src/robot_description -type f | grep -E "gazebo|world|spawn|wood|box|model|sdf"
grep -R "robot_description.*gazebo\|gazebo.launch\|random_wood\|spawn\|world" -n src/robot_description src/robot_bringup
```

Sau đó lập danh sách:

* Launch Gazebo hiện tại nằm ở đâu.
* Script spawn/random object hiện tại nằm ở đâu.
* World/model/SDF/mesh/config Gazebo hiện tại nằm ở đâu.
* `robot_bringup` hiện đang include hoặc gọi launch Gazebo như thế nào.
* Những file nào nên copy sang package mới.
* Những file nào chỉ nên tham chiếu từ `robot_description`.

# 3. Bước 2 — Tạo package mới

Tạo package mới tên:

```text
robot_gazebo
```

Ưu tiên dùng `ament_cmake` nếu repo hiện tại chủ yếu dùng `ament_cmake`, vì package này chủ yếu chứa launch, world, model, script và asset. Nếu cấu trúc repo đang dùng `ament_python` cho các script Gazebo thì có thể chọn `ament_python`, nhưng phải giải thích lý do trong báo cáo.

Cấu trúc package đề xuất:

```text
src/robot_gazebo/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── gazebo.launch.py
│   └── ...
├── gazebo/
│   ├── random_wood_blocks.py
│   ├── random_box.py
│   └── ...
├── worlds/
│   └── ...
├── models/
│   └── ...
├── config/
│   └── ...
└── README.md
```

Không bắt buộc phải đúng 100% như cây trên, nhưng phải tách rõ:

* `launch/`: launch Gazebo.
* `gazebo/` hoặc `scripts/`: script Python spawn object/random object.
* `worlds/`: file world.
* `models/`: model SDF/Gazebo.
* `config/`: config liên quan Gazebo nếu có.

# 4. Bước 3 — Copy/port từ `robot_description`

Tham khảo Gazebo launch hiện có trong `robot_description`, sau đó tạo phiên bản mới trong `robot_gazebo`.

Yêu cầu:

* Copy logic launch Gazebo hiện tại sang `robot_gazebo/launch/...`.
* Copy các script spawn/random object cần thiết sang `robot_gazebo/gazebo/` hoặc `robot_gazebo/scripts/`.
* Copy world/model Gazebo cần thiết sang `robot_gazebo/worlds/` và `robot_gazebo/models/`.
* Không xóa bản cũ trong `robot_description`.
* Khi port launch, sửa đường dẫn asset từ `robot_description` sang `robot_gazebo` nếu asset đó đã copy sang package mới.
* Những file robot URDF/xacro vẫn lấy từ `robot_description`.
* Không duplicate robot URDF/xacro sang `robot_gazebo`.

Ví dụ nguyên tắc:

```python
robot_description_share = get_package_share_directory("robot_description")
robot_gazebo_share = get_package_share_directory("robot_gazebo")
```

* Robot model, xacro, ros2_control config robot nếu thuộc description thì lấy từ `robot_description`.
* World, model object, spawn script Gazebo nếu đã tách ra thì lấy từ `robot_gazebo`.

# 5. Bước 4 — Cập nhật `CMakeLists.txt` / `package.xml`

Trong `robot_gazebo/package.xml`, thêm dependency cần thiết theo thực tế project, ví dụ:

```xml
<depend>robot_description</depend>
<depend>launch</depend>
<depend>launch_ros</depend>
<depend>xacro</depend>
<depend>robot_state_publisher</depend>
<depend>gazebo_ros</depend>
```

Nếu project dùng Gazebo mới `gz_sim` thay vì Gazebo Classic thì dùng dependency tương ứng đang có trong repo, không tự ý đổi stack Gazebo.

Trong `CMakeLists.txt`, cài đặt đầy đủ các thư mục:

```cmake
install(DIRECTORY
  launch
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)
```

Với script Python Gazebo, cài đặt bằng:

```cmake
install(PROGRAMS
  gazebo/random_wood_blocks.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Chỉ thêm script thật sự tồn tại.

# 6. Bước 5 — Cập nhật `robot_bringup`

Cập nhật các launch trong `robot_bringup` đang gọi Gazebo từ `robot_description` để chuyển sang gọi package mới `robot_gazebo`.

Ví dụ nếu hiện tại có:

```python
get_package_share_directory("robot_description")
...
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(robot_description_share, "launch", "gazebo.launch.py")
    )
)
```

Thì đổi sang:

```python
robot_gazebo_share = get_package_share_directory("robot_gazebo")

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(robot_gazebo_share, "launch", "gazebo.launch.py")
    )
)
```

Yêu cầu:

* Chỉ cập nhật `robot_bringup` để dùng package Gazebo mới.
* Không phá các launch thật/real hardware.
* Không đổi tên launch argument nếu không cần.
* Nếu launch cũ có argument như `use_sim_time`, `world`, `gui`, `paused`, `spawn_objects`, `random_seed`, `number_of_blocks`, phải giữ tương thích hoặc giải thích rõ nếu cần đổi.
* Nếu `robot_bringup/sim.launch.py` đang là entrypoint chính, sau khi sửa vẫn phải chạy qua entrypoint này.

# 7. Bước 6 — Giữ backward compatibility

Tại bước này không xóa launch cũ trong `robot_description`.

Nếu cần, có thể để lại ghi chú trong README rằng:

```text
Gazebo launch has been moved to robot_gazebo.
The old files in robot_description are kept temporarily for backward compatibility and will be removed later after validation.
```

Không được tự ý xóa.

# 8. Bước 7 — Build và test bắt buộc

Sau khi sửa, chạy tối thiểu:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_description robot_gazebo robot_bringup --symlink-install
source install/setup.bash
```

Sau đó kiểm tra package:

```bash
ros2 pkg list | grep robot_gazebo
ros2 pkg prefix robot_gazebo
```

Test launch trực tiếp package mới:

```bash
ros2 launch robot_gazebo gazebo.launch.py
```

Test qua bringup:

```bash
ros2 launch robot_bringup sim.launch.py
```

Nếu tên launch thực tế khác, hãy dùng tên launch hiện có trong repo và báo cáo lại.

# 9. Bước 8 — Báo cáo sau khi hoàn thành

Tạo file báo cáo:

```text
src/robot_gazebo/GAZEBO_MIGRATION_REPORT.md
```

Nội dung bắt buộc:

* Gazebo cũ nằm ở đâu trong `robot_description`.
* Đã tạo package mới `robot_gazebo` gồm những file/thư mục nào.
* Những file nào được copy/port từ `robot_description`.
* Những file nào vẫn được tham chiếu từ `robot_description`.
* `robot_bringup` đã được cập nhật ở file nào.
* Lệnh build đã chạy.
* Lệnh launch đã test.
* Lỗi còn tồn tại nếu có.
* Danh sách file đã sửa.

# 10. Điều kiện hoàn thành

Nhiệm vụ chỉ được xem là xong khi:

* `robot_gazebo` tồn tại như một ROS2 package độc lập.
* `colcon build --packages-select robot_description robot_gazebo robot_bringup --symlink-install` chạy thành công.
* `robot_bringup` đã chuyển sang include Gazebo launch từ `robot_gazebo`.
* Không xóa file Gazebo cũ trong `robot_description`.
* Có báo cáo `GAZEBO_MIGRATION_REPORT.md`.
