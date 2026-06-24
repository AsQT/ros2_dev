# README cho ArUco / ChArUco Extrinsic Calibration

README này mô tả node:

```text
robot_vision_pipeline/calib/aruco_extrinsic_calibrator_node.py
```

Node này dùng để hiệu chuẩn ngoại camera RGB so với hệ tọa độ world được định nghĩa trên board ArUco / ChArUco. Nội dung dưới đây bám theo code hiện tại của node, file launch `launch/aruco_extrinsic_calibrator.launch.py`, file board layout `config/aruco_board_layout.yaml` và file kết quả `config/aruco_extrinsic_result.yaml`.

## 1. Mục đích của node

`aruco_extrinsic_calibrator_node.py` dùng ảnh màu và `CameraInfo` của camera để tìm pose của camera so với board ArUco / ChArUco. Board được xem như một mặt phẳng đã biết kích thước thật, còn các corner detect được trên ảnh là điểm ảnh 2D. Từ cặp điểm 3D trên board và điểm 2D trên ảnh, node dùng `solvePnP` để tính extrinsic.

Node xác định phép biến đổi giữa:

| Frame | Mặc định | Ý nghĩa |
| ----- | -------- | ------- |
| `world_frame` | `aruco_world` | Hệ tọa độ world đặt trên board. |
| `camera_frame` | `camera_color_optical_frame` | Hệ tọa độ camera RGB. |

Kết quả quan trọng:

- `T_world_camera`: ma trận biến đổi từ camera sang world, cho biết pose camera trong hệ world.
- `T_camera_world`: ma trận biến đổi từ world/board sang camera, tương ứng với `rvec`, `tvec` ban đầu từ `solvePnP`.
- Vị trí camera trong world: `x`, `y`, `z`.
- Góc quay camera trong world: `roll`, `pitch`, `yaw`.
- Quaternion của camera trong world.
- Reprojection error: `mean_px`, `max_px`.
- File YAML kết quả: `config/aruco_extrinsic_result.yaml`.
- TF nếu pose được chấp nhận: `aruco_world -> camera_color_optical_frame`.

Kết quả này là cơ sở để các node xử lý thị giác sau này chuyển tọa độ từ camera sang hệ world/robot. Ví dụ, một điểm 3D đo trong camera frame có thể được biến đổi sang world frame bằng extrinsic đã lưu.

## 2. Dữ liệu đầu vào và đầu ra

Đầu vào:

| Input | Kiểu dữ liệu | Mặc định trong code | Ý nghĩa |
| ----- | ------------ | ------------------- | ------- |
| Ảnh màu | `sensor_msgs/Image` | `/camera/camera/color/image_raw` | Ảnh RGB/BGR từ RealSense hoặc camera RGB. |
| CameraInfo | `sensor_msgs/CameraInfo` | `/camera/camera/color/camera_info` | Nội camera: `fx`, `fy`, `cx`, `cy` và distortion. |
| Board layout YAML | YAML file | `config/aruco_board_layout.yaml` | Kích thước marker, kích thước ô cờ, số ô, dictionary, frame và layout marker fallback. |

Đầu ra:

| Output | Kiểu dữ liệu | Mặc định trong code | Ý nghĩa |
| ------ | ------------ | ------------------- | ------- |
| Debug image | `sensor_msgs/Image` | `/vision/aruco_calib/debug_image` | Ảnh debug có marker, ChArUco corner, điểm chiếu lại, trục X/Y world, trạng thái pose. |
| TF | `tf2` | `aruco_world -> camera_color_optical_frame` | Transform từ world frame của board sang camera frame. |
| Result YAML | YAML file | `config/aruco_extrinsic_result.yaml` | Lưu kết quả extrinsic, error, intrinsics, thông tin board và debug metadata. |

Node không dùng depth image. Calibration dựa trên quan hệ 2D pixel và 3D point trên board đã biết kích thước thật.

## 3. Các tham số quan trọng

Các parameter được khai báo trực tiếp trong `aruco_extrinsic_calibrator_node.py`:

| Parameter | Mặc định | Ý nghĩa |
| --------- | -------- | ------- |
| `image_topic` | `/camera/camera/color/image_raw` | Topic ảnh màu đầu vào. |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Topic `CameraInfo` của camera màu. |
| `debug_image_topic` | `/vision/aruco_calib/debug_image` | Topic ảnh debug. |
| `marker_size` | `0.021` | Kích thước cạnh marker ArUco, đơn vị mét. |
| `square_size` | `0.029` | Kích thước cạnh ô ChArUco/chessboard, đơn vị mét. |
| `squares_x` | `6` | Số ô theo chiều X của board. |
| `squares_y` | `8` | Số ô theo chiều Y của board. |
| `aruco_dictionary` | `DICT_4X4_50` | Dictionary dùng để detect marker. |
| `board_layout_file` | `config/aruco_board_layout.yaml` | File YAML mô tả board. |
| `world_frame` | `aruco_world` | Tên world frame của board. |
| `camera_frame` | `camera_color_optical_frame` | Tên camera frame dùng khi publish TF. |
| `publish_tf` | `True` | Nếu `True`, publish TF khi pose được ACCEPTED. |
| `publish_bad_tf_for_debug` | `False` | Nếu pose REJECTED, chỉ publish TF debug khi tham số này là `True`. |
| `save_result` | `True` | Nếu `True`, lưu YAML khi pose được ACCEPTED. |
| `result_yaml_path` | tự suy ra, thường là `config/aruco_extrinsic_result.yaml` | Đường dẫn lưu kết quả. |
| `min_detected_markers` | `2` | Số marker hợp lệ tối thiểu cho nhánh ArUco fallback. |
| `min_charuco_corners` | `6` | Số ChArUco corner tối thiểu để dùng nhánh ChArUco. |
| `show_debug_axes` | `True` | Nếu `True`, vẽ trục X/Y world trên debug image khi pose hợp lệ. |
| `axis_length` | `0.08` | Độ dài trục debug X/Y, đơn vị mét. |
| `use_ransac` | `True` | Nếu `True`, thử `cv2.solvePnPRansac` trước. |
| `ransac_reprojection_error` | `3.0` | Ngưỡng reprojection error của RANSAC, đơn vị pixel. |
| `ransac_confidence` | `0.99` | Confidence của RANSAC. |
| `max_allowed_reprojection_error` | `5.0` | Ngưỡng mean reprojection error để ACCEPT pose, đơn vị pixel. |
| `corner_order_mode` | `opencv` | Thứ tự corner khi dùng ArUco fallback. Hỗ trợ `opencv`, `flip_vertical`, `flip_horizontal`, `reverse`. |
| `print_marker_debug` | `True` | In log debug correspondence marker/corner. |
| `use_board_roi` | `False` | Có đọc từ YAML, nhưng code hiện cảnh báo ROI cropping đang bị tắt cho ChArUco calibration. |
| `flip_charuco_y_axis` | `True` | Đảo hàng ChArUco để world Y+ hướng lên. |
| `layout_rows_from_top` | `False` | Cách hiểu row trong YAML marker layout. |

Đơn vị xử lý của kích thước board là mét:

```text
marker_size = 0.021  # 21 mm
square_size = 0.029  # 29 mm
```

File `config/aruco_board_layout.yaml` hiện cũng khai báo:

```yaml
marker_size: 0.021
square_size: 0.029
squares_x: 6
squares_y: 8
aruco_dictionary: DICT_4X4_50
world_frame: aruco_world
camera_frame: camera_color_optical_frame
use_board_roi: false
```

Lưu ý: `_load_board_layout()` có thể override các giá trị `marker_size`, `square_size`, `squares_x`, `squares_y`, `aruco_dictionary`, `world_frame`, `camera_frame`, `use_board_roi`, `flip_charuco_y_axis`, `layout_rows_from_top` bằng dữ liệu trong YAML.

## 4. Quy ước hệ tọa độ world

Theo code và `config/aruco_board_layout.yaml`, world frame của board được định nghĩa như sau:

| Thành phần | Quy ước |
| ---------- | ------- |
| Gốc tọa độ | Tâm hình học của board. |
| X+ | Hướng sang phải trên board, theo chiều tăng `col`. |
| Y+ | Hướng lên trên board, theo chiều tăng `row_world`. |
| Z+ | Hướng ra khỏi mặt board. |

Debug image chỉ vẽ trục X và Y:

- `O_world`: gốc world ở tâm board.
- `X+`: mũi tên màu đỏ.
- `Y+`: mũi tên màu xanh lá.

Node không vẽ trục Z trên debug image. Việc này đủ để kiểm tra nhanh hướng trái/phải và lên/xuống của board trên ảnh.

Ý nghĩa của `flip_charuco_y_axis`:

- OpenCV đánh số hàng ChArUco theo chiều từ trên xuống ảnh/board.
- World frame của node lại muốn Y+ hướng lên trên board.
- Khi `flip_charuco_y_axis=True`, node đổi `row_opencv` thành `row_world` bằng:

```text
row_world = (corners_y - 1) - row_opencv
```

Nhờ vậy ChArUco corner được đưa vào hệ world có Y+ đi lên.

Ý nghĩa của `layout_rows_from_top`:

- Tham số này áp dụng cho `marker_layout` trong YAML khi node fallback sang ArUco marker corner.
- Nếu `layout_rows_from_top=False`, `row` trong YAML được hiểu theo hệ world thông thường: `row=0` là hàng dưới cùng.
- Nếu `layout_rows_from_top=True`, node hiểu YAML được nhập từ trên xuống và đổi sang row world bằng:

```text
row_world = (squares_y - 1) - row_yaml
```

## 5. Luồng xử lý tổng quát

### Bước 1: Node khởi động và đọc parameter

Khi khởi động, node:

- khai báo topic ảnh, topic `CameraInfo`, topic debug;
- đọc `board_layout_file`;
- xác định `result_yaml_path`;
- tạo ArUco dictionary từ `aruco_dictionary`;
- tạo `cv2.aruco` detector parameters;
- tạo ChArUco board từ `squares_x`, `squares_y`, `square_size`, `marker_size`;
- tạo subscriber ảnh;
- tạo subscriber `CameraInfo`;
- tạo publisher debug image;
- tạo `TransformBroadcaster` để publish TF.

Nếu `result_yaml_path` mặc định đang trỏ vào `install/share`, node có cảnh báo nên override về source config nếu muốn lưu vào source package.

### Bước 2: Nhận CameraInfo

Trong `camera_info_callback()`:

- node lấy ma trận nội camera từ `msg.k`;
- reshape thành ma trận 3x3 `camera_matrix`;
- lấy distortion coefficients từ `msg.d`;
- nếu distortion rỗng thì dùng mảng 0 gồm 5 phần tử;
- lưu `camera_info_frame` từ `msg.header.frame_id`;
- log `fx`, `fy`, `cx`, `cy`, `D`.

`CameraInfo` là bắt buộc vì `solvePnP` cần camera matrix và distortion để suy ra pose từ 2D/3D correspondences.

### Bước 3: Nhận ảnh RGB

Trong `image_callback()`:

- node chuyển ROS `Image` sang ảnh OpenCV BGR bằng `cv_bridge`;
- tạo bản copy `annotated` để vẽ debug;
- nếu chưa có `CameraInfo`, node không chạy pose estimation;
- trong trạng thái chờ `CameraInfo`, node publish debug image với status `Waiting for camera_info`.

### Bước 4: Detect ArUco marker

Node xử lý ảnh như sau:

- chuyển ảnh BGR sang grayscale;
- gọi `cv2.aruco.detectMarkers`;
- lấy `corners`, `ids`, `rejected`;
- nếu không thấy marker, trả lý do `No ArUco markers detected`.

Marker detect được luôn được dùng để vẽ debug image. Riêng tính pose sẽ ưu tiên ChArUco corner trước.

### Bước 5: Ưu tiên nội suy ChArUco corner

Trong `_detect_charuco_board()`:

- node lọc các marker ID bị trùng bằng `_filter_duplicate_aruco_markers_for_charuco`;
- marker ID bị trùng sẽ bị bỏ qua trước khi nội suy ChArUco;
- gọi `cv2.aruco.interpolateCornersCharuco`;
- nếu số corner nội suy được lớn hơn hoặc bằng `min_charuco_corners`, node dùng ChArUco corner làm nguồn điểm cho `solvePnP`;
- đây là nhánh chính vì ChArUco corner thường chính xác hơn 4 góc marker ArUco.

### Bước 6: Nếu ChArUco không đủ thì dùng ArUco fallback

Nếu ChArUco không đủ corner, node chuyển sang nhánh `aruco_fallback`:

- dùng `marker_layout` trong YAML để biết mỗi marker nằm ở `row`, `col` nào;
- từ `row`, `col`, `square_size`, `marker_size`, `board_width`, `board_height`, node tạo tọa độ 3D cho 4 góc marker;
- ghép 4 object points 3D với 4 image points detect được;
- nếu marker ID không có trong YAML thì bỏ qua và log `Detected marker ID not found in fallback layout`;
- nếu marker ID bị trùng trong cùng frame thì bỏ qua;
- nếu số marker hợp lệ nhỏ hơn `min_detected_markers`, node không `solvePnP`.

`corner_order_mode` chỉ ảnh hưởng nhánh fallback này.

## 6. Tạo cặp điểm 2D - 3D

Node tạo hai mảng chính:

| Mảng | Ý nghĩa |
| ---- | ------- |
| `object_points` | Tọa độ 3D của corner trên mặt board, đơn vị mét, `z = 0`. |
| `image_points` | Tọa độ pixel tương ứng trên ảnh. |

Với ChArUco:

- mỗi `charuco_id` được đổi sang `row_world`, `col`;
- object point được tính quanh gốc board ở tâm:

```text
x = (col + 1) * square_size - board_width / 2
y = (row_world + 1) * square_size - board_height / 2
z = 0
```

Với ArUco fallback:

- marker center được tính từ `row`, `col`;
- 4 góc marker được tạo quanh tâm marker với `half = marker_size / 2`;
- mặc định thứ tự corner dùng `corner_order_mode=opencv`.

## 7. Lọc điểm bằng homography cho ChArUco

Vì board là một mặt phẳng, node có bước lọc outlier bằng homography:

- chỉ áp dụng khi `points_source == "charuco"`;
- dùng `cv2.findHomography` với `cv2.RANSAC`;
- input là `object_points[:, :2]` và `image_points`;
- threshold dùng `max(ransac_reprojection_error, 5.0)`;
- nếu số inlier nhỏ hơn `max(4, min_charuco_corners)`, node dừng và báo lỗi;
- nếu pass, chỉ các inlier được đưa vào `solvePnP`.

Bước này giúp loại các ChArUco correspondence bị nội suy sai hoặc không cùng mặt phẳng board.

## 8. SolvePnP

Trước `solvePnP`, node kiểm tra:

- `object_points` và `image_points` không được `None`;
- số điểm 2D và 3D phải bằng nhau;
- cần ít nhất 4 điểm;
- shape phải là `(N, 3)` và `(N, 2)`;
- không có NaN/Inf.

Nếu `use_ransac=True`, node thử:

```text
cv2.solvePnPRansac(...)
```

Nếu RANSAC thất bại, node thử lại:

```text
cv2.solvePnP(...)
```

Nếu `use_ransac=False`, node dùng `solvePnP` trực tiếp.

Kết quả `solvePnP` trả về:

- `rvec`
- `tvec`

Trong quy ước của OpenCV, `rvec`, `tvec` biểu diễn pose của object/world/board trong hệ camera. README này gọi đó là:

```text
T_camera_world
```

## 9. Đảo transform

Node dùng Rodrigues để đổi `rvec` sang rotation matrix:

```text
R_camera_world = Rodrigues(rvec)
t_camera_world = tvec
```

Sau đó đảo transform:

```text
R_world_camera = R_camera_world.T
t_world_camera = -R_world_camera @ t_camera_world
```

Từ đó tạo:

- `T_camera_world`: world/board trong camera frame;
- `T_world_camera`: camera trong world frame.

`T_world_camera` là ma trận quan trọng nhất khi muốn biết camera đang nằm ở đâu so với board/world.

## 10. Quaternion, roll pitch yaw và reprojection error

Từ `R_world_camera`, node tính:

- quaternion `x`, `y`, `z`, `w`;
- roll, pitch, yaw bằng `_rotation_matrix_to_rpy()`;
- translation camera trong world `x`, `y`, `z`.

Sau đó node tính reprojection error:

1. Chiếu lại `object_points` lên ảnh bằng `cv2.projectPoints`.
2. So sánh điểm chiếu lại với `image_points` detect thật.
3. Tính:

```text
mean_error = mean(norm(projected - image_points))
max_error  = max(norm(projected - image_points))
```

Pose được ACCEPTED nếu:

```text
mean_error <= max_allowed_reprojection_error
```

Mặc định:

```text
max_allowed_reprojection_error = 5.0 px
```

Nếu mean reprojection error lớn hơn ngưỡng, pose bị REJECTED.

## 11. Publish TF, lưu YAML và publish debug image

Nếu pose ACCEPTED:

- publish TF `world_frame -> camera_frame`;
- lưu `aruco_extrinsic_result.yaml` nếu `save_result=True`;
- debug image hiển thị status `ACCEPTED`;
- log ma trận, pose, quaternion, reprojection error.

Nếu pose REJECTED:

- không lưu kết quả chính thức;
- không publish TF;
- chỉ publish TF khi `publish_bad_tf_for_debug=True`;
- debug image hiển thị status `REJECTED` và lý do lỗi.

Debug image trên `/vision/aruco_calib/debug_image` có thể chứa:

- marker ArUco detect được;
- ChArUco corners nếu có;
- điểm chiếu lại màu magenta để kiểm tra sai số;
- gốc world tại tâm board;
- trục `X+` và `Y+`;
- trạng thái `ACCEPTED`, `REJECTED` hoặc `Waiting for camera_info`;
- lý do lỗi nếu có;
- danh sách marker ID detect được;
- số ChArUco corner detect được;
- reprojection mean/max.

## 12. Cấu trúc file YAML kết quả

File kết quả mặc định:

```text
config/aruco_extrinsic_result.yaml
```

Các nhóm dữ liệu chính:

| Nhóm | Ý nghĩa |
| ---- | ------- |
| `world_frame` | Tên world frame, mặc định `aruco_world`. |
| `camera_frame` | Tên camera frame, mặc định `camera_color_optical_frame`. |
| `extrinsic_matrix/T_world_camera` | Ma trận 4x4 camera trong world. |
| `extrinsic_matrix/T_camera_world` | Ma trận 4x4 world/board trong camera. |
| `camera_pose_in_world` | `x`, `y`, `z`, `roll`, `pitch`, `yaw`. |
| `camera_pose_quaternion` | Quaternion camera trong world. |
| `reprojection_error` | `mean_px`, `max_px`. |
| `camera_matrix` | Ma trận nội camera K lấy từ `CameraInfo`. |
| `distortion_coefficients` | Distortion coefficients lấy từ `CameraInfo`. |
| `board` | `marker_size`, `square_size`, `squares_x`, `squares_y`, `board_width`, `board_height`, dictionary, layout file. |
| `world_convention` | Quy ước gốc và hướng trục world. |
| `debug` | Nguồn điểm, số điểm dùng, số inlier homography/RANSAC, đường dẫn lưu, các cờ trục. |
| `translation` | Bản copy tiện đọc của camera position. |
| `rotation_rpy` | Bản copy tiện đọc của roll/pitch/yaw. |
| `rotation_quaternion` | Bản copy tiện đọc của quaternion. |
| `T_world_camera` | Bản copy top-level của ma trận `T_world_camera`. |
| `T_camera_world` | Bản copy top-level của ma trận `T_camera_world`. |

File này là kết quả quan trọng để các node khác biến đổi tọa độ camera sang world/robot.

## 13. Cách chạy node

Build và source workspace:

```bash
cd ~/ros_vision
colcon build --packages-select robot_vision_pipeline --symlink-install
source install/setup.bash
```

Chạy bằng launch file hiện có:

```bash
ros2 launch robot_vision_pipeline aruco_extrinsic_calibrator.launch.py
```

Launch file này truyền hai parameter:

```text
board_layout_file
result_yaml_path
```

Chạy trực tiếp executable:

```bash
ros2 run robot_vision_pipeline aruco_extrinsic_calibrator_node
```

Ví dụ override parameter:

```bash
ros2 run robot_vision_pipeline aruco_extrinsic_calibrator_node --ros-args \
  -p marker_size:=0.021 \
  -p square_size:=0.029 \
  -p result_yaml_path:=/home/asus/ros_vision/src/robot_vision_pipeline/config/aruco_extrinsic_result.yaml
```

Ví dụ chạy launch và ép đường dẫn result về source package:

```bash
ros2 launch robot_vision_pipeline aruco_extrinsic_calibrator.launch.py \
  result_yaml_path:=/home/asus/ros_vision/src/robot_vision_pipeline/config/aruco_extrinsic_result.yaml
```

Executable `aruco_extrinsic_calibrator_node` hiện đã được khai báo trong `setup.py`:

```text
aruco_extrinsic_calibrator_node = robot_vision_pipeline.calib.aruco_extrinsic_calibrator_node:main
```

## 14. Cách kiểm tra kết quả

Kiểm tra camera info:

```bash
ros2 topic echo /camera/camera/color/camera_info
```

Kiểm tra topic ArUco/debug:

```bash
ros2 topic list | grep aruco
```

Mở debug image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Trong `rqt_image_view`, chọn:

```text
/vision/aruco_calib/debug_image
```

Kiểm tra TF:

```bash
ros2 run tf2_ros tf2_echo aruco_world camera_color_optical_frame
```

Calibration ổn khi:

- debug image hiện `Status: ACCEPTED`;
- reprojection error nhỏ hơn ngưỡng, mặc định mean <= `5.0 px`;
- trục `X+` và `Y+` đúng hướng trên board;
- file `config/aruco_extrinsic_result.yaml` được cập nhật;
- `tf2_echo aruco_world camera_color_optical_frame` có transform hợp lý.

## 15. Các lỗi thường gặp và cách xử lý

### 1. `Waiting for camera_info`

Nguyên nhân:

- Chưa chạy camera.
- Sai topic `camera_info_topic`.
- CameraInfo chưa publish.

Cách xử lý:

```bash
ros2 topic list
ros2 topic echo /camera/camera/color/camera_info
```

Nếu topic khác tên, override:

```bash
ros2 run robot_vision_pipeline aruco_extrinsic_calibrator_node --ros-args \
  -p camera_info_topic:=/topic/camera_info
```

### 2. `No ArUco markers detected`

Nguyên nhân:

- Board không nằm trong khung hình.
- Sai `aruco_dictionary`.
- Ánh sáng yếu.
- Marker bị che, mờ hoặc quá nhỏ.

Cách xử lý:

- Đưa board rõ vào camera.
- Kiểm tra `aruco_dictionary`, mặc định `DICT_4X4_50`.
- Tăng ánh sáng, giảm blur.
- Xem ảnh màu camera bằng `rqt_image_view`.

### 3. `Not enough ChArUco corners`

Nguyên nhân:

- Thấy marker nhưng chưa đủ ChArUco corner để `solvePnP`.
- Board quá xa, quá nghiêng hoặc bị che.
- Ảnh mờ làm nội suy corner kém.

Cách xử lý:

- Đặt board gần camera hơn.
- Giữ board phẳng và rõ nét.
- Đảm bảo thấy nhiều marker/corner.
- Chỉ giảm `min_charuco_corners` khi hiểu rõ ảnh hưởng tới độ ổn định pose.

### 4. `Detected marker ID not found in fallback layout`

Nguyên nhân:

- File `aruco_board_layout.yaml` thiếu ID marker.
- Layout YAML không khớp board thật.
- Trong ảnh có marker ngoài board.

Cách xử lý:

- Kiểm tra `marker_layout`.
- Đảm bảo ID trong YAML đúng với ID in trên board.
- Nếu dùng ChArUco đủ corner thì fallback layout ít khi là nguồn chính, nhưng layout vẫn cần đúng khi ChArUco không đủ.

### 5. Reprojection error quá lớn

Nguyên nhân thường gặp:

- Sai `marker_size` hoặc `square_size`.
- Sai `CameraInfo`.
- Sai hướng trục board.
- Sai `flip_charuco_y_axis`.
- Sai `layout_rows_from_top`.
- Sai `corner_order_mode` trong nhánh fallback.
- Board không phẳng hoặc ảnh corner bị mờ/nhiễu.

Cách xử lý:

- Kiểm tra `marker_size=0.021`, `square_size=0.029`.
- Kiểm tra debug image xem trục X/Y có đúng hướng không.
- Thử kiểm tra lại `flip_charuco_y_axis`.
- Kiểm tra lại `marker_layout` nếu đang fallback sang ArUco.
- Đảm bảo `CameraInfo` thuộc đúng camera đang dùng.

### 6. Trục Y bị ngược

Giải thích:

- Pixel ảnh có trục y đi xuống.
- World frame của node muốn Y+ hướng lên trên board.
- Tham số quan trọng là `flip_charuco_y_axis`.

Nếu debug image cho thấy `Y+` sai hướng:

- kiểm tra `flip_charuco_y_axis`;
- kiểm tra `layout_rows_from_top` nếu đang dùng fallback layout;
- kiểm tra comment layout trong `config/aruco_board_layout.yaml`.

### 7. Kết quả lưu sai vị trí

Node cố gắng ưu tiên lưu vào thư mục `config` của source package nếu suy ra được. Nếu chạy từ install/share hoặc đường dẫn mặc định bị suy ra không như mong muốn, hãy override:

```bash
ros2 run robot_vision_pipeline aruco_extrinsic_calibrator_node --ros-args \
  -p result_yaml_path:=/home/asus/ros_vision/src/robot_vision_pipeline/config/aruco_extrinsic_result.yaml
```

Hoặc với launch:

```bash
ros2 launch robot_vision_pipeline aruco_extrinsic_calibrator.launch.py \
  result_yaml_path:=/home/asus/ros_vision/src/robot_vision_pipeline/config/aruco_extrinsic_result.yaml
```

## 16. Ghi chú về độ chính xác

Node không dùng depth để tính extrinsic. Calibration dựa trên:

- tọa độ 3D đã biết của các corner trên board;
- tọa độ 2D pixel detect được trên ảnh;
- nội camera từ `CameraInfo`;
- `solvePnP`.

Depth không cần thiết vì board là vật chuẩn có kích thước thật đã biết. Độ chính xác phụ thuộc vào:

- `marker_size` và `square_size` đúng với board thật;
- `CameraInfo` đúng với camera đang dùng;
- marker/corner detect rõ, không mờ;
- board phẳng;
- board chiếm đủ diện tích ảnh;
- reprojection error nhỏ;
- hướng trục world đúng.

Nếu dùng kết quả này cho robot, nên kiểm tra nhiều frame và chỉ tin kết quả khi debug image ổn định, reprojection error nhỏ và TF hợp lý.

## 17. Sơ đồ luồng xử lý

```text
Camera RGB image
        │
        ▼
Detect ArUco markers
        │
        ▼
Interpolate ChArUco corners
        │
        ├── đủ corner ──► dùng ChArUco 2D-3D points
        │
        └── không đủ ──► fallback dùng ArUco marker layout YAML
                                  │
                                  ▼
                         Tạo object_points + image_points
                                  │
                                  ▼
                         Homography/RANSAC filter
                         chỉ áp dụng cho ChArUco
                                  │
                                  ▼
                              solvePnP
                                  │
                                  ▼
                       T_camera_world, rvec, tvec
                                  │
                                  ▼
                          Đảo transform
                                  │
                                  ▼
                            T_world_camera
                                  │
                    ┌─────────────┼─────────────┐
                    ▼             ▼             ▼
               Publish TF   Save YAML result  Publish debug image
```

## 18. Tóm tắt ngắn

`aruco_extrinsic_calibrator_node.py` biến board ArUco / ChArUco thành một hệ world chuẩn. Node detect marker/corner trên ảnh, tạo cặp điểm 2D-3D, chạy `solvePnP`, đảo transform để lấy camera pose trong world, kiểm tra reprojection error rồi publish TF và lưu YAML nếu kết quả đạt chất lượng.

Khi dùng node này, hãy nhìn debug image trước: nếu marker/corner rõ, trục X/Y đúng hướng, status `ACCEPTED` và reprojection error nhỏ thì file `aruco_extrinsic_result.yaml` có thể dùng làm extrinsic camera-world cho các bước xử lý tọa độ tiếp theo.
