# Thư mục pose_estimation

## 1. Mục đích

Thư mục `pose_estimation` phụ trách phần sau YOLO detection:

- nhận detection 2D từ adapter YOLO;
- nhận ảnh màu, ảnh depth aligned và `CameraInfo`;
- lấy tâm bbox làm điểm đại diện của object;
- lấy depth quanh tâm bbox;
- dùng intrinsics `fx`, `fy`, `cx`, `cy` để đổi pixel `(u, v, depth)` sang tọa độ 3D;
- publish object pose ra `/vision/wood_objects` và `/vision/box_objects`;
- tính yaw cho `wood` bằng node Hough yaw riêng và đưa yaw vào `Wood.pose.orientation` nếu hợp lệ.

Tên node mapper là `pixel_to_base_mapper_node`, nhưng code hiện tại publish trong frame `camera_color_optical_frame` và chưa transform sang `base_link` hoặc `world`.

## 2. Cấu trúc file trong thư mục

| File | Chức năng | Loại file | Input | Output | Ghi chú |
| ---- | --------- | --------- | ----- | ------ | ------- |
| `__init__.py` | Đánh dấu thư mục là Python package. | Package file | Không có | Không có | File rỗng. |
| `pixel_to_base_mapper_node.py` | Chuyển `BoxDetection` của `wood`/`box` sang object 3D. | Node chính | detection, RGB, depth, `CameraInfo`, yaw JSON | `/vision/wood_objects`, `/vision/box_objects`, `/vision/debug_image_camera` | Được khai báo trong `setup.py`. |
| `yolo_hough_yaw_estimator_node.py` | Tính yaw cho `wood` từ bbox YOLO và ảnh màu. | Node chính | `/vision/yolo/detections_json`, `/camera/camera/color/image_raw` | `/vision/yolo/hough_yaw_json`, `/vision/debug_hough_yaw_image`, `/vision/debug_hough_edges` | Được khai báo trong `setup.py`. |
| `pick_pose_estimator_node.py` | Chưa có nội dung code. | Placeholder | Chưa có | Chưa có | Không thấy dùng trong launch/setup. |
| `target_filter_node.py` | Chưa có nội dung code. | Placeholder | Chưa có | Chưa có | Không thấy dùng trong launch/setup. |
| `tf_utils.py` | Chưa có nội dung code. | Placeholder/helper | Chưa có | Chưa có | Hiện chưa có hàm transform dùng trong mapper. |

## 3. Luồng xử lý trong pose_estimation

```text
/vision/wood_detection
/vision/box_detection
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
/vision/yolo/hough_yaw_json
        │
        ▼
┌─────────────────────────────────────────────┐
│  Layer 3 — Pixel/Depth Mapper               │
│  pixel_to_base_mapper_node                  │
│                                             │
│  Inputs:  wood/box detections               │
│           RGB image                         │
│           aligned depth image               │
│           CameraInfo intrinsics             │
│           yaw JSON cho wood                 │
│                                             │
│  Outputs: /vision/wood_objects              │
│           /vision/box_objects               │
│           /vision/debug_image_camera        │
│                                             │
│  Frame:   camera_color_optical_frame        │
└─────────────────────────────────────────────┘
```

Nhánh yaw riêng:

```text
/vision/yolo/detections_json
/camera/camera/color/image_raw
        │
        ▼
┌─────────────────────────────────────────────┐
│  Layer 2B — Wood Yaw Estimator              │
│  yolo_hough_yaw_estimator_node              │
│                                             │
│  Inputs:  YOLO JSON + RGB image             │
│  Outputs: /vision/yolo/hough_yaw_json       │
│  Debug:   /vision/debug_hough_yaw_image     │
│           /vision/debug_hough_edges         │
└─────────────────────────────────────────────┘
```

## 4. Node `pixel_to_base_mapper_node`

- File: `robot_vision_pipeline/pose_estimation/pixel_to_base_mapper_node.py`
- Class: `PixelToBaseMapperNode`
- ROS node name: `pixel_to_base_mapper_node`
- Executable: `pixel_to_base_mapper_node`
- Entry point trong `setup.py`: `pixel_to_base_mapper_node = robot_vision_pipeline.pose_estimation.pixel_to_base_mapper_node:main`

Subscribe:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/wood_detection` | `robot_vision_pipeline_msgs/BoxDetection` | Detection của class `wood`. |
| `/vision/box_detection` | `robot_vision_pipeline_msgs/BoxDetection` | Detection của class `box`. |
| `/vision/yolo/hough_yaw_json` | `std_msgs/String` | Yaw JSON cho `wood`. |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Ảnh màu để vẽ debug. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth aligned để lấy Z. |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics camera. |

Publish:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/wood_objects` | `robot_vision_pipeline_msgs/WoodArray` | Danh sách wood có pose 3D. |
| `/vision/box_objects` | `robot_vision_pipeline_msgs/BoxArray` | Danh sách box có pose 3D và size. |
| `/vision/debug_image_camera` | `sensor_msgs/Image` | Ảnh debug bbox và tâm object. Wood vẫn hiển thị tọa độ 3D và yaw nếu có; box hiển thị chiều dài và chiều rộng ước lượng từ bbox + depth, không hiển thị tọa độ trên ảnh debug. |

Tham số quan trọng:

| Tham số | Giá trị mặc định trong code/YAML | Ý nghĩa |
| ------- | ------------------------------- | ------- |
| `output_frame_id` | `camera_color_optical_frame` | Frame của object output. |
| `depth_kernel_radius` | `2` | Bán kính vùng lấy depth quanh tâm bbox. |
| `min_depth_m`, `max_depth_m` | `0.05`, `3.0` | Khoảng depth hợp lệ. |
| `depth_outlier_threshold_m` | `0.02` | Ngưỡng bỏ outlier depth quanh median. |
| `min_valid_depth_samples` | `3` | Số mẫu depth hợp lệ tối thiểu. |
| `default_box_size_x_m/y_m/z_m` | `0.08`, `0.08`, `0.05` | Size fallback cho box. |
| `box_obstacle_margin_m` | `0.02` | Margin giữ lại cho debug/tính safe area nội bộ; không cộng vào `Box.size` khi publish `/vision/box_objects`. |
| `use_hough_yaw_for_wood` | `true` | Dùng yaw JSON cho wood. |
| `yaw_match_max_center_dist_px` | `40.0` hoặc override trong launch | Khoảng cách tâm tối đa để match yaw với detection. |
| `yaw_stale_timeout_sec` | `0.5` hoặc override trong launch | Timeout yaw cũ. |
| `draw_wood_yaw_arrow` | `true` | Vẽ mũi tên yaw trên debug image. |

## 5. Cách tính tọa độ 3D

Mapper xử lý `wood` và `box` theo cùng nguyên lý tọa độ:

1. Nhận `BoxDetection` từ adapter.
2. Lấy tâm bbox:

```text
u = center_x
v = center_y
```

3. Lấy depth `Z` quanh tâm bbox từ `/camera/camera/aligned_depth_to_color/image_raw`.
4. Hàm `robust_center_depth()` lấy một cửa sổ nhỏ quanh tâm, bỏ depth invalid/zero/out-of-range, lấy median và bỏ outlier.
5. Nếu depth từ ảnh không hợp lệ, mapper fallback sang `distance_m`, `roi_median_raw_depth`, hoặc `center_raw_depth` trong `BoxDetection`.
6. Lấy intrinsics từ `CameraInfo.k`:

```text
fx = k[0]
fy = k[4]
cx = k[2]
cy = k[5]
```

7. Chuyển pixel sang tọa độ camera:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

8. Ghi vào `pose.position` của `Wood` hoặc `Box`.

Code hiện tại không dùng homography và không đọc extrinsic/base transform trong mapper. Output frame là:

```text
camera_color_optical_frame
```

Nếu downstream cần `base_link` hoặc `world`, cần thêm transform ở node khác hoặc mở rộng mapper trong code sau này. README này chỉ mô tả hiện trạng.

## 6. Xử lý `wood`

Input:

```text
/vision/wood_detection
/vision/yolo/hough_yaw_json
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

Output:

```text
/vision/wood_objects
```

Message:

```text
robot_vision_pipeline_msgs/WoodArray
  header
  woods[]
    header
    wood_id
    class_name
    confidence
    pose
```

Với `wood`:

- `pose.position` là XYZ từ pixel/depth/intrinsics.
- `pose.orientation` là quaternion quanh trục Z nếu có yaw hợp lệ.
- Nếu không match được yaw, orientation là identity `(0, 0, 0, 1)`.

## 7. Xử lý `box` / vật cản

Input:

```text
/vision/box_detection
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

Output:

```text
/vision/box_objects
```

Message:

```text
robot_vision_pipeline_msgs/BoxArray
  header
  boxes[]
    header
    box_id
    class_name
    confidence
    pose
    size
```

Với `box`:

- `pose.position` là XYZ từ pixel/depth/intrinsics.
- `pose.orientation` luôn identity `(0, 0, 0, 1)` trong code hiện tại.
- `size.x` và `size.y` được ước lượng trực tiếp từ bbox YOLO, depth và intrinsics camera:

```text
size_x = bbox_width_px  * depth / fx
size_y = bbox_height_px * depth / fy
size_z = default_box_size_z_m
```

- `box_obstacle_margin_m` có thể được dùng để tính safe area cho debug nội bộ:

```text
safe_width  = size_x + 2 * box_obstacle_margin_m
safe_length = size_y + 2 * box_obstacle_margin_m
```

Margin an toàn không được cộng vào dữ liệu publish ra `/vision/box_objects`. `Box.size` luôn là kích thước bbox detect được:

```text
box_msg.size.x = size_x
box_msg.size.y = size_y
box_msg.size.z = default_box_size_z_m
```

`/vision/box_objects` có thể dùng làm object/vật cản cho tầng điều khiển hoặc RL.

## 8. Node `yolo_hough_yaw_estimator_node`

- File: `robot_vision_pipeline/pose_estimation/yolo_hough_yaw_estimator_node.py`
- Class: `YoloHoughYawEstimatorNode`
- ROS node name: `yolo_hough_yaw_estimator_node`
- Executable: `yolo_hough_yaw_estimator_node`
- Entry point trong `setup.py`: `yolo_hough_yaw_estimator_node = robot_vision_pipeline.pose_estimation.yolo_hough_yaw_estimator_node:main`

Subscribe:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/yolo/detections_json` | `std_msgs/String` | Bbox YOLO của các object. |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Ảnh màu dùng để cắt ROI và tìm cạnh. |

Publish:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/yolo/hough_yaw_json` | `std_msgs/String` | Yaw JSON cho `wood`. |
| `/vision/debug_hough_yaw_image` | `sensor_msgs/Image` | Ảnh debug có bbox, line, mũi tên yaw. |
| `/vision/debug_hough_edges` | `sensor_msgs/Image` | Ảnh Canny edge. |

Tham số quan trọng:

| Tham số | Mặc định | Ý nghĩa |
| ------- | -------- | ------- |
| `target_class` | `wood` | Chỉ tính yaw cho class này. |
| `bbox_padding` | `25` | Padding quanh bbox khi crop ROI. |
| `canny_low` | `30` | Ngưỡng thấp Canny. |
| `canny_high` | `100` | Ngưỡng cao Canny. |
| `hough_threshold` | `10` | Ngưỡng vote HoughLinesP. |
| `min_line_length` | `10` | Độ dài line tối thiểu. |
| `max_line_gap` | `8` | Khoảng nối line tối đa. |
| `arrow_length` | `60` | Độ dài mũi tên debug. |
| `publish_debug_image` | `true` | Publish ảnh debug. |

## 9. Luồng tính yaw chi tiết

Yaw là góc hướng của vật gỗ trên ảnh hoặc mặt phẳng làm việc. Đây không phải góc xoay TCP của robot.

Theo code hiện tại, yaw được tính như sau:

1. Đọc YOLO JSON từ `/vision/yolo/detections_json`.
2. Lọc detection có `class_name == target_class`, mặc định là `wood`.
3. Lấy bbox từ `bbox_xyxy` hoặc `bbox`.
4. Clamp bbox vào kích thước ảnh.
5. Mở rộng bbox bằng `bbox_padding`.
6. Crop ROI từ ảnh màu.
7. Chuyển ROI sang grayscale:

```text
cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
```

8. Làm mờ bằng Gaussian blur:

```text
cv2.GaussianBlur(gray, (5, 5), 0)
```

9. Tìm cạnh bằng Canny:

```text
cv2.Canny(blur, canny_low, canny_high)
```

10. Tìm line bằng HoughLinesP:

```text
cv2.HoughLinesP(
    edges,
    rho=1,
    theta=np.pi / 180,
    threshold=hough_threshold,
    minLineLength=min_line_length,
    maxLineGap=max_line_gap,
)
```

11. Chọn line dài nhất trong các line tìm được.
12. Tính góc raw:

```text
yaw_raw_deg = atan2(y2 - y1, x2 - x1)
```

13. Chuẩn hóa yaw bằng `normalize_yaw_0_90()`:

```text
angle = angle % 180
if angle > 90:
    angle = 180 - angle
```

Kết quả yaw nằm trong khoảng `[0°, 90°]`. Giá trị này mô tả độ nghiêng của cạnh/line được chọn, không giữ dấu âm/dương.

14. Publish JSON ra `/vision/yolo/hough_yaw_json`.
15. Mapper đọc JSON, match yaw với `wood_detection` theo tâm bbox.
16. Nếu yaw còn mới và tâm đủ gần, mapper đổi yaw sang quaternion:

```text
qx = 0
qy = 0
qz = sin(yaw_rad / 2)
qw = cos(yaw_rad / 2)
```

17. Gán quaternion vào `Wood.pose.orientation`.

Code yaw hiện tại không dùng HSV mask, Otsu threshold, contour, `minAreaRect`, PCA hoặc smoothing nhiều frame. Mapper có smoothing nhẹ cho tâm vẽ debug trong trường hợp tự vẽ arrow từ yaw, nhưng khi `use_yaw_arrow_from_hough_json=true`, mũi tên debug ưu tiên dùng line/arrow từ Hough JSON.

## 10. Vì sao yaw bị nhảy/jitter

Các nguyên nhân dễ thấy từ thuật toán hiện tại:

- Bbox YOLO ổn định nhưng edge trong ROI thay đổi giữa các frame.
- Canny rất nhạy với ánh sáng, bóng, phản chiếu, nền đen và cạnh bị mờ.
- ROI quá sát bbox có thể cắt mất cạnh thật của gỗ.
- ROI quá rộng có thể lấy thêm cạnh nền, cạnh bàn hoặc object khác.
- HoughLinesP có thể trả nhiều line; code chọn line dài nhất, nhưng line dài nhất chưa chắc là cạnh đúng của vật.
- Vật gỗ gần vuông hoặc có hai cạnh gần bằng nhau có thể làm line đại diện đổi qua lại, gây nhảy gần 90 độ.
- Khi cạnh gần ngang/dọc hoặc góc gần 45 độ, sai số nhỏ ở edge có thể làm mũi tên nhìn rung.
- Code hiện tại chưa có smoothing nhiều frame, chưa giữ yaw cũ khi line yếu, chưa giới hạn max yaw jump per frame.
- Nếu không có line Hough, yaw JSON không có yaw hợp lệ; mapper sẽ dùng orientation identity.
- Depth không trực tiếp quyết định yaw 2D, nhưng depth/tâm pose rung có thể làm ảnh debug và điểm vẽ nhìn không ổn định.

Nếu bbox YOLO đúng mà yaw nhảy, lỗi chủ yếu nằm ở xử lý ROI/edge/Hough trong yaw estimator, không phải do confidence YOLO.

## 11. Debug visualization

Ảnh debug của yaw estimator:

```text
/vision/debug_hough_yaw_image
```

Nội dung vẽ:

- bbox YOLO của `wood`;
- tâm bbox;
- line Hough được chọn;
- mũi tên yaw;
- text `wood yaw=<deg>`.

Ảnh edge:

```text
/vision/debug_hough_edges
```

Nội dung:

- edge sau Canny trong ROI đã padding;
- vùng ngoài ROI là ảnh đen.

Ảnh debug của mapper:

```text
/vision/debug_image_camera
```

Nội dung vẽ:

- ROI YOLO nếu bật `draw_yolo_roi`;
- bbox `wood` và `box`;
- tâm object;
- yaw text và yaw arrow cho `wood` nếu có yaw hợp lệ;
- Trên `/vision/debug_image_camera`, wood vẫn hiển thị tọa độ 3D và yaw nếu có; box hiển thị chiều dài và chiều rộng ước lượng từ bbox + depth, không hiển thị tọa độ trên ảnh debug.

## 12. Topic liên quan

| Topic | Message | Publish bởi | Subscribe bởi | Ý nghĩa |
| ----- | ------- | ----------- | ------------- | ------- |
| `/vision/wood_detection` | `robot_vision_pipeline_msgs/BoxDetection` | object adapter | mapper | Bbox/depth detection của `wood`. |
| `/vision/box_detection` | `robot_vision_pipeline_msgs/BoxDetection` | object adapter | mapper | Bbox/depth detection của `box`. |
| `/vision/yolo/detections_json` | `std_msgs/String` | YOLO detector | yaw estimator | YOLO JSON để lấy bbox wood. |
| `/vision/yolo/hough_yaw_json` | `std_msgs/String` | yaw estimator | mapper | Yaw Hough cho wood. |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | camera | yaw estimator, mapper | Ảnh màu. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | camera | mapper | Depth aligned. |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | camera | mapper | Intrinsics. |
| `/vision/wood_objects` | `robot_vision_pipeline_msgs/WoodArray` | mapper | downstream | Pose 3D wood. |
| `/vision/box_objects` | `robot_vision_pipeline_msgs/BoxArray` | mapper | downstream | Pose 3D và size box. |
| `/vision/debug_image_camera` | `sensor_msgs/Image` | mapper | debug viewer | Ảnh debug mapper. |
| `/vision/debug_hough_yaw_image` | `sensor_msgs/Image` | yaw estimator | debug viewer | Ảnh debug yaw. |
| `/vision/debug_hough_edges` | `sensor_msgs/Image` | yaw estimator | debug viewer | Ảnh Canny edge. |

## 13. Cách chạy riêng phần pose_estimation

Build và source:

```bash
cd ~/ros_vision
colcon build --packages-select robot_vision_pipeline --symlink-install
source install/setup.bash
```

Chạy mapper riêng khi các topic input đã có:

```bash
ros2 run robot_vision_pipeline pixel_to_base_mapper_node \
  --ros-args --params-file /home/asus/ros_vision/src/robot_vision_pipeline/config/pixel_to_base_mapper.yaml
```

Chạy yaw estimator riêng:

```bash
ros2 launch robot_vision_pipeline yolo_hough_yaw_estimator.launch.py
```

Chạy full pipeline:

```bash
ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py
```

## 14. Cách debug

Kiểm tra input trước:

```bash
ros2 topic list
ros2 topic list | grep camera_info
ros2 topic echo /camera/camera/color/camera_info
ros2 topic echo /vision/wood_detection
ros2 topic echo /vision/box_detection
```

Kiểm tra output:

```bash
ros2 topic echo /vision/yolo/hough_yaw_json
ros2 topic echo /vision/wood_objects
ros2 topic echo /vision/box_objects
```

Xem ảnh:

```bash
ros2 run rqt_image_view rqt_image_view
```

Chọn:

```text
/vision/debug_hough_yaw_image
/vision/debug_hough_edges
/vision/debug_image_camera
```

Thứ tự debug yaw nên làm:

1. Kiểm tra `/vision/yolo/detections_json` có bbox `wood` đúng không.
2. Xem `/vision/debug_hough_edges` để biết Canny có bắt đúng cạnh gỗ không.
3. Xem `/vision/debug_hough_yaw_image` để xem line Hough được chọn có đúng cạnh đại diện không.
4. Echo `/vision/yolo/hough_yaw_json` để xem `yaw_deg` có nhảy không.
5. Echo `/vision/wood_objects` để xem quaternion orientation có đổi theo yaw không.

Nếu bbox ổn nhưng yaw nhảy, không chỉnh confidence YOLO trước. Hãy kiểm tra ROI padding, Canny và Hough.

## 15. Lỗi thường gặp

- Không có `/camera/camera/color/camera_info`: mapper sẽ bỏ qua object vì thiếu intrinsics.
- Không có `/camera/camera/aligned_depth_to_color/image_raw`: mapper không lấy được depth từ ảnh.
- Depth invalid hoặc bằng 0: mapper skip object hoặc fallback sang depth trong detection nếu có.
- Có `/vision/wood_detection` nhưng không có `/vision/wood_objects`: kiểm tra `CameraInfo`, depth và log mapper.
- Có YOLO bbox nhưng không có yaw: kiểm tra `/vision/yolo/hough_yaw_json` và ảnh `/vision/debug_hough_edges`.
- Có yaw JSON nhưng orientation vẫn identity: yaw có thể quá cũ hoặc tâm yaw không match detection theo `yaw_match_max_center_dist_px`.
- Không thấy debug image: kiểm tra mapper/yaw node có đang chạy và topic ảnh màu có dữ liệu không.
- Chưa source workspace sau build: executable hoặc config mới không được ROS 2 tìm thấy.

## 16. Ghi chú khi chỉnh tham số

Nhóm tham số ảnh hưởng depth/pose:

- `depth_kernel_radius`
- `min_depth_m`, `max_depth_m`
- `depth_outlier_threshold_m`
- `min_valid_depth_samples`
- `box_obstacle_margin_m`

Nhóm tham số ảnh hưởng yaw:

- `bbox_padding`
- `canny_low`, `canny_high`
- `hough_threshold`
- `min_line_length`
- `max_line_gap`
- `yaw_match_max_center_dist_px`
- `yaw_stale_timeout_sec`

Nếu cần giảm yaw jitter trong code sau này, nên cân nhắc smoothing nhiều frame, giữ yaw cũ khi line hiện tại yếu, lọc line theo hướng/khoảng cách so với bbox, hoặc giới hạn max yaw jump per frame. README này không thay đổi thuật toán hiện tại.
