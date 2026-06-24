# Thư mục yolo

## 1. Mục đích

Thư mục `robot_vision_pipeline/yolo` chứa node YOLO detector chính của package. Nhiệm vụ của phần này là nhận ảnh màu từ camera RealSense, chạy model YOLO, tạo bbox cho các object như `wood` và `box`, rồi publish kết quả detection dạng JSON để các node phía sau xử lý tiếp.

Adapter chuyển YOLO JSON sang ROS detection message không nằm trong thư mục `yolo`, mà nằm ở cấp package:

```text
robot_vision_pipeline/yolo_json_to_object_detection_node.py
```

Adapter này vẫn được mô tả trong README này vì nó là bước ngay sau YOLO detector trong pipeline.

## 2. Cấu trúc file trong thư mục

| File | Chức năng | Loại file | Input | Output | Ghi chú |
| ---- | --------- | --------- | ----- | ------ | ------- |
| `__init__.py` | Đánh dấu `yolo` là Python package. | Package file | Không có | Không có | File rỗng. |
| `yolo_detect_node.py` | Node YOLO chính, đọc ảnh màu, chạy inference, publish JSON và ảnh debug. | Node chính | `/camera/camera/color/image_raw` | `/vision/yolo/detections_json`, `/vision/yolo/image_annotated`, `/vision/yolo/roi_debug_image` | Được khai báo trong `setup.py` với executable `yolo_detect_node`. |
| `yolo_detect_node_v1.py` | Phiên bản YOLO cũ hơn. | Legacy/test | Ảnh màu theo tham số | JSON/ảnh annotated theo tham số | Có script install trong `CMakeLists.txt`, nhưng không thấy launch chính gọi file này. |
| `yolo_utils.py` | Helper dự phòng. | Hỗ trợ | Không có | Không có | Hiện là file rỗng. |

## 3. Luồng xử lý trong phần YOLO

```text
/camera/camera/color/image_raw
        │
        ▼
┌─────────────────────────────────────────────┐
│  Layer 1 — YOLO Detector                    │
│  yolo_detect_node                           │
│                                             │
│  Inputs:  RGB image                         │
│           /camera/camera/color/image_raw    │
│                                             │
│  Outputs: /vision/yolo/detections_json      │
│           /vision/yolo/image_annotated      │
│           /vision/yolo/roi_debug_image      │
└──────────────┬──────────────────────────────┘
               │
               │ /vision/yolo/detections_json
               ▼
┌─────────────────────────────────────────────┐
│  Layer 2 — YOLO JSON Object Adapter         │
│  yolo_json_to_object_detection_node         │
│                                             │
│  Inputs:  /vision/yolo/detections_json      │
│           /camera/camera/aligned_depth_     │
│           to_color/image_raw                │
│                                             │
│  Outputs: /vision/wood_detection            │
│           /vision/box_detection             │
└─────────────────────────────────────────────┘
```

YOLO JSON cũng đi sang nhánh yaw cho `wood`:

```text
/vision/yolo/detections_json
/camera/camera/color/image_raw
        │
        ▼
yolo_hough_yaw_estimator_node
        ├── /vision/yolo/hough_yaw_json
        ├── /vision/debug_hough_yaw_image
        └── /vision/debug_hough_edges
```

## 4. Node `yolo_detect_node`

- File: `robot_vision_pipeline/yolo/yolo_detect_node.py`
- Class: `YoloDetectNode`
- ROS node name: `yolo_detect_node`
- Executable: `yolo_detect_node`
- Entry point trong `setup.py`: `yolo_detect_node = robot_vision_pipeline.yolo.yolo_detect_node:main`

Subscribe:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Ảnh màu đầu vào cho YOLO. |

Publish:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/yolo/detections_json` | `std_msgs/String` | Kết quả YOLO dạng JSON. |
| `/vision/yolo/image_annotated` | `sensor_msgs/Image` | Ảnh màu đã vẽ bbox YOLO và thông tin inference. |
| `/vision/yolo/roi_debug_image` | `sensor_msgs/Image` | Ảnh ROI được đưa vào YOLO khi bật `enable_roi_crop`. |

Cách xử lý chính:

1. Nhận frame BGR từ `sensor_msgs/Image`.
2. Nếu `enable_roi_crop=true`, cắt ROI theo `roi_x`, `roi_y`, `roi_width`, `roi_height`.
3. Chạy `ultralytics.YOLO(model_path).predict()` trên ROI hoặc full image.
4. Lọc class theo `class_filter` nếu có.
5. Đổi bbox từ tọa độ ROI về tọa độ ảnh gốc.
6. Publish JSON gồm `class_id`, `class_name`, `confidence`, `bbox_xyxy`, `bbox_xywh`.
7. Publish ảnh annotated và ROI debug nếu bật.

## 5. Model YOLO và tham số hiện tại

Theo `config/yolo_detect_real.yaml`:

| Tham số | Giá trị hiện tại | Ý nghĩa |
| ------- | ---------------- | ------- |
| `image_topic` | `/camera/camera/color/image_raw` | Topic ảnh màu đầu vào. |
| `model_path` | `/home/asus/ros_vision/src/robot_vision_pipeline/robot_vision_pipeline/models/yolov8.pt` | Đường dẫn model YOLO. |
| `device` | `cpu` | Thiết bị inference. |
| `conf_threshold` | `0.35` | Ngưỡng confidence YOLO. |
| `iou_threshold` | `0.45` | Ngưỡng IoU. |
| `imgsz` | `256` | Kích thước inference. |
| `max_det` | `20` | Số detection tối đa. |
| `class_filter` | `wood,box` | Chỉ giữ class `wood` và `box`. |
| `enable_roi_crop` | `true` | Bật crop ROI trước khi chạy YOLO. |
| `publish_roi_debug_image` | `true` | Publish `/vision/yolo/roi_debug_image`. |

Danh sách class thật được lấy từ `self.model.names` sau khi load model. YAML hiện lọc theo tên `wood,box`, nên nếu model không có đúng tên class này thì adapter phía sau sẽ không có output tương ứng.

Ghi chú: `model_path` ở bảng trên là giá trị đang ghi trong `config/yolo_detect_real.yaml`. Trong workspace hiện tại cũng có thư mục model ở `/home/asus/ros_vision/src/robot_vision_pipeline/models`. Nếu node báo lỗi không tìm thấy file `.pt`, kiểm tra đường dẫn thật hoặc chạy launch với `model_path:=/absolute/path/to/model.pt`.

## 6. Output JSON của YOLO

Mỗi payload trên `/vision/yolo/detections_json` là `std_msgs/String` chứa JSON. Các trường chính:

```text
stamp
frame_id
image_width
image_height
mode
inference_time_ms
num_detections
detections[]
roi_enabled
roi_x, roi_y, roi_width, roi_height
```

Mỗi detection trong `detections[]` có:

```text
id
class_id
class_name
confidence
bbox_xyxy: x1, y1, x2, y2
bbox_xywh: cx, cy, w, h
```

Bbox YOLO là vùng chữ nhật bao quanh object. Bbox này là input quan trọng cho adapter, mapper và yaw estimator.

## 7. Node `yolo_json_to_object_detection_node`

- File: `robot_vision_pipeline/yolo_json_to_object_detection_node.py`
- Class: `YoloJsonToObjectDetectionNode`
- ROS node name: `yolo_json_to_object_detection_node`
- Executable: `yolo_json_to_object_detection_node`
- Entry point trong `setup.py`: `yolo_json_to_object_detection_node = robot_vision_pipeline.yolo_json_to_object_detection_node:main`

Subscribe:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/yolo/detections_json` | `std_msgs/String` | YOLO JSON từ `yolo_detect_node`. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth aligned để lấy depth tại tâm bbox. |

Publish:

| Topic | Message | Ý nghĩa |
| ----- | ------- | ------- |
| `/vision/wood_detection` | `robot_vision_pipeline_msgs/BoxDetection` | Detection cho class `wood`. |
| `/vision/box_detection` | `robot_vision_pipeline_msgs/BoxDetection` | Detection cho class `box`. |

Adapter dùng tên `object` vì nó không chỉ phục vụ `box`. Nó chuyển YOLO JSON thành detection object tổng quát theo class, nhưng message hiện tại vẫn là `BoxDetection` để tương thích với package `robot_vision_pipeline_msgs`.

Các trường message được điền gồm:

```text
class_name
confidence
object_id
x_min, y_min, x_max, y_max
center_x, center_y
width_px, height_px
center_raw_depth
roi_median_raw_depth
depth_encoding
distance_m
```

## 8. Vì sao bbox YOLO ổn định hơn yaw

Bbox YOLO thường ổn định hơn yaw vì bbox chỉ cần model xác định vùng chứa vật thể. Nếu object vẫn nằm trong vùng nhìn tốt và model detect đúng class, bbox thường ít nhảy.

Yaw hiện tại không được YOLO dự đoán trực tiếp. Yaw được tính ở node `yolo_hough_yaw_estimator_node` bằng edge/line bên trong ROI. Vì vậy yaw nhạy hơn với:

- cạnh vật bị mờ hoặc thiếu;
- bóng, phản chiếu, nền đen;
- ROI lấy thừa cạnh nền hoặc cắt mất cạnh vật;
- nhiều line Hough cùng xuất hiện;
- vật gần vuông hoặc hai cạnh gần bằng nhau;
- không có smoothing nhiều frame trong yaw estimator hiện tại.

Nếu bbox ổn nhưng yaw/mũi tên yaw nhảy thì không nên chỉnh `conf_threshold` YOLO trước. Cần kiểm tra ROI, Canny edge, Hough line và các tham số yaw.

## 9. Danh sách topic liên quan

| Topic | Message | Publish bởi | Subscribe bởi | Ý nghĩa |
| ----- | ------- | ----------- | ------------- | ------- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense/static camera | `yolo_detect_node`, yaw estimator | Ảnh màu. |
| `/vision/yolo/detections_json` | `std_msgs/String` | `yolo_detect_node` | adapter, yaw estimator | Detection JSON. |
| `/vision/yolo/image_annotated` | `sensor_msgs/Image` | `yolo_detect_node` | Debug viewer | Ảnh có bbox YOLO. |
| `/vision/yolo/roi_debug_image` | `sensor_msgs/Image` | `yolo_detect_node` | Debug viewer | ROI YOLO. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense/static camera | adapter | Depth tại tâm bbox. |
| `/vision/wood_detection` | `robot_vision_pipeline_msgs/BoxDetection` | adapter | mapper | Detection class `wood`. |
| `/vision/box_detection` | `robot_vision_pipeline_msgs/BoxDetection` | adapter | mapper | Detection class `box`. |
| `/vision/yolo/hough_yaw_json` | `std_msgs/String` | yaw estimator | mapper | Yaw của `wood`. |
| `/vision/debug_hough_yaw_image` | `sensor_msgs/Image` | yaw estimator | Debug viewer | Debug line/yaw. |
| `/vision/debug_hough_edges` | `sensor_msgs/Image` | yaw estimator | Debug viewer | Debug Canny edge. |

## 10. Cách chạy

Build và source:

```bash
cd ~/ros_vision
colcon build --packages-select robot_vision_pipeline --symlink-install
source install/setup.bash
```

Chạy riêng YOLO detector:

```bash
ros2 run robot_vision_pipeline yolo_detect_node \
  --ros-args --params-file /home/asus/ros_vision/src/robot_vision_pipeline/config/yolo_detect_real.yaml
```

Chạy riêng adapter object khi đã có YOLO JSON và depth:

```bash
ros2 run robot_vision_pipeline yolo_json_to_object_detection_node \
  --ros-args --params-file /home/asus/ros_vision/src/robot_vision_pipeline/config/yolo_json_adapter.yaml
```

Chạy YOLO cùng RealSense:

```bash
ros2 launch robot_vision_pipeline yolo_detect_real.launch.py
```

Chạy full pipeline:

```bash
ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py
```

## 11. Cách debug

Kiểm tra executable:

```bash
ros2 pkg executables robot_vision_pipeline | grep yolo
```

Kiểm tra topic:

```bash
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /vision/yolo/detections_json
ros2 topic hz /vision/yolo/detections_json
ros2 topic echo /vision/wood_detection
ros2 topic echo /vision/box_detection
```

Xem ảnh:

```bash
ros2 run rqt_image_view rqt_image_view
```

Chọn lần lượt:

```text
/vision/yolo/image_annotated
/vision/yolo/roi_debug_image
/vision/debug_hough_yaw_image
/vision/debug_hough_edges
```

Lỗi thường gặp:

- Sai `model_path`: node lỗi khi load `ultralytics.YOLO`.
- Camera topic không có dữ liệu: kiểm tra `/camera/camera/color/image_raw`.
- Không thấy `/vision/yolo/detections_json`: kiểm tra YOLO node có chạy không và model có load được không.
- Không thấy `/vision/wood_detection` hoặc `/vision/box_detection`: kiểm tra JSON có class `wood`/`box` đúng tên không.
- Không tìm thấy executable: build và source lại workspace.
- Bbox ổn nhưng yaw nhảy: kiểm tra debug edge/Hough, không chỉnh confidence YOLO vội.

## 12. Ghi chú khi chỉnh tham số

Các tham số YOLO nên chỉnh khi bbox sai hoặc mất object:

- `model_path`
- `class_filter`
- `enable_roi_crop`
- `roi_x`, `roi_y`, `roi_width`, `roi_height`
- `imgsz`
- `max_det`

Các tham số yaw nên chỉnh khi bbox ổn nhưng yaw nhảy:

- `bbox_padding`
- `canny_low`, `canny_high`
- `hough_threshold`
- `min_line_length`
- `max_line_gap`

Không đổi `conf_threshold` chỉ để xử lý jitter yaw nếu bbox YOLO đã ổn định.
