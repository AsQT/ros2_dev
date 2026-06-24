# robot_vision_pipeline

## Mục đích

`robot_vision_pipeline` là package ROS 2 dùng cho luồng vision RGB-D của robot. Luồng chính hiện tại nhận ảnh từ RealSense, chạy YOLO để phát hiện `wood` và `box`, chuyển kết quả YOLO JSON sang detection message, sau đó dùng depth aligned và `CameraInfo` để tạo object pose 3D.

Điểm quan trọng theo code hiện tại:

- Input camera chính là `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info`.
- YOLO chỉ nhận ảnh màu và publish bbox/detection dạng JSON.
- Adapter chính là `yolo_json_to_object_detection_node`, không phải node tên cố định theo `box`.
- Mapper publish pose trong frame `camera_color_optical_frame`.
- Code mapper hiện chưa transform object sang `base_link` hoặc `world`.
- Mapper không dùng homography.
- Yaw của `wood` được lấy từ node Hough yaw riêng và ghi vào `Wood.pose.orientation` nếu match được.
- `box` được publish với position + size, orientation identity.

## Cấu trúc thư mục chính

```text
robot_vision_pipeline/
├── config/
│   ├── rs_camera.yaml
│   ├── rs_camera_yolo.yaml
│   ├── yolo_detect_real.yaml
│   ├── yolo_json_adapter.yaml
│   └── pixel_to_base_mapper.yaml
├── launch/
│   ├── vision_full_pipeline.launch.py
│   ├── vision_image_test.launch.py
│   ├── yolo_detect_real.launch.py
│   └── yolo_hough_yaw_estimator.launch.py
├── scripts/
│   ├── yolo_detect_node
│   ├── yolo_json_to_object_detection_node
│   ├── pixel_to_base_mapper_node
│   └── yolo_hough_yaw_estimator_node
├── robot_vision_pipeline/
│   ├── yolo/
│   │   ├── yolo_detect_node.py
│   │   ├── yolo_detect_node_v1.py
│   │   └── yolo_utils.py
│   ├── pose_estimation/
│   │   ├── pixel_to_base_mapper_node.py
│   │   └── yolo_hough_yaw_estimator_node.py
│   ├── yolo_json_to_object_detection_node.py
│   ├── static_image_camera_node.py
│   ├── realsense_depth_debug_node.py
│   ├── depth_utils.py
│   ├── aruco/
│   └── calib/
├── setup.py
├── CMakeLists.txt
└── package.xml
```

`aruco/` và `calib/` chứa các node calibration/kiểm tra riêng. Chúng không nằm trong luồng YOLO RGB-D chính mô tả bên dưới.

## Luồng xử lý chính

Launch chính:

```bash
ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py
```

Sơ đồ theo node/topic thật trong code:

```text
RealSense D435 / RGB-D camera
    │
    ├── /camera/camera/color/image_raw ───────────────────────────────┐
    ├── /camera/camera/aligned_depth_to_color/image_raw ───────────┐  │
    └── /camera/camera/color/camera_info ───────────────────────┐  │  │
                                                                 │  │  │
                                                                 │  │  ▼
                           ┌─────────────────────────────────────┐ │  │
                           │  Layer 1 — YOLO Detector            │ │  │
                           │  yolo_detect_node                   │ │  │
                           │                                     │ │  │
                           │  Inputs:  RGB image                 │ │  │
                           │           /camera/camera/color/     │ │  │
                           │           image_raw                 │ │  │
                           │                                     │ │  │
                           │  Outputs: /vision/yolo/             │ │  │
                           │           detections_json           │ │  │
                           │           /vision/yolo/             │ │  │
                           │           image_annotated           │ │  │
                           │           /vision/yolo/             │ │  │
                           │           roi_debug_image           │ │  │
                           └──────────────┬──────────────────────┘ │  │
                                          │                        │  │
                                          │ /vision/yolo/detections_json
                                          │                        │  │
                    ┌─────────────────────┴─────────────────────┐  │  │
                    ▼                                           ▼  │  │
┌─────────────────────────────────────┐   ┌─────────────────────────────────────┐
│ Layer 2A — YOLO JSON Object Adapter  │   │ Layer 2B — Wood Yaw Estimator       │
│ yolo_json_to_object_detection_node   │   │ yolo_hough_yaw_estimator_node       │
│                                     │   │                                     │
│ Inputs:  YOLO JSON                  │   │ Inputs:  YOLO JSON                  │
│          aligned depth              │   │          RGB image                  │
│                                     │   │                                     │
│ Outputs: /vision/wood_detection     │   │ Outputs: /vision/yolo/              │
│          /vision/box_detection      │   │          hough_yaw_json             │
│                                     │   │ Debug:  /vision/debug_hough_        │
│                                     │   │         yaw_image                   │
│                                     │   │         /vision/debug_hough_edges   │
└──────────────┬──────────────────────┘   └──────────────┬──────────────────────┘
               │                                         │
               │ /vision/wood_detection                  │ /vision/yolo/hough_yaw_json
               │ /vision/box_detection                   │
               └─────────────────────┬───────────────────┘
                                     │
                                     ▼
                           ┌─────────────────────────────────────┐
                           │  Layer 3 — Pixel/Depth Mapper       │
                           │  pixel_to_base_mapper_node          │
                           │                                     │
                           │  Inputs:  wood/box detections       │
                           │           RGB image                 │
                           │           aligned depth             │
                           │           CameraInfo intrinsics     │
                           │           wood yaw JSON             │
                           │                                     │
                           │  Outputs: /vision/wood_objects      │
                           │           /vision/box_objects       │
                           │           /vision/debug_image_      │
                           │           camera                    │
                           │                                     │
                           │  Frame:   camera_color_optical_frame│
                           └─────────────────────────────────────┘
```

Luồng dữ liệu ngắn gọn:

```text
RGB image
→ YOLO detector
→ bbox wood/box JSON
→ object adapter
→ /vision/wood_detection + /vision/box_detection
→ pixel/depth mapper
→ depth + fx/fy/cx/cy
→ XYZ trong camera_color_optical_frame
→ yaw quaternion cho wood nếu có
→ /vision/wood_objects + /vision/box_objects
```

## Node chính

| Node | File | Subscribe | Publish | Vai trò |
| ---- | ---- | --------- | ------- | ------- |
| `yolo_detect_node` | `robot_vision_pipeline/yolo/yolo_detect_node.py` | `/camera/camera/color/image_raw` | `/vision/yolo/detections_json`, `/vision/yolo/image_annotated`, `/vision/yolo/roi_debug_image` | Chạy YOLO trên ảnh màu, tạo bbox và detection JSON. |
| `yolo_json_to_object_detection_node` | `robot_vision_pipeline/yolo_json_to_object_detection_node.py` | `/vision/yolo/detections_json`, `/camera/camera/aligned_depth_to_color/image_raw` | `/vision/wood_detection`, `/vision/box_detection` | Chuyển YOLO JSON sang `BoxDetection` cho `wood` và `box`, kèm depth tại tâm bbox nếu có. |
| `yolo_hough_yaw_estimator_node` | `robot_vision_pipeline/pose_estimation/yolo_hough_yaw_estimator_node.py` | `/vision/yolo/detections_json`, `/camera/camera/color/image_raw` | `/vision/yolo/hough_yaw_json`, `/vision/debug_hough_yaw_image`, `/vision/debug_hough_edges` | Tính yaw cho `wood` bằng ROI từ bbox, Canny và HoughLinesP. |
| `pixel_to_base_mapper_node` | `robot_vision_pipeline/pose_estimation/pixel_to_base_mapper_node.py` | `/vision/wood_detection`, `/vision/box_detection`, `/vision/yolo/hough_yaw_json`, RGB, depth, `CameraInfo` | `/vision/wood_objects`, `/vision/box_objects`, `/vision/debug_image_camera` | Tính XYZ từ pixel + depth + intrinsics, gán yaw quaternion cho `wood`. |
| `static_image_camera_node` | `robot_vision_pipeline/static_image_camera_node.py` | Không có | `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info` | Camera giả cho test bằng ảnh tĩnh. |
| `realsense_depth_debug` | `robot_vision_pipeline/realsense_depth_debug_node.py` | RGB, depth, `CameraInfo` | Log console | Kiểm tra nhanh RealSense RGB/depth/info. |

## Topic chính

| Topic | Message | Publish bởi | Subscribe bởi | Ý nghĩa |
| ----- | ------- | ----------- | ------------- | ------- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RealSense hoặc `static_image_camera_node` | YOLO, yaw estimator, mapper debug | Ảnh màu đầu vào. |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense hoặc `static_image_camera_node` | adapter, mapper | Depth đã align theo ảnh màu. |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense hoặc `static_image_camera_node` | mapper | Intrinsics `fx`, `fy`, `cx`, `cy`. |
| `/vision/yolo/detections_json` | `std_msgs/String` | `yolo_detect_node` | object adapter, yaw estimator | Kết quả YOLO gồm class, confidence, bbox. |
| `/vision/yolo/image_annotated` | `sensor_msgs/Image` | `yolo_detect_node` | Debug viewer | Ảnh đã vẽ bbox YOLO. |
| `/vision/yolo/roi_debug_image` | `sensor_msgs/Image` | `yolo_detect_node` | Debug viewer | Ảnh ROI đưa vào YOLO khi bật crop. |
| `/vision/wood_detection` | `robot_vision_pipeline_msgs/BoxDetection` | `yolo_json_to_object_detection_node` | mapper | Detection `wood` dạng ROS message. |
| `/vision/box_detection` | `robot_vision_pipeline_msgs/BoxDetection` | `yolo_json_to_object_detection_node` | mapper | Detection `box` dạng ROS message. |
| `/vision/yolo/hough_yaw_json` | `std_msgs/String` | `yolo_hough_yaw_estimator_node` | mapper | Yaw cho `wood`, tính từ cạnh trong ROI. |
| `/vision/debug_hough_yaw_image` | `sensor_msgs/Image` | `yolo_hough_yaw_estimator_node` | Debug viewer | Ảnh màu có bbox, line và mũi tên yaw. |
| `/vision/debug_hough_edges` | `sensor_msgs/Image` | `yolo_hough_yaw_estimator_node` | Debug viewer | Ảnh edge sau Canny trong ROI. |
| `/vision/wood_objects` | `robot_vision_pipeline_msgs/WoodArray` | mapper | Downstream robot/RL/control | Wood pose 3D, orientation có yaw nếu hợp lệ. |
| `/vision/box_objects` | `robot_vision_pipeline_msgs/BoxArray` | mapper | Downstream robot/RL/control | Box pose 3D; `size.x` và `size.y` là kích thước ước lượng trực tiếp từ bounding box YOLO, depth và intrinsics camera, không cộng margin an toàn khi publish. |
| `/vision/debug_image_camera` | `sensor_msgs/Image` | mapper | Debug viewer | Ảnh debug bbox và tâm object. Wood vẫn hiển thị tọa độ 3D và yaw nếu có; box hiển thị chiều dài và chiều rộng ước lượng từ bbox + depth, không hiển thị tọa độ trên ảnh debug. |

## Coordinate Conversion

Mapper dùng tâm bbox `(u, v)`, depth `Z` và intrinsics từ `CameraInfo`:

```text
fx = CameraInfo.k[0]
fy = CameraInfo.k[4]
cx = CameraInfo.k[2]
cy = CameraInfo.k[5]

X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

Depth được lấy ưu tiên từ ảnh `/camera/camera/aligned_depth_to_color/image_raw` bằng hàm `robust_center_depth()` trong `depth_utils.py`. Hàm này lấy vùng nhỏ quanh tâm bbox, bỏ depth invalid/zero/outlier rồi lấy giá trị ổn định hơn. Nếu không lấy được depth từ ảnh, mapper fallback sang các trường depth đã có trong `BoxDetection`.

Output frame hiện tại là:

```text
camera_color_optical_frame
```

Tên node là `pixel_to_base_mapper_node`, nhưng code hiện tại chưa có bước transform camera sang `base_link` hoặc `world`. File `config/Extrinsic_camera_to_world.yaml` tồn tại trong package để phục vụ calibration/luồng khác, nhưng mapper chính không đọc file này.

## Yaw Estimation

Yaw trong package này là góc hướng của vật `wood` trên ảnh/mặt phẳng làm việc, không phải góc xoay TCP của robot.

Luồng yaw hiện tại:

```text
YOLO bbox của wood
→ crop ROI quanh bbox, có padding
→ grayscale
→ GaussianBlur
→ Canny edge
→ HoughLinesP
→ chọn line dài nhất
→ yaw_raw = atan2(dy, dx)
→ normalize_yaw_0_90()
→ publish /vision/yolo/hough_yaw_json
→ mapper match yaw theo tâm bbox
→ yaw_to_quaternion_z()
→ Wood.pose.orientation
```

Code hiện tại của `yolo_hough_yaw_estimator_node.py` không dùng `minAreaRect`, PCA, HSV mask hoặc smoothing nhiều frame. Mapper chỉ kiểm tra yaw còn mới (`yaw_stale_timeout_sec`) và match theo khoảng cách tâm bbox (`yaw_match_max_center_dist_px`). Nếu không match được yaw hợp lệ, `Wood.pose.orientation` là identity quaternion `(0, 0, 0, 1)`.

Vì sao bbox YOLO có thể ổn định nhưng yaw vẫn nhảy:

- Bbox YOLO phụ thuộc vào model detect vùng vật thể, thường ổn định hơn cạnh nhỏ bên trong ROI.
- Yaw Hough phụ thuộc vào edge/line sau Canny, nên rất nhạy với ánh sáng, bóng, phản chiếu, nền đen hoặc cạnh bị mờ.
- Nếu ROI quá sát bbox thì mất cạnh thật; nếu ROI quá rộng thì dễ lấy cạnh nền hoặc vật khác.
- HoughLinesP chọn line dài nhất, nhưng line dài nhất có thể là cạnh bóng, cạnh nhiễu hoặc cạnh không đại diện cho hướng gỗ.
- Vật gần vuông hoặc có hai cạnh dài gần nhau có thể làm thuật toán đổi qua lại giữa hai cạnh, gây nhảy góc.
- Khi vật gần nằm ngang/dọc hoặc gần 45 độ, sai số nhỏ của edge có thể làm mũi tên yaw nhìn rung rõ hơn.
- Code yaw hiện tại không có smoothing nhiều frame và không có giới hạn max yaw jump per frame, nên mỗi frame có thể chọn line độc lập.
- Depth không trực tiếp quyết định yaw 2D, nhưng depth noise hoặc lệch RGB-depth có thể làm pose/tâm/debug nhìn rung.

Không nên sửa `conf_threshold` YOLO chỉ vì yaw nhảy. Nếu bbox ổn nhưng yaw nhảy thì vấn đề nằm ở xử lý edge/line trong yaw estimator hoặc vùng ROI/debug, không phải confidence YOLO.

## Launch Files

| Launch file | Mục đích |
| ----------- | -------- |
| `vision_full_pipeline.launch.py` | Launch chính với RealSense, YOLO, object adapter, yaw estimator và mapper. |
| `vision_image_test.launch.py` | Test pipeline bằng `static_image_camera_node` và fake depth. |
| `yolo_detect_real.launch.py` | Chạy RealSense color-only và `yolo_detect_node`. |
| `yolo_hough_yaw_estimator.launch.py` | Chạy riêng yaw estimator từ RGB + YOLO JSON. |
| `yolo_depth_xyz_from_intrinsic.launch.py` | Chạy node calibration/test `yolo_depth_xyz_from_intrinsic`. |
| `aruco_detect.launch.py` | Luồng ArUco riêng, không thuộc YOLO RGB-D main pipeline. |
| `aruco_extrinsic_calibrator.launch.py` | Calib ngoại camera màu RealSense từ board ChArUco 6x8, publish TF `aruco_world -> camera_color_optical_frame`. |

## ChArUco Extrinsic Camera Calibration

Node calib ngoại mới chạy độc lập với pipeline YOLO:

```bash
ros2 run robot_vision_pipeline aruco_extrinsic_calibrator_node
```

Hoặc chạy bằng launch:

```bash
ros2 launch robot_vision_pipeline aruco_extrinsic_calibrator.launch.py
```

Input mặc định:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

Output:

```text
/vision/aruco_calib/debug_image
TF: aruco_world -> camera_color_optical_frame
```

Node lấy `fx`, `fy`, `cx`, `cy` và distortion coefficients `D` trực tiếp từ `/camera/camera/color/camera_info`; không dùng intrinsic hard-code và không dùng depth để calib ngoại.

World frame của board được định nghĩa trong `config/aruco_board_layout.yaml`:

```text
O_world: tâm hình học của ChArUco board 6x8
X_world+: sang phải theo chiều ngang board
Y_world+: đi lên theo chiều dọc board
Z_world+: hướng lên khỏi mặt board
```

Board mặc định:

```text
squares_x = 6
squares_y = 8
square_size = 0.029 m
marker_size = 0.021 m
board_width = 0.174 m
board_height = 0.232 m
```

ChArUco corners là nguồn calib chính. Với ChArUco corner theo world frame tự định nghĩa:

```text
x = c * square_size - board_width / 2
y = r * square_size - board_height / 2
z = 0
```

Node dùng `cv2.aruco.interpolateCornersCharuco()` để lấy ChArUco image corners, sau đó tự tạo object points theo gốc ở tâm board. Nếu không đủ ChArUco corners, node mới fallback sang 4 góc marker ArUco từ `marker_layout`.

Trước `solvePnP`, node lọc các ChArUco correspondence bằng homography RANSAC trên mặt phẳng board. Bước này loại các corner bị nội suy sai do marker rời/trùng ID ngoài board, rồi chỉ đưa các inlier cùng một mặt phẳng board vào PnP.

File layout mẫu:

```yaml
marker_size: 0.021
square_size: 0.029
squares_x: 6
squares_y: 8
aruco_dictionary: DICT_4X4_50

world_frame: aruco_world
camera_frame: camera_color_optical_frame

origin_convention: center
x_axis: right
y_axis: up
z_axis: out_of_board

use_board_roi: false

# Fallback ArUco only:
marker_layout:
  0: {row: 0, col: 0}
  1: {row: 0, col: 1}
  2: {row: 0, col: 2}
  3: {row: 1, col: 0}
  4: {row: 1, col: 1}
  5: {row: 1, col: 2}
```

Không cần crop ROI trong luồng ChArUco chính. Nếu trong ảnh có marker rời, chúng không tạo ChArUco corners hợp lệ với board 6x8 thì không được dùng cho solvePnP ChArUco.

Chạy với RealSense D435:

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

ros2 run robot_vision_pipeline aruco_extrinsic_calibrator_node --ros-args \
  -p board_layout_file:=/home/asus/ros_vision/src/robot_vision_pipeline/config/aruco_board_layout.yaml \
  -p publish_tf:=true \
  -p save_result:=true \
  -p print_marker_debug:=true
```

Xem debug image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Chọn topic:

```text
/vision/aruco_calib/debug_image
```

Xem TF:

```bash
ros2 run tf2_ros tf2_echo aruco_world camera_color_optical_frame
```

Kết quả solvePnP của OpenCV là `T_camera_world`, tức:

```text
P_camera = T_camera_world * P_world
```

Node luôn đảo transform trước khi log, save YAML và publish TF:

```text
T_world_camera = inverse(T_camera_world)
P_world = T_world_camera * P_camera
```

Vì vậy TF `aruco_world -> camera_color_optical_frame` và YAML `T_world_camera` là pose camera trong world frame.

Kết quả YAML mặc định lưu ở:

```text
robot_vision_pipeline/config/aruco_extrinsic_result.yaml
```

Nội dung chính gồm:

```text
extrinsic_matrix/T_world_camera
extrinsic_matrix/T_camera_world
camera_pose_in_world: x, y, z, roll, pitch, yaw
camera_pose_quaternion
reprojection_error
camera_matrix
distortion_coefficients
board
debug
```

`T_world_camera` là pose camera trong world frame sau khi đảo từ `solvePnP`; đây là ma trận nên dùng cho calib ngoại. Các key cũ `translation`, `rotation_rpy`, `rotation_quaternion`, `T_world_camera`, `T_camera_world` vẫn được giữ để tương thích.

Đọc reprojection error:

```text
< 0.5 px       rất tốt
0.5 - 1.5 px   tốt
1.5 - 3 px     tạm chấp nhận
> 3 px         kiểm tra intrinsic, distortion, kích thước board, layout ID hoặc thứ tự corner
```

Nếu `mean_px > max_allowed_reprojection_error` mặc định `5.0 px`, node reject pose, không save YAML và không publish TF sai. Muốn publish TF xấu chỉ để debug thì bật:

```bash
-p publish_bad_tf_for_debug:=true
```

Nếu mean error lớn hàng trăm pixel như `878 px`, gần như chắc chắn object points/layout/corner order đang sai, hoặc fallback ArUco đang dùng layout sai.

Nếu status là `solvePnP failed`, kiểm tra log:

```text
detected_aruco_ids
num_interpolated_charuco_corners
object_points.shape
image_points.shape
camera_matrix K
distortion D
```

Nếu `num_interpolated_charuco_corners < min_charuco_corners`, kiểm tra board có đủ rõ trong ảnh, đúng dictionary `DICT_4X4_50`, đúng `squares_x/squares_y`, `marker_size`, `square_size`, và `camera_info`.

Debug image vẽ marker ArUco, ChArUco corners, `O_world` ở tâm board và 2 trục trên mặt board:

```text
X+ màu đỏ
Y+ màu xanh lá
O_world màu vàng
```

Debug image không vẽ `Z+` để tránh rối hình. Kết quả toán học vẫn giữ đầy đủ 6D gồm `x, y, z, roll, pitch, yaw`.

## Config Parameters Quan Trọng

`config/yolo_detect_real.yaml`:

- `image_topic`: `/camera/camera/color/image_raw`
- `model_path`: `/home/asus/ros_vision/src/robot_vision_pipeline/robot_vision_pipeline/models/yolov8.pt`
- `conf_threshold`: `0.35`
- `iou_threshold`: `0.45`
- `imgsz`: `256`
- `max_det`: `20`
- `class_filter`: `wood,box`
- `enable_roi_crop`: `true`
- `roi_x`, `roi_y`, `roi_width`, `roi_height`
- `publish_roi_debug_image`: `true`

Ghi chú: `model_path` ở trên là giá trị đang ghi trong YAML. Trong workspace hiện tại cũng có thư mục model ở `/home/asus/ros_vision/src/robot_vision_pipeline/models`. Nếu YOLO báo lỗi không load được model, kiểm tra đường dẫn file `.pt` thật hoặc truyền override bằng launch argument `model_path:=...`.

`config/yolo_json_adapter.yaml`:

- `detections_json_topic`: `/vision/yolo/detections_json`
- `depth_topic`: `/camera/camera/aligned_depth_to_color/image_raw`
- `depth_roi_half_size`: `5`
- `fake_depth_m`: `0.55`
- `use_fake_depth`: `false`

`config/pixel_to_base_mapper.yaml`:

- `output_frame_id`: `camera_color_optical_frame`
- `camera_info_topic`: `/camera/camera/color/camera_info`
- `color_image_topic`: `/camera/camera/color/image_raw`
- `depth_image_topic`: `/camera/camera/aligned_depth_to_color/image_raw`
- `wood_detection_topic`: `/vision/wood_detection`
- `box_detection_topic`: `/vision/box_detection`
- `wood_objects_topic`: `/vision/wood_objects`
- `box_objects_topic`: `/vision/box_objects`
- `enable_yaw_estimation`: có trong YAML nhưng mapper hiện dùng tham số `use_hough_yaw`/`use_hough_yaw_for_wood` từ code/launch.
- `yaw_avg_frames`: có trong YAML nhưng không được mapper hiện tại dùng cho smoothing yaw Hough.

## Cách Build

```bash
cd ~/ros_vision
colcon build --packages-select robot_vision_pipeline --symlink-install
source install/setup.bash
```

Kiểm tra executable:

```bash
ros2 pkg executables robot_vision_pipeline
```

Cần thấy các executable chính:

```text
robot_vision_pipeline yolo_detect_node
robot_vision_pipeline yolo_json_to_object_detection_node
robot_vision_pipeline yolo_hough_yaw_estimator_node
robot_vision_pipeline pixel_to_base_mapper_node
```

## Cách chạy

Chạy full pipeline:

```bash
cd ~/ros_vision
source install/setup.bash
ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py
```

Chạy RealSense trước nếu muốn tự kiểm tra camera:

```bash
ros2 launch realsense2_camera rs_launch.py
```

Chạy YOLO riêng:

```bash
ros2 launch robot_vision_pipeline yolo_detect_real.launch.py
```

Chạy yaw estimator riêng khi đã có camera + YOLO JSON:

```bash
ros2 launch robot_vision_pipeline yolo_hough_yaw_estimator.launch.py
```

Chạy test bằng ảnh tĩnh:

```bash
ros2 launch robot_vision_pipeline vision_image_test.launch.py \
  image_path:=/absolute/path/to/image.jpg \
  model_path:=/absolute/path/to/model.pt \
  fake_depth_m:=0.55
```

## Cách Debug

Kiểm tra topic:

```bash
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /vision/yolo/detections_json
ros2 topic echo /vision/yolo/detections_json
ros2 topic echo /vision/wood_detection
ros2 topic echo /vision/box_detection
ros2 topic echo /vision/yolo/hough_yaw_json
ros2 topic echo /vision/wood_objects
ros2 topic echo /vision/box_objects
```

Xem ảnh:

```bash
ros2 run rqt_image_view rqt_image_view
```

Các topic ảnh nên xem theo thứ tự:

```text
/vision/yolo/image_annotated
/vision/yolo/roi_debug_image
/vision/debug_hough_yaw_image
/vision/debug_hough_edges
/vision/debug_image_camera
```

Khi debug yaw:

1. Xem `/vision/yolo/image_annotated` để chắc bbox YOLO đúng.
2. Xem `/vision/yolo/roi_debug_image` để chắc ROI YOLO không cắt mất vật.
3. Xem `/vision/debug_hough_edges` để kiểm tra Canny có bắt đúng cạnh gỗ không.
4. Xem `/vision/debug_hough_yaw_image` để kiểm tra line Hough và mũi tên yaw.
5. Echo `/vision/yolo/hough_yaw_json` để xem yaw có nhảy không.
6. Echo `/vision/wood_objects` để xem yaw đã vào quaternion chưa.

## Ghi chú khi chỉnh tham số

Không tự ý tăng/giảm `conf_threshold` YOLO nếu vấn đề là mũi tên yaw nhảy trong khi bbox vẫn ổn. Các tham số nên kiểm tra trước nằm ở yaw/ROI:

- `bbox_padding`
- `canny_low`, `canny_high`
- `hough_threshold`
- `min_line_length`
- `max_line_gap`
- ROI YOLO: `roi_x`, `roi_y`, `roi_width`, `roi_height`
- Mapper yaw match: `yaw_match_max_center_dist_px`
- Mapper yaw timeout: `yaw_stale_timeout_sec`
- Debug arrow: `draw_wood_yaw_arrow`, `use_yaw_arrow_from_hough_json`

Nếu muốn giảm jitter yaw bằng code trong tương lai, các hướng hợp lý là smoothing nhiều frame, giữ yaw cũ khi line yếu, giới hạn max yaw jump/frame, hoặc chọn line theo tiêu chí ổn định hơn thay vì chỉ chọn line dài nhất. README này chỉ mô tả hiện trạng code, không thay đổi thuật toán.
