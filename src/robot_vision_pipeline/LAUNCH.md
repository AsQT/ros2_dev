# robot_vision_pipeline - Launch Guide

## 1. Danh sách launch file
| Launch file | Mục đích | Node được chạy | Điều kiện cần |
|---|---|---|---|
| `vision_full_pipeline.launch.py` | Pipeline RealSense đầy đủ | YOLO, adapter, yaw, mapper, markers, optional rs_launch | Camera/model/calib |
| `vision_image_test.launch.py` | Test bằng ảnh tĩnh | static camera + pipeline | `image_path` hợp lệ |
| `yolo_detect_real.launch.py` | YOLO camera thật | rs_launch + yolo_detect_node | RealSense |
| `yolo_detect_sim.launch.py` | YOLO nguồn sim/override | yolo_detect_node | Topic ảnh |
| `yolo_hough_yaw_estimator.launch.py` | Estimate yaw | yolo_hough_yaw_estimator_node | JSON + image |
| `yolo_depth_xyz_from_intrinsic.launch.py` | XYZ từ depth/intrinsic | yolo_depth_xyz_from_intrinsic | Depth + intrinsic |
| `aruco_detect.launch.py` | ArUco detection | aruco_detect_node | Image + config |
| `aruco_extrinsic_calibrator.launch.py` | Calibrate extrinsic | aruco_extrinsic_calibrator_node | Board layout + CameraInfo |

## 2. Chi tiết từng launch file
### vision_full_pipeline.launch.py
Chạy pipeline chính. Include `realsense2_camera/rs_launch.py` nếu `use_camera=true`; chạy YOLO, adapter, yaw, mapper, marker node.
```bash
ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py
```

### vision_image_test.launch.py
Phát ảnh tĩnh/fake depth rồi chạy pipeline để debug không cần camera.
```bash
ros2 launch robot_vision_pipeline vision_image_test.launch.py image_path:=/path/to/image.png
```

### yolo_detect_real.launch.py
Chạy RealSense và YOLO detection.
```bash
ros2 launch robot_vision_pipeline yolo_detect_real.launch.py
```

### yolo_detect_sim.launch.py
Chạy YOLO với topic ảnh override/sim.
```bash
ros2 launch robot_vision_pipeline yolo_detect_sim.launch.py image_topic:=/camera/image_raw
```

### yolo_hough_yaw_estimator.launch.py
Ước lượng yaw từ detection JSON và ảnh.
```bash
ros2 launch robot_vision_pipeline yolo_hough_yaw_estimator.launch.py
```

### yolo_depth_xyz_from_intrinsic.launch.py
Tính XYZ từ intrinsic/depth.
```bash
ros2 launch robot_vision_pipeline yolo_depth_xyz_from_intrinsic.launch.py
```

### aruco_detect.launch.py
Detect ArUco pose.
```bash
ros2 launch robot_vision_pipeline aruco_detect.launch.py
```

### aruco_extrinsic_calibrator.launch.py
Tạo/cập nhật extrinsic camera-world.
```bash
ros2 launch robot_vision_pipeline aruco_extrinsic_calibrator.launch.py
```

#### Lỗi thường gặp
Thiếu model YOLO, topic camera sai, CameraInfo không đồng bộ, YAML calibration chưa đúng.
