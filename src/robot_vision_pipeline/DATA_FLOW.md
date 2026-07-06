# robot_vision_pipeline - Data Flow

## 1. Mục tiêu luồng dữ liệu
Chuyển ảnh/depth/camera info thành detection JSON, object pose, yaw và marker visualization.

## 2. Input
Camera RealSense hoặc static image, depth image, CameraInfo, model YOLO, intrinsic/extrinsic YAML, ArUco board layout.

## 3. Output
YOLO detection JSON, Hough yaw JSON, WoodArray/BoxArray, ArUcoPoseArray, debug images, MarkerArray.

## 4. Internal processing
YOLO detect -> JSON adapter -> yaw estimator -> depth/intrinsic/extrinsic mapper -> markers/object arrays.

## 5. Sơ đồ luồng dữ liệu
```mermaid
flowchart LR
  Camera[Camera/static image] --> YOLO[yolo_detect_node]
  YOLO --> Json[/vision/yolo/detections_json]
  Json --> Adapter[yolo_json_to_object_detection]
  Json --> Yaw[yolo_hough_yaw_estimator]
  Adapter --> Mapper[pixel_to_base_mapper]
  Yaw --> Mapper
  Depth[Depth + CameraInfo] --> Mapper
  Mapper --> Objects[/vision/wood_objects + /vision/box_objects]
  Mapper --> Markers[/vision/object_markers]
```

## 6. Liên kết với package khác
Dùng `robot_vision_pipeline_msgs` cho object messages; downstream có thể là GUI, DRL planner hoặc RViz.

## 7. Các điểm cần chú ý
- Đơn vị depth raw phụ thuộc encoding camera; output pose dùng mét.
- Calibration sai làm pose base/world sai dù detection đúng.
- Cần xác nhận topic camera sau khi launch RealSense.
