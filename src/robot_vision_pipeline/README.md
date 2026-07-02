# robot_vision_pipeline

## 1. Vai trò package
Pipeline perception Python dùng YOLO, ArUco, depth/intrinsic/extrinsic và marker RViz để xuất object pose cho robot/DRL/GUI.

## 2. Vị trí trong hệ thống
Đứng trước planner/task layer. Nhận ảnh RealSense hoặc static image, xuất detection/object pose/marker cho `robot_drl`, GUI hoặc node downstream.

## 3. Thành phần chính
- YOLO: `yolo_detect_node`, `yolo_json_to_object_detection_node`.
- Pose/yaw: `pixel_to_base_mapper_node`, `yolo_hough_yaw_estimator_node`, `vision_detection_marker_node`.
- ArUco/calibration: `aruco_detect_node`, `aruco_extrinsic_calibrator_node`, `calib_camera_to_world_aruco_5points`.
- Depth helpers: `yolo_depth_to_camera`, `yolo_depth_xyz_from_intrinsic`, `yolo_wood_center_to_world`.
- Test: `static_image_camera_node`.
- GUI tools: `vision_gui`, `vision_gui_astra`.

## 4. Node / executable
| Node / executable | Nguồn | Vai trò |
|---|---|---|
| `yolo_detect_node` | `robot_vision_pipeline/yolo/yolo_detect_node.py` | YOLO detection |
| `yolo_json_to_object_detection_node` | Python package | Convert JSON detection |
| `pixel_to_base_mapper_node` | pose_estimation | Map pixel/depth to base/world |
| `yolo_hough_yaw_estimator_node` | pose_estimation | Estimate yaw |
| `vision_detection_marker_node` | pose_estimation | RViz markers |
| `aruco_detect_node` | aruco | ArUco pose |
| `aruco_extrinsic_calibrator_node` | calib | Camera-world calibration |
| `static_image_camera_node` | test/main package | Static image source |

## 5. Topic / Service / Action
| Interface | Type | Vai trò |
|---|---|---|
| `/camera/camera/color/image_raw` | Image | Ảnh màu đầu vào |
| `/camera/camera/aligned_depth_to_color/image_raw` | Image | Depth đầu vào |
| `/vision/yolo/detections_json` | String | YOLO detections JSON |
| `/vision/yolo/hough_yaw_json` | String | Yaw estimation JSON |
| `/vision/woods`, `/vision/boxes` | WoodArray/BoxArray | Object pose output |
| `/vision/object_markers` | MarkerArray | RViz visualization |

## 6. File launch liên quan
`vision_full_pipeline.launch.py`, `vision_image_test.launch.py`, `yolo_detect_real.launch.py`, `yolo_detect_sim.launch.py`, `yolo_hough_yaw_estimator.launch.py`, `yolo_depth_xyz_from_intrinsic.launch.py`, `aruco_detect.launch.py`, `aruco_extrinsic_calibrator.launch.py`.

## 7. File cấu hình liên quan
`config/yolo_detect_real.yaml`, `yolo_json_adapter.yaml`, `pixel_to_base_mapper.yaml`, `vision_markers.yaml`, `aruco_detect.yaml`, `aruco_board_layout.yaml`, `Intrinsic.yaml`, `Extrinsic_camera_to_world.yaml`, `rs_camera*.yaml`.

## 8. Cách build riêng package
```bash
cd ~/ros2_dev
colcon build --packages-select robot_vision_pipeline
source install/setup.bash
```

## 9. Cách chạy nhanh
```bash
ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py
```
Test ảnh tĩnh:
```bash
ros2 launch robot_vision_pipeline vision_image_test.launch.py image_path:=/path/to/image.png
```

## 10. Ghi chú kỹ thuật / giới hạn hiện tại
- Cần model YOLO hợp lệ và calibration YAML đúng camera.
- Topic thực tế có thể đổi qua launch/YAML override.
- Output pose phụ thuộc frame `aruco_world`/world transform khi bật `use_world_transform`.
