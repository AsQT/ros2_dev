# robot_vision_pipeline - Parameters

## 1. Tổng quan
Parameter nằm trong Python nodes, launch args và YAML trong `config/`.

## 2. Bảng parameter
| Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
|---|---:|---|---|---|---|
| `model_path` | package/model override | path | launch/node | YOLO node | Đường dẫn model YOLO |
| `image_topic` | `/camera/camera/color/image_raw` | string | launch/YAML | YOLO/yaw | Topic ảnh màu |
| `depth_image_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | string | YAML/node | depth mapper | Topic depth |
| `conf_threshold` | `0.35` | double | node/YAML | YOLO | Ngưỡng confidence |
| `iou_threshold` | `0.45` | double | node/YAML | YOLO | NMS IoU |
| `imgsz` | `640` | int | node/YAML | YOLO | Input size |
| `device` | `cpu` | string | node/YAML | YOLO | Thiết bị infer |
| `world_frame_id` | `aruco_world` | string | launch/mapper | mapper | Frame world output |
| `use_world_transform` | `true` | bool | launch/mapper | mapper | Dùng extrinsic camera-world |
| `bbox_padding` | `25` | int | yaw launch | yaw estimator | Padding bbox |
| `canny_low/high` | `30/100` | int | yaw launch | yaw estimator | Canny thresholds |
| `hough_threshold` | `10` | int | yaw launch | yaw estimator | Hough line threshold |
| `board_layout_file` | `aruco_board_layout.yaml` | path | calibrator launch | ArUco calib | Layout board |
| `result_yaml_path` | `aruco_extrinsic_result.yaml` | path | calibrator launch | ArUco calib | Output extrinsic |

## 3. Parameter theo launch file
Xem các launch trong package; `vision_full_pipeline` có `use_camera`, `use_mapper`, `use_markers`, `model_path`, `image_topic`, `use_world_transform`, `world_frame_id`, `extrinsic_yaml_path`.

## 4. Parameter theo YAML config
YAML trong `config/` chứa camera, intrinsic/extrinsic, YOLO, adapter, mapper, marker và ArUco settings.

## 5. Giá trị mặc định quan trọng
YOLO thường chạy `cpu` nếu không override; confidence mặc định khoảng `0.35`; frame world mặc định `aruco_world` trong launch tổng hợp.

## 6. Ghi chú thay đổi / rủi ro cấu hình
Model path, intrinsic/extrinsic và RealSense topic là ba nguồn lỗi chính. Sau khi đổi YAML cần source/relaunch node.
