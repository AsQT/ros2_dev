# Image Topic Display Report

## 1. Package check
- robot_gui path: `/home/minhquang/ros2_dev/src/robot_gui`
- package type: `ament_cmake`
- executable: `robot_gui_node`
- confirmed C++ package: yes
- robot_gui_old ignored: yes

## 2. UI object mapping

| Image panel | Widget objectName | Widget type | Found in .ui | Connected |
|---|---|---|---|---|
| Raw Image | `rawImageView` | `QLabel` | yes | yes |
| Detection Image | `detectionImageView` | `QLabel` | yes | yes |
| YOLO Image | `yoloPreviewWidget` | `QLabel` | yes | yes |

## 3. Config

- Config file: `robot_gui/config/config.yaml`
- Installed config: `install/robot_gui/share/robot_gui/config/config.yaml`
- raw topic: `/camera/color/image_raw`
- detection topic: `/yolo/detection_image`
- yolo topic: `/yolo/image`
- Topics read from parameter: yes

## 4. ROS subscribers

| Panel | Topic | Message type | QoS |
|---|---|---|---|
| Raw Image | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `rclcpp::SensorDataQoS()` |
| Detection Image | `/yolo/detection_image` | `sensor_msgs/msg/Image` | `rclcpp::SensorDataQoS()` |
| YOLO Image | `/yolo/image` | `sensor_msgs/msg/Image` | `rclcpp::SensorDataQoS()` |

## 5. Image conversion

- Supported encodings: `rgb8`, `bgr8`, `mono8`, `rgba8`, `bgra8`
- cv_bridge used: no
- QImage/QPixmap scaling: `Qt::KeepAspectRatio`, `Qt::SmoothTransformation`
- Unsupported encoding behavior: logs warning and leaves the existing placeholder or previous image unchanged

## 6. Placeholder behavior

- No topic: panel shows `<Panel title>` and `No topic configured`
- Waiting for image: panel shows `<Panel title>`, topic name, and `Waiting for image...`
- Topic name visible: yes
- Behavior when topic not published: GUI keeps waiting placeholder and does not crash

## 7. Test result

- Build: OK, `colcon build --packages-select robot_gui --event-handlers console_direct+`
- Config install: OK, `install/robot_gui/share/robot_gui/config/config.yaml`
- GUI standalone: OK, `ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=3` started and loaded image topics
- Placeholder without topics: OK by startup path and standalone launch without image publishers
- Mock raw image: OK, received `160x120` `rgb8` on `/camera/color/image_raw`
- Mock detection image: OK, received `160x120` `bgr8` on `/yolo/detection_image`
- Mock yolo image: OK, received `160x120` `mono8` on `/yolo/image`
- Config topic rename test: OK, params file changed detection to `/test/detection_image` and yolo to `/test/yolo_image`
- Remaining issues: none found in this task

## 8. Non-regression

- Layout from robot_gui.ui still used: yes, `.ui` was not edited
- Logo.png tab Main unaffected: yes
- RViz unaffected: yes, existing RViz setup path unchanged
- joint_states unaffected: yes
- robot_hw flags unaffected: yes
- servo_all unaffected: yes
