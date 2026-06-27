# robot_vision_pipeline_msgs

## 1. Vai trò package
Interface package chứa message cho object detection/pose output của vision pipeline.

## 2. Vị trí trong hệ thống
Được `robot_vision_pipeline` publish và các package downstream subscribe để dùng pose/size/confidence.

## 3. Thành phần chính
- `BoxDetection.msg`: bbox pixel + depth raw/distance.
- `Wood.msg`, `WoodArray.msg`: object wood pose.
- `Box.msg`, `BoxArray.msg`: box pose và size.

## 4. Node / executable
Không có node; chỉ generate message type.

## 5. Topic / Service / Action
| Interface | Type | Vai trò |
|---|---|---|
| `BoxDetection` | msg | Detection bbox/depth |
| `Wood`, `WoodArray` | msg | Wood pose array |
| `Box`, `BoxArray` | msg | Box pose/size array |

## 6. File launch liên quan
Không có.

## 7. File cấu hình liên quan
Không có.

## 8. Cách build riêng package
```bash
cd ~/ros2_dev
colcon build --packages-select robot_vision_pipeline_msgs
source install/setup.bash
```

## 9. Cách chạy nhanh
Không chạy trực tiếp; dùng bằng topic type.

## 10. Ghi chú kỹ thuật / giới hạn hiện tại
Thay đổi message cần rebuild package publish/subscribe phụ thuộc.
