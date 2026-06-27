# robot_task_executor_msgs

## 1. Vai trò package
Package interface định nghĩa các service contract cho executor MoveIt dạng service. Source of truth là `srv/*.srv` và `CMakeLists.txt` hiện tại.

## 2. Vị trí trong hệ thống
Được `robot_task_executor` dùng cho service-based task execution và `robot_drl_executor` dùng lại `MoveCartesianPoseSequence.srv` cho DRL Cartesian executor.

## 3. Thành phần chính
- `srv/MoveToNamedTarget.srv`: move tới named target trong SRDF.
- `srv/MoveToJointTarget.srv`: move tới joint target.
- `srv/MoveToPoseTarget.srv`: move tới pose target.
- `srv/MoveToNamedPoseTarget.srv`: move tới pose đã lưu.
- `srv/MoveToCartesianTarget.srv`: Cartesian tới điểm x/y/z.
- `srv/MoveToNamedCartesianTarget.srv`: Cartesian tới target tên.
- `srv/MoveCartesianSequence.srv`: sequence cartesian từ waypoint name.
- `srv/MoveCartesianPoseSequence.srv`: sequence `PoseStamped[]`, đang dùng bởi DRL executor.
- `srv/MoveSequence.srv`: sequence named joint waypoint.

## 4. Node / executable
| Node / executable | Nguồn | Vai trò |
|---|---|---|
| Không có | interface-only | Chỉ generate service type |

## 5. Topic / Service / Action
| Interface | Type | Vai trò |
|---|---|---|
| `MoveToNamedTarget` | srv | Named MoveIt target |
| `MoveToJointTarget` | srv | Joint-space target |
| `MoveToPoseTarget` | srv | Pose target |
| `MoveCartesianPoseSequence` | srv | PoseStamped[] Cartesian path |
| `MoveSequence` | srv | Named sequence |

## 6. File launch liên quan
Không có launch file riêng.

## 7. File cấu hình liên quan
Không có YAML config riêng.

## 8. Cách build riêng package
```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_executor_msgs
source install/setup.bash
```

## 9. Cách chạy nhanh
Package này không chạy độc lập; dùng bằng service type trong client/server.

## 10. Ghi chú kỹ thuật / giới hạn hiện tại
- Các request thường có `execute`; `execute=false` là plan-only ở server triển khai.
- Cartesian response có `fraction`; service non-Cartesian chủ yếu trả `success/message`.
