# robot_task_executor_msgs - Launch Guide

## 1. Danh sách launch file
| Launch file | Mục đích | Node được chạy | Điều kiện cần |
|---|---|---|---|
| Không có | Interface-only | Không có | Dùng qua dependency |

## 2. Chi tiết từng launch file
### Không có launch file

#### Chức năng
Package chỉ generate service type.

#### Node được khởi tạo
Không có.

#### Argument
Không có.

#### Parameter truyền vào node
Không có.

#### Package phụ thuộc
`geometry_msgs`, `rosidl_default_generators`.

#### Điều kiện thực thi
Build package trước khi build executor/client phụ thuộc.

#### Lệnh chạy
Không chạy trực tiếp.

#### Lỗi thường gặp
Client/server chưa source workspace sau khi thay đổi `.srv`.
