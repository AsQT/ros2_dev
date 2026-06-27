# robot_task_executor_msgs - Parameters

## 1. Tổng quan
Package này không khai báo runtime parameter vì chỉ generate service interface.

## 2. Bảng parameter
| Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
|---|---:|---|---|---|---|
| Không có | - | - | - | - | Interface-only package |

## 3. Parameter theo launch file
Không có launch file.

## 4. Parameter theo YAML config
Không có YAML config.

## 5. Giá trị mặc định quan trọng
Không có trong package này; default nằm ở executor triển khai service.

## 6. Ghi chú thay đổi / rủi ro cấu hình
Thay đổi `.srv` là thay đổi ABI/API ROSIDL, cần rebuild các package phụ thuộc.
