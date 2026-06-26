Trong pkg `robot_gui`, kiểm tra file layout:

```text
robot_gui/ui/robot_gui.ui
```

Tìm khu vực hiển thị log action ở phần dưới cùng màn hình, nơi đang dùng để hiển thị thông tin khi gọi action. Khu vực này có thể là một trong các widget sau:

```text
txtActionLog
actionLog
actionLogText
actionLogTextEdit
actionStatusLog
QTextEdit
QPlainTextEdit
```

## Yêu cầu sửa

Giảm cỡ chữ khu vực log action vì hiện tại chữ đang quá to, khó đọc.

Ưu tiên nếu có widget tên:

```text
txtActionLog
```

thì chỉ sửa style/font của đúng widget này.

## Style đề xuất

Nếu là `QPlainTextEdit` hoặc `QTextEdit`, đặt font nhỏ hơn:

```css
QPlainTextEdit#txtActionLog,
QTextEdit#txtActionLog {
    font-family: "DejaVu Sans Mono";
    font-size: 8pt;
}
```

Nếu hiện tại objectName khác `txtActionLog`, dùng đúng objectName thực tế, không đổi tên nếu code đang dùng.

Ví dụ:

```css
QPlainTextEdit#<objectName_thuc_te> {
    font-family: "DejaVu Sans Mono";
    font-size: 8pt;
}
```

## Quy tắc bắt buộc

* Chỉ sửa khu vực hiển thị log action.
* Không sửa font toàn bộ `TaskControlPanel`.
* Không sửa font toàn bộ `MainWindow`.
* Không đổi layout các tab.
* Không đổi logic gọi action.
* Không đổi objectName widget nếu code C++ đang dùng.
* Không chỉnh các label/nút/ô nhập dữ liệu khác.

## Nếu trong `.ui` đang set font bằng property

Nếu widget log đang có property:

```xml
<property name="font">
```

thì sửa riêng point size về:

```text
8 hoặc 9
```

Khuyến nghị dùng `8pt`.

## Nếu không tìm thấy widget log

Tìm trong `.ui` các widget loại:

```xml
<class>QTextEdit</class>
<class>QPlainTextEdit</class>
```

hoặc tìm các objectName có chứa:

```text
log
status
action
message
```

Sau đó xác định widget nằm ở khu vực dưới cùng màn hình, bên dưới RViz và TaskControlPanel, rồi chỉ sửa font của widget đó.

## Kiểm tra sau khi sửa

Build lại:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_gui --symlink-install
source install/setup.bash
```

Chạy GUI, gọi thử một action bất kỳ và kiểm tra:

* Log action hiển thị nhỏ hơn.
* Có thể đọc được nhiều dòng hơn.
* Không ảnh hưởng font của các tab/nút/label khác.
* GUI không crash.

Sau khi hoàn thành, cập nhật báo cáo ngắn trong:

```text
robot_gui/task_action_gui_report.md
```

Thêm mục:

```text
- Đã giảm font khu vực action log về 8pt.
- Widget đã chỉnh: <objectName thực tế>.
```
