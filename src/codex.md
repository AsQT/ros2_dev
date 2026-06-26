Trong pkg `robot_gui`, sửa triệt để lỗi **font khu vực action log phía dưới màn hình vẫn quá lớn**.

## Hiện tượng

Khi chạy GUI và gọi action, khu vực log nằm dưới RViz và TaskControlPanel hiển thị chữ cực lớn, ví dụ:

```text
feedback stage=Move directly to place ...
```

Chữ quá to nên bị cắt, không đọc được đầy đủ log.

Lần sửa trước chưa đúng, có thể đã sửa nhầm widget hoặc style trong `.ui` bị code runtime ghi đè.

## Yêu cầu chính

Phải tìm đúng widget thực tế đang được dùng để hiển thị log action, rồi giảm font về `8pt` hoặc `9pt`.

Không chỉ sửa theo suy đoán objectName `txtActionLog`. Phải trace từ code nơi append/set log action.

---

## 1. Tìm đúng widget đang hiển thị log

Trong pkg `robot_gui`, tìm toàn bộ code liên quan log action:

```bash
cd ~/ros2_dev/src/robot_gui
grep -R "feedback stage" -n .
grep -R "appendActionLog" -n .
grep -R "txtActionLog" -n .
grep -R "setText" -n src include
grep -R "appendPlainText" -n src include
grep -R "append" -n src include
grep -R "ActionLog" -n .
grep -R "action log" -ni .
```

Mục tiêu là xác định chính xác widget nào đang nhận text log.

Có thể là một trong các loại:

```cpp
QLabel
QTextEdit
QPlainTextEdit
QTextBrowser
```

Nếu widget đang là `QLabel` thì không đủ phù hợp cho log nhiều dòng. Nên đổi sang `QPlainTextEdit` hoặc đảm bảo QLabel có font nhỏ và word wrap.

---

## 2. Sửa bằng code runtime, không chỉ sửa `.ui`

Sau `setupUi(this)` hoặc sau khi tạo `TaskActionController`, set trực tiếp font/style cho widget log thực tế.

Ví dụ nếu widget là `QPlainTextEdit`:

```cpp
ui->txtActionLog->setReadOnly(true);
ui->txtActionLog->setLineWrapMode(QPlainTextEdit::NoWrap);

QFont logFont("DejaVu Sans Mono");
logFont.setPointSize(8);
ui->txtActionLog->setFont(logFont);

ui->txtActionLog->setStyleSheet(
    "QPlainTextEdit#txtActionLog {"
    " font-family: 'DejaVu Sans Mono';"
    " font-size: 8pt;"
    " color: #111111;"
    " background-color: white;"
    "}"
);
```

Nếu widget là `QTextEdit`:

```cpp
ui->txtActionLog->setReadOnly(true);

QFont logFont("DejaVu Sans Mono");
logFont.setPointSize(8);
ui->txtActionLog->setFont(logFont);

ui->txtActionLog->setStyleSheet(
    "QTextEdit#txtActionLog {"
    " font-family: 'DejaVu Sans Mono';"
    " font-size: 8pt;"
    " color: #111111;"
    " background-color: white;"
    "}"
);
```

Nếu widget thực tế là `QLabel`:

```cpp
ui->txtActionLog->setWordWrap(true);
ui->txtActionLog->setAlignment(Qt::AlignLeft | Qt::AlignTop);

QFont logFont("DejaVu Sans Mono");
logFont.setPointSize(8);
ui->txtActionLog->setFont(logFont);

ui->txtActionLog->setStyleSheet(
    "QLabel#txtActionLog {"
    " font-family: 'DejaVu Sans Mono';"
    " font-size: 8pt;"
    " color: #111111;"
    " background-color: white;"
    " padding: 6px;"
    "}"
);
```

Quan trọng: dùng đúng objectName thực tế, không bắt buộc phải là `txtActionLog` nếu file đang dùng tên khác.

---

## 3. Nếu hiện tại log đang dùng QLabel thì nên đổi sang QPlainTextEdit

Nếu khu vực dưới cùng đang là `QLabel` và dùng `setText()` để hiển thị log, cần cân nhắc sửa lại thành `QPlainTextEdit` để log nhiều dòng dễ đọc.

Yêu cầu:

* ObjectName nên là:

```text
txtActionLog
```

* Font `8pt`.
* ReadOnly.
* Có scroll.
* Append log từng dòng, không phóng to chữ.

Trong code, thay vì:

```cpp
ui->xxxLabel->setText(log);
```

nên dùng:

```cpp
ui->txtActionLog->appendPlainText(log);
```

Nếu không muốn đổi layout nhiều, chỉ cần set font runtime cho QLabel hiện có, nhưng phải đảm bảo chữ không còn to.

---

## 4. Kiểm tra stylesheet global đang ghi đè

Tìm trong `.ui`, `.qss`, `.cpp` các style font lớn:

```bash
grep -R "font-size" -n ~/ros2_dev/src/robot_gui
grep -R "pointsize" -n ~/ros2_dev/src/robot_gui
grep -R "setPointSize" -n ~/ros2_dev/src/robot_gui
grep -R "setStyleSheet" -n ~/ros2_dev/src/robot_gui
```

Nếu có style global kiểu:

```css
QLabel {
    font-size: 48px;
}
```

hoặc:

```css
QFrame QLabel {
    font-size: 48px;
}
```

thì phải override riêng cho widget log bằng selector objectName cụ thể:

```css
QLabel#txtActionLog {
    font-size: 8pt;
}
```

hoặc:

```css
QPlainTextEdit#txtActionLog {
    font-size: 8pt;
}
```

Không sửa font toàn bộ GUI.

---

## 5. Đảm bảo `appendActionLog()` không làm mất style

Kiểm tra hàm log, ví dụ:

```cpp
appendActionLog(...)
```

Không được dùng HTML với size lớn kiểu:

```cpp
"<span style='font-size:48px'>...</span>"
```

Không được dùng `setText()` với rich text có font lớn.

Nếu đang dùng `QTextEdit`, có thể append plain text:

```cpp
ui->txtActionLog->append(message);
```

Nhưng phải đảm bảo không insert HTML font-size lớn.

Ưu tiên dùng `QPlainTextEdit`:

```cpp
ui->txtActionLog->appendPlainText(message);
```

---

## 6. Kết quả mong muốn

Sau khi sửa, khu vực log phía dưới phải hiển thị dạng nhỏ gọn, ví dụ:

```text
[Pick Place] Sending goal to /pickplace, execute=true
[Pick Place] feedback stage=Open gripper, progress=5%
[Pick Place] feedback stage=Move directly to place approach, progress=70%
[Pick Place] result success=true, message=...
```

Font khoảng `8pt` hoặc `9pt`, đọc được nhiều dòng, không bị phóng to và không bị cắt như hiện tại.

---

## 7. Build và test

Build lại:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_gui --symlink-install
source install/setup.bash
```

Chạy task servers:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Chạy GUI.

Test:

1. Vào tab `Pick Place`.
2. Bấm `Start` hoặc `Plan`.
3. Quan sát khu vực log phía dưới.
4. Xác nhận font đã nhỏ.
5. Xác nhận log không bị cắt.
6. Xác nhận GUI không crash.
7. Xác nhận các vùng khác không bị đổi font ngoài ý muốn.

---

## 8. Báo cáo

Cập nhật file:

```text
src/robot_gui/task_action_gui_report.md
```

Thêm rõ:

```text
## Fix action log font

- Nguyên nhân font log vẫn to:
  - <ghi rõ do widget nào / stylesheet nào / code nào ghi đè>
- Widget thực tế hiển thị log:
  - <objectName>
  - <class: QLabel/QPlainTextEdit/QTextEdit/...>
- File đã sửa:
  - <danh sách file>
- Font sau sửa:
  - 8pt hoặc 9pt
- Kết quả test:
  - Log action đã hiển thị nhỏ, đọc được nhiều dòng.
```

Chỉ xem là hoàn thành khi ảnh/log thực tế trong GUI không còn chữ lớn như hiện tại.
