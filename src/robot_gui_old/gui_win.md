# Tài liệu GUI đã test OK trên Windows/Python

Tài liệu này ghi lại các phần GUI đã chỉnh và test OK ở mức UI độc lập trên Windows/Python. Mục tiêu là dùng làm cơ sở khi chuyển sang Linux/ROS2 và viết lại `main_window.py`.

## 1. Tổng quan

GUI được thiết kế bằng Qt Designer. File `.ui` chịu trách nhiệm cho layout, kích thước, style mặc định và `objectName` của widget. `main_window.py` chỉ chịu trách nhiệm load UI, kết nối button, cập nhật LED, cập nhật trạng thái nút và xử lý ROS2.

Nguyên tắc bắt buộc:

```text
Mọi layout, vị trí, kích thước, sizePolicy, minimumSize, maximumSize phải chỉnh trong file .ui.
main_window.py không được setGeometry(), move(), resize(), setFixedSize() để sửa layout.
```

Không tạo layout mới hoặc set geometry trong `main_window.py`, trừ các trường hợp runtime đặc biệt đã ghi rõ trong tài liệu này.

## 2. RViz embedded area

`embeddedRvizWidget` đã được đổi từ `QLabel` sang `QFrame` hoặc `QWidget` để làm container nhúng RViz.

Object name phải giữ nguyên:

```text
embeddedRvizWidget
```

Widget này dùng để nhúng `rviz2` bằng Python sau này. Layout bên trong là `QVBoxLayout`, margin `0,0,0,0`, spacing `0`. Nếu cần placeholder thì placeholder là `QLabel` con, không dùng `QLabel` chính làm container.

Ghi chú runtime khi viết lại trên Linux/ROS2:

- `main_window.py` sẽ chạy `rviz2` bằng subprocess.
- Sau đó dùng `QWindow.fromWinId()` và `QWidget.createWindowContainer()` để nhúng RViz vào `embeddedRvizWidget`.
- Cần ưu tiên X11/Xorg, không ưu tiên Wayland.

## 3. Image preview boxes

Các box hiển thị ảnh camera/YOLO vẫn dùng `QLabel`, ví dụ:

```text
yoloPreviewWidget
```

`VisionPreviewGroup` là `QFrame` chứa ảnh. `yoloPreviewWidget` là `QLabel`, dùng `setPixmap()` để hiển thị ảnh. Không đổi `QLabel` hiển thị ảnh thành `QFrame`.

Ghi chú:

- Ảnh nên scale trong Python bằng `Qt.KeepAspectRatio`.
- Layout của `VisionPreviewGroup` phải nằm trong `.ui`.
- Python chỉ cập nhật ảnh bằng `setPixmap()`.

## 4. SideMenu button selected state

Các nút menu bên trái đã test theo kiểu `checkable`. Nút nào tương ứng với page hiện tại thì giữ trạng thái selected.

Danh sách nút:

```text
btnHome
btnJog
btnMain
btnRobot
btnSetting
btnVision
```

Runtime cần viết lại:

- Set `setCheckable(True)` cho các nút nếu chưa được set trong `.ui`.
- Dùng `QButtonGroup`, `exclusive=True`.
- Khi đổi page trong `QStackedWidget`, phải sync lại nút tương ứng.
- Style active dùng `QPushButton:checked`.

Ghi chú:

- Hover chỉ là trạng thái khi rê chuột.
- Trạng thái page hiện tại phải dùng `checked`.

## 5. Button style chung

Các nút đã được chỉnh để dễ nhận biết là nút bấm. Button nên có:

- Gradient.
- Border rõ.
- Border-radius.
- Hover.
- Pressed.
- Padding hợp lý.

Ghi chú:

- Không nên dùng màu phẳng đơn giản cho các nút điều khiển chính.
- Các nút robot-level như `btnRobotEnable`, `btnRobotDisable` cần đồng bộ style với các nút `Run`, `Stop`, `Home`, `Jog`.

## 6. Status bar LEDs

Có 2 LED status bar:

```text
ConnectStatus_led
RobotEnableStatus_led
```

Quy ước màu:

- `ConnectStatus_led`:
  - Connected: xanh.
  - Disconnected: đỏ.
- `RobotEnableStatus_led`:
  - Robot enabled: xanh.
  - Disabled/error/disconnected: đỏ.

Ghi chú:

- Hiện tại trạng thái thật chưa match hoàn toàn, nên đã dùng chế độ test LED theo chu kỳ.
- Khi chuyển sang ROS2, cần cập nhật LED theo feedback thật từ hardware/robot status.
- Status bar LED không dùng màu xanh nhạt `#95c7ea`; màu đó chỉ dùng cho LED lỗi động cơ ở trạng thái bình thường.

## 7. Motor status LEDs

Các LED động cơ được chia theo từng trục.

Nhóm trạng thái thường:

```text
ledAxis{N}ServoOn
ledAxis{N}Running
ledAxis{N}OrgOK
ledAxis{N}LimitPositive
ledAxis{N}LimitNegative
```

Quy ước màu:

- Không tác động / False: xám.
- Có tác động / True: xanh.

Style OFF:

```css
background-color: #808080;
border: 1px solid #4b5563;
border-radius: 7px;
```

Style ON:

```css
background-color: #00cc66;
border: 1px solid #006b2d;
border-radius: 7px;
```

Nhóm cảnh báo/lỗi:

```text
ledAxis{N}Alarm
ledAxis{N}EMG
ledAxis{N}ErrorAll
```

Quy ước màu:

- Bình thường / không lỗi: `#95c7ea`.
- Có cảnh báo/lỗi: đỏ sáng.

Style normal:

```css
background-color: #95c7ea;
border: 1px solid #4b8db8;
border-radius: 2px;
```

Style warning/error:

```css
background-color: #ff3333;
border: 1px solid #a00018;
border-radius: 7px;
```

Ghi chú:

- LED runtime hiển thị dạng chấm bo tròn bằng stylesheet Python.
- Kích thước LED trong `.ui`: `14x14`.
- `text` của LED để rỗng.

## 7.1 `/robot_hw/flags` runtime source

GUI ROS2 đọc topic:

```text
/robot_hw/flags
robot_hardware_interface/msg/FlagStatus
```

Nguồn chính cho LED là:

```python
flags = [axis.status_f for axis in msg.axes]
```

Không đọc `msg.flags`, `msg.data`, hoặc field bool làm nguồn bitmask chính.

Mapping bắt buộc:

```text
msg.axes[0].status_f -> Axis 1
msg.axes[1].status_f -> Axis 2
msg.axes[2].status_f -> Axis 3
msg.axes[3].status_f -> Axis 4
msg.axes[4].status_f -> Axis 5
msg.axes[5].status_f -> Axis 6
```

Mask LED dùng flag 32-bit từ STM32:

```text
ERROR_ALL   = 0x00000001
SOF_LIMIT_P = 0x00000008
SOF_LIMIT_M = 0x00000010
EMG         = 0x00010000
SERVO_ON    = 0x00100000
ALARM       = 0x00200000
ORG_SET_OK  = 0x02000000
RUNNING     = 0x08000000
```

## 8. LED test mode

Đã test LED bằng `QTimer`.

Mục tiêu test:

- Xác nhận đúng widget LED.
- Xác nhận màu LED.
- Chưa phụ thuộc trạng thái thật của hệ thống.

Runtime logic cần ghi lại:

- `led_test_mode = True` để test.
- Khi `led_test_mode=True`, không cho trạng thái thật ghi đè LED.
- Test cycle:
  - Nhóm status thường: xám qua xanh và ngược lại.
  - Nhóm lỗi: `#95c7ea` qua đỏ và ngược lại.
  - Status bar LED: đỏ qua xanh nếu cần test riêng.

Khi chuyển sang ROS2:

- Đổi `led_test_mode = False`.
- Cập nhật LED bằng callback/status thật.

## 9. Axis control groups

`Axis1ControlGroup` là layout chuẩn. Các group còn lại đã được yêu cầu sửa theo `Axis1ControlGroup`:

```text
Axis2ControlGroup
Axis3ControlGroup
Axis4ControlGroup
Axis5ControlGroup
Axis6ControlGroup
```

Các thành phần theo pattern:

```text
AxisNControlGroup
├── AxisNCommandPanel
│   ├── btnAxisNAlarmReset
│   ├── btnAxisNHome
│   ├── btnAxisNJogNegative
│   ├── btnAxisNJogPositive
│   ├── btnAxisNRun
│   ├── btnAxisNStop
│   ├── txtAxisNCommandPos
│   └── txtAxisNCommandVel
└── AxisNHeaderActual
    ├── btnAxisNEnable
    ├── txtAxisNActualPos
    └── txtAxisNActualVel
```

Ghi chú:

- Axis2-Axis6 phải đồng bộ layout, style, kích thước với Axis1.
- Không sửa bằng Python.

## 10. Axis Enable button behavior

Các nút sau là nút 2 trạng thái:

```text
btnAxis1Enable
btnAxis2Enable
btnAxis3Enable
btnAxis4Enable
btnAxis5Enable
btnAxis6Enable
```

Quy ước:

- Servo OFF:
  - Text: `Enable`.
  - Màu xanh/gradient xanh.
  - `checked = False`.
- Servo ON:
  - Text: `Disable`.
  - Màu đỏ/gradient đỏ.
  - `checked = True`.

Ghi chú quan trọng:

- Trong hệ thống thật, nút không nên đổi trạng thái chỉ vì click.
- Nút phải đổi theo feedback thật `ServoOn`.
- Trong test mode được phép toggle/timer để kiểm tra UI.

Kích thước chuẩn:

```text
width = 70
height = 30
```

Nếu nằm trong layout:

- Set `minimumSize = 70x30`.
- Set `maximumSize = 70x30`.
- `sizePolicy = Fixed, Fixed`.

## 11. Robot Enable / Disable buttons

Hai nút robot-level:

```text
btnRobotEnable
btnRobotDisable
```

Yêu cầu style:

- Giống nhóm nút điều khiển `Run`, `Stop`, `Home`, `Jog`.
- Có gradient, border, hover, pressed, border-radius.
- Không dùng màu phẳng đơn giản.

Quy ước:

- `btnRobotEnable`: xanh gradient, text `Enable`.
- `btnRobotDisable`: đỏ gradient, text `Disable`.
- Kích thước: `70x30`.
- Không set kích thước trong Python.

## 12. Những gì không được làm trong `main_window.py`

Danh sách cấm:

```python
setGeometry()
move()
resize()
setFixedSize()
```

Không dùng Python để:

- Sửa layout.
- Add layout.
- Add widget vào layout trừ trường hợp đặc biệt như nhúng RViz vào container đã có layout sẵn.
- Chỉnh vị trí widget.

Ngoại lệ hợp lệ:

- Nhúng RViz vào `embeddedRvizWidget.layout()`.
- Cập nhật text button.
- Cập nhật màu LED/button bằng `setStyleSheet()`.
- Cập nhật giá trị line edit/label.
- Kết nối signal/slot.
- Xử lý ROS2 callback.

## 13. Checklist khi viết lại `main_window.py` trên Linux/ROS2

```text
[ ] Load file .ui đúng đường dẫn.
[ ] Không tạo lại layout đã có trong .ui.
[ ] Kết nối SideMenu với QStackedWidget.
[ ] Setup QButtonGroup cho SideMenu.
[ ] Setup RViz embedder cho embeddedRvizWidget.
[ ] Setup image preview bằng QLabel.setPixmap().
[ ] Tạo hàm set LED status bar.
[ ] Tạo hàm set motor status LED.
[ ] Tạo hàm set motor error LED.
[ ] Tạo hàm update_axis_servo_button(axis_id, servo_on).
[ ] Tạo hàm update_robot_enable_led(enabled).
[ ] Tắt led_test_mode khi dùng trạng thái thật.
[ ] Cập nhật UI từ ROS2 callback qua signal/slot an toàn.
[ ] Không update Qt widget trực tiếp từ thread ROS nếu khác thread GUI.
```

## 14. Gợi ý hàm cần có trong `main_window.py`

```python
setup_side_menu()
sync_side_menu_with_page(index)

set_status_led(led_widget, active)
set_motor_status_led(led_widget, active)
set_motor_error_led(led_widget, active)

update_connection_led(connected)
update_robot_enable_led(enabled)

update_axis_servo_button(axis_id, servo_on)
update_axis_status_leds(axis_id, status)

start_led_test_timer()
stop_led_test_timer()
update_led_test_cycle()

update_yolo_preview(qimage_or_pixmap)
start_rviz_embedder()
stop_rviz_embedder()
```

## 15. Kết luận kỹ thuật

File `.ui` hiện đã là nguồn chuẩn cho layout. `main_window.py` khi chuyển sang ROS2 chỉ cần nối logic, signal/slot, cập nhật text/màu/trạng thái và xử lý dữ liệu runtime.

Các trạng thái hiện đang test OK ở mức UI độc lập. Bước tiếp theo là map trạng thái thật từ ROS2/hardware vào các hàm update UI tương ứng.
