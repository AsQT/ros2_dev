Bổ sung thêm các yêu cầu sau cho demo RL pick_place trong Gazebo.

## 1. Thêm ràng buộc wood không được sát mép vùng thao tác / mép bàn

Hiện tại nếu wood random quá sát mép thì RL có thể không plan ra được. 
Cần thêm ràng buộc vị trí wood cách mép ít nhất 5 cm theo cả 2 trục X và Y.

Quy ước kiểm tra khoảng cách mép phải tính cả kích thước thực của wood:

```text
wood_x_min_edge_distance = wood_x - wood_size_x / 2 - region_x_min
wood_x_max_edge_distance = region_x_max - (wood_x + wood_size_x / 2)

wood_y_min_edge_distance = wood_y - wood_size_y / 2 - region_y_min
wood_y_max_edge_distance = region_y_max - (wood_y + wood_size_y / 2)

Điều kiện bắt buộc:

wood_x_min_edge_distance >= 0.08
wood_x_max_edge_distance >= 0.08
wood_y_min_edge_distance >= 0.08
wood_y_max_edge_distance >= 0.08

Tương đương với điều kiện random hợp lệ:

region_x_min + 0.08 + wood_size_x / 2 <= wood_x <= region_x_max - 0.08 - wood_size_x / 2

region_y_min + 0.08 + wood_size_y / 2 <= wood_y <= region_y_max - 0.08 - wood_size_y / 2

Nếu random ra wood không đạt điều kiện này thì phải random lại.

Lưu ý:

region_x_min, region_x_max, region_y_min, region_y_max phải lấy theo vùng spawn hợp lệ hiện tại của demo hoặc theo mép bàn nếu code hiện tại đã có thông tin này.
Không hard-code bừa nếu trong code đã có biến workspace/table boundary.
Không làm thay đổi logic pick/place chính.
Không đổi kích thước wood nếu hiện tại đang đúng.
Chỉ thêm kiểm tra hợp lệ trước khi spawn / trước khi truyền pose vào task.

Log bắt buộc:

wood_pose
wood_size
region_x_min, region_x_max
region_y_min, region_y_max
wood_edge_distance_x_min
wood_edge_distance_x_max
wood_edge_distance_y_min
wood_edge_distance_y_max

Nếu không đạt thì in rõ:

Rejected wood pose: too close to edge

và random lại.

2. Giữ logic random trái dấu Y như yêu cầu trước

Vẫn phải giữ nguyên nguyên tắc:

wood_y nằm một phía: khoảng +0.08 đến +0.13 m hoặc -0.08 đến -0.13 m
start_y trái dấu với wood_y
place_y trái dấu với wood_y
box_y gần 0

Nhưng khi random wood_y phải đồng thời thỏa điều kiện cách mép 5 cm.

Nếu vùng spawn không đủ rộng để vừa thỏa wood_y = ±0.08 -> ±0.13 vừa cách mép 5 cm thì phải báo rõ trong log, không được im lặng clamp sai.

3. Kiểm tra nguyên nhân lúc mới khởi động demo chạy chậm

Khi mới launch demo Gazebo pick_place RL, hệ thống đang chạy khá chậm.
Codex cần kiểm tra nguyên nhân là do máy bị đơ, Gazebo/MoveIt khởi động nặng, hay do code đang có delay/sleep/wait service quá lâu.

Chỉ kiểm tra và báo cáo trước, không refactor lớn.

Cần audit các điểm sau:

- Thời gian khởi động Gazebo
- Thời gian spawn robot
- Thời gian spawn wood/box
- Thời gian load controller
- Thời gian MoveIt ready
- Thời gian load model RL
- Thời gian chờ TF
- Thời gian chờ service/action server
- Các sleep cố định trong launch/node
- Các vòng while wait không timeout rõ ràng
- Có đoạn nào đang retry quá nhiều hoặc polling quá dày không

Thêm log timestamp theo từng phase nếu cần, ví dụ:

[timing] gazebo ready: ... s
[timing] controllers ready: ... s
[timing] moveit ready: ... s
[timing] rl model loaded: ... s
[timing] scene spawned: ... s
[timing] first plan started: ... s
[timing] first execution started: ... s

Mục tiêu là xác định rõ:

- Chậm do máy/Gazebo startup nặng
- Hay chậm do code có delay cố định
- Hay chậm do đang wait service/action/TF
- Hay chậm do RL model load / planning

Nếu phát hiện delay cố định không cần thiết, chỉ đề xuất cách sửa trong báo cáo.
Chỉ sửa nếu đó là lỗi rõ ràng, nhỏ, an toàn, không ảnh hưởng pipeline.

4. Trong sim, scale vận tốc execute đặt bằng 1.0

Vì đây là demo trong Gazebo sim nên lúc execute quỹ đạo cần chạy nhanh hơn.
Set velocity scale cho execution trong demo sim là:

velocity_scale = 1.0

Áp dụng cho các action/client liên quan trong demo pick_place RL nếu hiện tại đang truyền scale thấp hơn.

Yêu cầu:

Chỉ áp dụng cho demo Gazebo sim này.
Không đổi default global của các action thật / hardware thật nếu có nguy cơ ảnh hưởng robot thật.
Không đổi logic planner lớn.
Nếu launch có parameter velocity_scale thì set mặc định của launch demo này thành 1.0.
Nếu node demo truyền goal có velocity_scale thì truyền 1.0.
Nếu có nhiều đoạn pick/pregrasp/place/retract dùng scale riêng, trong sim demo đều ưu tiên 1.0 trừ khi có đoạn bắt buộc phải chậm để tránh lỗi gripper.

Log khi execute:

execution_velocity_scale = 1.0
5. Tiêu chí nghiệm thu bổ sung

Sau khi chỉnh, chạy lại demo hiện tại và báo cáo:

- Wood có cách mép tối thiểu 5 cm theo cả X/Y không.
- Wood/start/place/box có giữ đúng quy tắc trái dấu Y và box gần y=0 không.
- Box có nằm gần đường đi nominal không.
- Velocity scale khi execute trong Gazebo có đúng 1.0 không.
- Thời gian từng phase khởi động là bao nhiêu.
- Kết luận chậm lúc mới khởi động là do máy/Gazebo hay do delay trong code.
- Có phát sinh lỗi plan, execute, spawn, Z, gripper không.

Không được refactor lớn.
Ưu tiên chỉnh nhỏ, đúng trọng tâm:

Random scene hợp lý hơn.
Wood không sát mép.
Sim execute nhanh với velocity_scale = 1.0.
Có timing log để biết startup chậm do đâu.