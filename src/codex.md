Hiện tại demo RL pick_place trong Gazebo đã chạy ổn, không được thay đổi nhiều logic pipeline. 
Chỉ chỉnh lại phần random vị trí của wood, box obstacle, start pose và place pose để vật cản thật sự nằm trên quỹ đạo dự kiến, giúp thể hiện rõ RL né vật cản.

Mục tiêu chính:
- Box là vật cản.
- Wood là vật cần gắp.
- Vị trí box hiện tại gần như không nằm trên đường đi nên demo chưa thể hiện rõ khả năng tránh vật cản.
- Cần random lại sao cho đường đi từ start -> wood -> place có xu hướng đi qua vùng vật cản.
- Không chỉnh reward, model RL, planner chính, task manager logic chính nếu không cần.
- Không phá các launch/demo hiện tại đã chạy ổn.

Yêu cầu chỉnh random:

## 1. Quy ước vùng random theo trục Y

Random theo nguyên tắc chia hai phía trái/phải qua trục y = 0:

```text
box obstacle: nằm gần vùng y = 0
wood object: nằm ở một phía, y ≈ +0.10 m hoặc y ≈ -0.10 m
start pose: nằm phía ngược dấu Y với wood
place pose: nằm phía ngược dấu Y với wood

Ví dụ:

Case A:
wood_y  = +0.10 m
start_y = -0.10 m
place_y = -0.10 m
box_y   ≈ 0.00 m

Case B:
wood_y  = -0.10 m
start_y = +0.10 m
place_y = +0.10 m
box_y   ≈ 0.00 m

Như vậy quỹ đạo từ start đến wood, hoặc từ wood đến place, bắt buộc có xu hướng cắt qua vùng y = 0, nơi đặt box obstacle.

2. Random wood

Wood là vật cần gắp.

Đề xuất:

wood_y_sign = random choice [-1, +1]
wood_y = wood_y_sign * random trong khoảng 0.08 đến 0.13 m

Có thể random nhẹ theo X nhưng vẫn giữ trong vùng robot gắp được:

wood_x = random trong khoảng hợp lệ hiện tại của demo
wood_y = ±(0.08 -> 0.13)
wood_z = giữ logic Z hiện tại đang chạy ổn

Không làm thay đổi logic xác định wood là object cần gắp.

3. Random start pose

Start pose phải nằm phía ngược dấu Y so với wood.

start_y = -sign(wood_y) * random trong khoảng 0.08 đến 0.13 m

X của start có thể random nhẹ nhưng không được làm robot mất khả năng plan.

start_x = random trong khoảng hợp lệ hiện tại
start_z / orientation giữ như logic hiện tại nếu đang ổn

Nếu hiện tại demo có fixed start pose, chỉ chỉnh thành random Y theo nguyên tắc trên, không thay đổi quá nhiều phần khác.

4. Random place pose

Place pose cũng phải nằm phía ngược dấu Y so với wood.

place_y = -sign(wood_y) * random trong khoảng 0.08 đến 0.15 m

Place pose có thể random X nhẹ để demo đa dạng hơn:

place_x = random trong khoảng hợp lệ hiện tại
place_z = giữ logic hiện tại đang chạy ổn

Đảm bảo place pose không trùng quá gần với start pose nếu demo cần phân biệt rõ đường đi.

5. Random box obstacle

Box là vật cản, đặt gần vùng y = 0 để nằm giữa hai phía của quỹ đạo.

box_y = random trong khoảng -0.025 đến +0.025 m

X của box nên nằm giữa vùng đường đi từ start/wood/place. Có thể lấy gần trung điểm theo X giữa wood và place hoặc random quanh vùng trung tâm thao tác hiện tại:

box_x = midpoint(wood_x, place_x) + random noise nhỏ

hoặc nếu demo hiện tại dùng vùng thao tác cố định thì:

box_x = random trong khoảng hợp lệ nằm giữa vùng pick/place hiện tại

Điều quan trọng:

Box phải nằm gần đoạn nối giữa wood và place.
Không được đặt box trùng trực tiếp với wood hoặc place.
Phải có khoảng hở tối thiểu để robot vẫn có khả năng tránh.

Thêm kiểm tra khoảng cách tối thiểu:

distance_xy(box, wood)  >= 0.06 m
distance_xy(box, place) >= 0.06 m
distance_xy(box, start) >= 0.06 m

Nếu không đạt thì random lại.

6. Tăng chiều cao box

Hiện tại box chưa đủ nổi bật. Cần tăng chiều cao để vật cản rõ ràng hơn trong Gazebo và trong quỹ đạo né.

Đề xuất:

box_height = random trong khoảng 0.10 đến 0.15 m

Nếu đang random size 5 đến 15 cm thì giữ logic đó nhưng ưu tiên height lớn hơn:

box_size_x = random 0.05 đến 0.12 m
box_size_y = random 0.05 đến 0.12 m
box_size_z = random 0.10 đến 0.15 m

Z của box phải được tính đúng theo mặt bàn:

box_z = table_z + box_size_z / 2

Không để box bị lơ lửng hoặc chìm xuống bàn.

7. Không thay đổi quá nhiều logic

Chỉ được chỉnh các phần liên quan đến:

- random wood pose
- random obstacle/box pose
- random box size nếu cần
- random start pose
- random place pose
- ground-truth output tương ứng nếu các pose này đang được truyền vào task/RL

Không được tự ý chỉnh:

- RL model
- reward
- observation format nếu không bắt buộc
- action interface
- MoveIt/action logic chính
- gripper logic
- Gazebo launch tổng thể đã chạy ổn

Nếu cần cập nhật observation do pose thay đổi thì chỉ cập nhật đúng giá trị pose mới sinh ra, không đổi format observation.

8. Log bắt buộc để dễ kiểm tra

Khi chạy demo, in rõ ra log các giá trị:

wood_pose
box_pose
box_size
start_pose
place_pose
wood_y_sign

Và in thêm kiểm tra logic:

sign(start_y) == -sign(wood_y)
sign(place_y) == -sign(wood_y)
abs(box_y) <= 0.025

Nếu check sai thì báo lỗi và random lại.

9. Tiêu chí nghiệm thu

Sau khi chỉnh, phải chạy lại demo pick_place RL trong Gazebo và báo cáo:

- Demo có spawn wood, box, start, place đúng quy tắc trái dấu Y hay không.
- Box có nằm gần vùng y = 0 hay không.
- Quỹ đạo nominal từ wood đến place có đi qua vùng gần box hay không.
- Robot/RL có thực hiện đường tránh vật cản rõ hơn trước hay không.
- Có phát sinh lỗi spawn, lỗi Z, lỗi gripper, lỗi planning/execution không.

Chỉ báo cáo kết quả sau khi đã test bằng launch demo hiện tại.

Tóm lại:
Không viết lại pipeline. Không refactor lớn.
Chỉ chỉnh random scene để tạo tình huống obstacle thật sự nằm trên đường đi: wood ở một phía Y, start/place ở phía ngược lại, box ở giữa gần y = 0 và cao hơn.