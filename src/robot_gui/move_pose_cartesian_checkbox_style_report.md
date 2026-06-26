# Move Pose Cartesian checkbox style report

## File da sua

- `robot_gui/include/robot_gui/task_action_controller.hpp`
- `robot_gui/src/task_action_controller.cpp`

## ObjectName checkbox

- `chkMovePoseCartesian`

ObjectName da ton tai trong `robot_gui/ui/robot_gui.ui`, khong can doi ten va van duoc logic gui action su dung.

## Hieu ung mau

Unchecked:

- Text mau trung tinh `#333333`.
- Nen trong suot.
- Vien trong suot de khong tao cam giac Cartesian dang bat.
- Indicator vien xam nhe.

Checked:

- Text mau cyan dam `#005A70`.
- Font weight `600`.
- Nen cyan nhat `#D9F4F7`.
- Vien cyan `#01BABE`.
- Indicator doi sang cyan `#01BABE`.

Style duoc cap nhat khi khoi tao GUI va khi `chkMovePoseCartesian::toggled` thay doi, nen tick/bo tick se refresh truc tiep.

## Logic action

Khong doi logic gui action:

- Unchecked van goi `/move_to_pose`.
- Checked van goi `/move_to_pose_cartesian`.
- Khong doi layout GUI va khong anh huong cac tab action khac.

## Test

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_gui --symlink-install
```

Ket qua: build thanh cong.

Chay GUI:

```bash
QT_QPA_PLATFORM=offscreen ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=0
```

Ket qua: `robot_gui_node` khoi dong thanh cong qua buoc setup UI/controller va thoat sach sau SIGINT. Trong moi truong offscreen khong xac nhan mau bang mat duoc, nhung khong co loi stylesheet/runtime cho `chkMovePoseCartesian`.
