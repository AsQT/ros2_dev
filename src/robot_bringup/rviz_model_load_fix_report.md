# RViz Model Load Fix Report

## 1. Nguyen nhan loi

Khi chay `ros2 launch robot_bringup real.launch.py`, `ros2_control_node` nap hardware plugin `robot_hardware_interface/RobotSystemHardware`.

Truoc khi sua, trong `on_activate()`, neu TCP den robot `192.168.2.50:5000` ket noi that bai hoac timeout thi hardware tra ve `CallbackReturn::ERROR`. Controller manager vi vay khong activate duoc hardware, dan den controller khong len `active`, `/joint_states` khong duoc publish on dinh va RViz embedded khong co du lieu can thiet de hien thi robot model.

## 2. File da sua

- `robot_hardware_interface/src/tcp_system_hardware.cpp`

Khong sua layout GUI, khong sua launch file, khong cap nhat cac report cu.

## 3. Hardware activation behavior

Ham chiu trach nhiem chinh:

- `RobotSystemHardware::on_activate()`

Behavior moi:

- Neu TCP ket noi thanh cong: tiep tuc doc state dau tien tu robot, sync `cmd_pos_` theo `hw_pos_`, giu hanh vi dieu khien that.
- Neu TCP ket noi that bai hoac throw exception: hardware van activate thanh cong, set `connected_ = false`, publish trang thai disconnected, set joint state ve 0 va cho phep controller manager tiep tuc song.
- `on_activate()` khong con tra ve `CallbackReturn::ERROR` chi vi TCP timeout.

## 4. Behavior khi TCP khong ket noi duoc

Da sua cac duong lifecycle/read/write de che do offline khong lam sap `ros2_control_node`:

- `on_activate()` activate offline voi joint position/velocity bang 0.
- `read()` khi mat ket noi tra ve `hardware_interface::return_type::OK`, giu/default state de `joint_state_broadcaster` tiep tuc publish.
- `write()` khi mat ket noi tra ve `OK` va bo qua lenh gui TCP.
- Loi `get_all_state()` va loi send TCP duoc log warning throttle, khong tra `ERROR` lam controller manager dung.
- `/robot_hw/flags` publish default flags bang 0 khi khong co state that.

Muc tieu cua behavior nay la: robot that co the offline, nhung controller manager, `joint_state_broadcaster`, `/joint_states`, TF va RViz model van khoi dong duoc.

## 5. Behavior khi TCP ket noi duoc

Khi TCP available, hardware van dung duong that:

- Ket noi TCP den robot.
- Goi `CMD_GET_ALL` de lay joint state.
- Sync command position theo state hien tai de tranh giat ve 0.
- `arm_controller`, `gripper_controller` va `joint_state_broadcaster` van activate binh thuong.

Trong lan test hien tai, TCP den `192.168.2.50:5000` ket noi thanh cong nen da xac nhan duoc duong connected.

## 6. Test result

Da chay build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_hardware_interface robot_bringup robot_gui robot_moveit robot_control --event-handlers console_direct+
```

Ket qua:

- PASS: 5 packages finished.

Da chay launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup real.launch.py
```

Ket qua runtime:

- PASS: `/controller_manager` ton tai.
- PASS: RViz embedded process `/embedded_rviz` ton tai.
- PASS: `joint_state_broadcaster` active.
- PASS: `arm_controller` active.
- PASS: `gripper_controller` active.
- PASS: `/joint_states` co publisher va publish khoang 15 Hz.
- PASS: `ros2 topic echo --once /joint_states` nhan duoc 8 joint: `joint_1` den `joint_6`, `joint_gl`, `joint_gr`, position/velocity deu bang 0 trong lan test.
- PASS: `/tf_static` co transform `world -> base_link`.
- PASS: `robot_description` co tren graph va cac node MoveIt/robot_state_publisher nap robot model.

Ghi chu:

- Lan test tren may hien tai ket noi duoc TCP robot that, nen offline runtime chua duoc kich hoat bang launch that.
- Code path offline da duoc sua de bat khi `client_.connect()` tra false hoac throw exception.
- Co warning san co ve duplicate node name va stylesheet cua mot so nut GUI; cac warning nay khong thuoc loi RViz/model load va khong duoc sua trong task nay.
