# pick_and_place_aruco

Node ROS 2 Python detect ArUco, đổi pose marker sang base frame, tạo pose pick theo yaw và gửi goal vào `/pickplace` với type `robot_task_manager/action/PickPlace`.

## Build

```bash
colcon build --packages-select robot_pick_and_place
source install/setup.bash
```

## Run

```bash
ros2 launch robot_pick_and_placearuco_pick_place.launch.py
```

Debug image:

```bash
rqt_image_view /aruco_pick_place/image_annotated
```

Computed pick pose:

```bash
ros2 topic echo /aruco_pick_place/pick_pose
```
