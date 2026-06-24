
## RUN

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

## Gọi GoHome

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=gohome
```

## Gọi MoveToPose

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=move_to_pose
```

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome "{start: true}" --feedback
```

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose "{target_pose: {position: {x: 0.4, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.1}" --feedback
```
```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose "{target_pose: {position: {x: 0.3, y: 0.0, z: 0.35}, orientation: {x: 1, y: 1, z: 0, w: 0}}, velocity_scale: 0.1}" --feedback
```
```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian "{target_pose: {position: {x: 0.3, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, velocity_scale: 0.1}" --feedback
```

```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.1}" --feedback


ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard "{step: 0.1, velocity_scale: 0.1}" --feedback
```

x  0.25 to 0.55
y -0.45 to 0.45
z  0.05 to 0.2
```bash
ros2 action send_goal /move_gripper  robot_task_manager/action/MoveGripper "{ position: 0.03 }" --feedback
```
```bash
ros2 interface show robot_task_manager/action/MoveGripper
ros2 action list -t | grep gripper
ros2 topic echo /joint_states
```

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace "{ pose_pick: { position: {x: 0.40, y: 0.10, z: 0.03}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0} }, pose_place: { position: {x: 0.30, y: -0.10, z: 0.1}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0} }, gripper: 0.025, velocity_scale: 0.2}" --feedback

```
# Active venvvenv
```bash
source ~/venvs/ros_env/bin/activate
```
```bash
python -m colcon build --packages-select perception
source install/setup.bash

head -n 1 install/perception/lib/perception/gui_pick_and_place

ros2 run perception gui_pick_and_place
```

ros2 run plotjuggler plotjuggler

ros2 run camera_calibration cameracalibrator \
  --size 5x8 \
  --square 0.029 \
  --no-service-check \
  image:=/camera/color/image_raw\
  camera:=/camera


  **** Calibrating ****
mono pinhole calibration...
D = [0.22245887424144464, -0.13444704391391848, 0.018063643854369534, 0.08957893266163468, 0.0]
K = [600.3701702803917, 0.0, 421.83263232383024, 0.0, 569.2100897091441, 219.84486646507202, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [578.0059204101562, 0.0, 463.0037228663823, 0.0, 0.0, 627.1709594726562, 225.14725147728313, 0.0, 0.0, 0.0, 1.0, 0.0]
None

# oST version 5.0 parameters

[image]

width
640

height
480

[narrow_stereo]

camera matrix
600.370170 0.000000 421.832632
0.000000 569.210090 219.844866
0.000000 0.000000 1.000000

distortion
0.222459 -0.134447 0.018064 0.089579 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
578.005920 0.000000 463.003723 0.000000
0.000000 627.170959 225.147251 0.000000
0.000000 0.000000 1.000000 0.000000

('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')
