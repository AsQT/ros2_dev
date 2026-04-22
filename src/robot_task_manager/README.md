
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
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose "{target_pose: {position: {x: 0.4, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5}" --feedback
```
```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose "{target_pose: {position: {x: 0.3, y: 0.0, z: 0.35}, orientation: {x: 1, y: 1, z: 0, w: 0}}, velocity_scale: 0.5}" --feedback
```
```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian "{target_pose: {position: {x: 0.3, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5}" --feedback
```

```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5}" --feedback


ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard "{step: 0.01, velocity_scale: 0.5}" --feedback
```

x  0.25 to 0.55
y -0.45 to 0.45
z  0.05 to 0.2