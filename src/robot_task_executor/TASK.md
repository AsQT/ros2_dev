# robot_task_executor Tasks

## Purpose

This task file tracks work specific to the `robot_task_executor` package. This package provides a C++ ROS 2 node that acts as a high-level service-based bridge to MoveIt 2. It accepts named, joint, pose, and Cartesian targets and produces planned trajectories or executes them directly.

## High Priority

- [ ] **TODO: verify** — All 9 service handlers (`/move_to_named_target`, `/move_to_joint_target`, `/move_to_pose_target`, `/move_to_named_pose_target`, `/move_to_cartesian_target`, `/move_to_named_cartesian_target`, `/move_cartesian_sequence`, `/move_sequence`, `/move_cartesian_pose_sequence`) respond correctly in simulation.
- [ ] **TODO: verify** — The `move_cartesian_pose_sequence` service correctly handles DRL arbitrary pose sequences and computes valid MoveIt Cartesian paths.
- [ ] **TODO: verify** — `joint_waypoints.yaml` loads correctly (degrees → radians conversion) and all 6 waypoints (`start`, `origin`, `P1`, `P2`, `P3`, `P4`) are available.
- [ ] **TODO: verify** — `pose_waypoints.yaml` loads correctly and all poses are in the expected frame (`base_link`).
- [ ] **TODO: verify** — `cartesian_points.yaml` loads correctly and all points use `base_link` as `frame_id`.
- [ ] **TODO: verify** — The `base_frame` and `ee_link` parameters (`base_link`, `tcp_link`) are correctly configured for the robot.

## Medium Priority

- [ ] **TODO: verify** — The TF2 transform in `handle_cartesian_target` and `handle_pose_target` correctly transforms poses between frames.
- [ ] **TODO: verify** — The fixed Cartesian orientation (RPY=π, 0, 0) is consistently used in `cartesian_quat()` across all Cartesian services.
- [ ] **TODO: verify** — The `visualization_utils` publishes trajectory markers to the correct topic for RViz display.
- [ ] **TODO: verify** — `yaml_cpp_vendor` dependency is correctly built and linked.
- [ ] **TODO: verify** — The node builds with C++17 and all include paths resolve correctly.

## Low Priority

- [ ] Add a service to query available named targets and their descriptions.
- [ ] Add trajectory execution feedback (publish current trajectory progress as a topic).
- [ ] Add retry logic for failed trajectory execution.
- [ ] Add support for cartesian orientation control (currently fixed at RPY=π, 0, 0).

## Debugging Tasks

- [ ] If `/move_to_named_target` fails with "target not found": verify service call format uses plain string, not nested message.
- [ ] If Cartesian path fraction < 0.95: check that the target pose is within reach and not in a singular configuration.
- [ ] If TF transform fails: verify `tf2_ros` buffer is populated before the service call.
- [ ] If trajectory visualization is not visible in RViz: check the marker topic name matches RViz subscription.

## Documentation Tasks

- [ ] Document all 9 service types with example service call commands.
- [ ] Document the YAML config format for `joint_waypoints.yaml`, `pose_waypoints.yaml`, and `cartesian_points.yaml`.
- [ ] Document the difference between pose targets (orientation included) and Cartesian targets (position only, fixed orientation).
- [ ] Document the `eef_step` and `jump_threshold` parameters for Cartesian planning.

## TODO Verify

- [ ] **TODO: verify** — The `planner_utils.cpp` implementation handles `CARTESIAN_SUCCESS_THRESHOLD = 0.95` correctly.
- [ ] **TODO: verify** — The `waypoint_loader.cpp` correctly handles YAML list format for joint positions (degrees → radians).
- [ ] **TODO: verify** — The `transform_utils.cpp` handles missing transforms gracefully (returns error, does not crash).
- [ ] **TODO: verify** — The `visualization_utils` publishes markers in `base_link` frame so they align with the robot model.
- [ ] **TODO: verify** — `package.xml` license (`TODO`) is updated to Apache-2.0.
