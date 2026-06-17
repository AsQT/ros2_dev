# Gazebo DRL Bringup Report

Date: 2026-06-11

## Scope

Per the requested edit boundary, code changes were kept inside:

- `robot_drl`
- `robot_bringup`
- `robot_task_executor`

No files in `robot_description` or `robot_moveit` were modified.

## External Blocking Issue

Command used for the smoke test:

```bash
source /opt/ros/jazzy/setup.bash
source /home/minhquang/ros2_dev/install/setup.bash
PYTHONPATH=/home/minhquang/venvs/ros_rl/lib/python3.12/site-packages:$PYTHONPATH \
  ros2 launch robot_drl drl_gazebo.launch.py use_rviz:=false
```

Observed error:

```text
package 'ros_gz_sim' not found
```

Additional package check:

```text
ros_gz_sim: Package not found
ros_gz_bridge: Package not found
```

`ros_gz_sim` is referenced by `robot_description/launch/gazebo.launch.py`, and
`ros_gz_bridge` is needed by the same Gazebo stack for ROS/Gazebo topic bridges.
Gazebo cannot start until the ROS Gazebo integration packages are installed or
the environment is sourced from an installation that provides them.

Suggested check:

```bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

For ROS 2 Jazzy this is usually provided by the `ros-jazzy-ros-gz` family of
packages, but please confirm against the target machine package policy before
installing.

APT candidates found on this machine:

```text
ros-jazzy-ros-gz:        1.0.22-1noble.20260412.072535
ros-jazzy-ros-gz-sim:    1.0.22-1noble.20260412.044601
ros-jazzy-ros-gz-bridge: 1.0.22-1noble.20260412.043437
ros-jazzy-ros-gz-image:  1.0.22-1noble.20260412.051928
```

Recommended install command:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

Minimal install command for the current launch error:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

## Notes

The old `robot_bringup/sim.launch.py` path included
`robot_task_manager/launch/task_servers_sim.launch.py`, which emitted a warning
about a missing file under `robot_moveit/config/robot.urdf.xacro`. The new DRL
Gazebo path uses `robot_task_executor/launch/task_executor.launch.py` instead,
matching the working mock-hardware DRL flow and avoiding that task-manager path.
