# robot_drl Tasks

## Purpose

This task file tracks work specific to the `robot_drl` package. The package provides the `drl_unified_planner_node` which handles all DRL inference (DDPG/SAC/TD3 policy), planning, and execution for the robot pick-and-place workflow.

Architecture: `drl_unified_planner_node` replaces the old 4-node stack (`drl_inference_node` + `drl_action_bridge_node` + `vision_trajectory_preview_node` + `manual_trajectory_test_node`).

---

## High Priority

- [ ] **TODO** — Verify `config.py` constants (`ACTION_STEP`, `FRAME_Z_OFFSET`, `OBS_DIM`, `ACTION_DIM`) are consistent with the training environment (`DRL_Pathplanning_trainning/config/environment.yaml`).
- [ ] **TODO** — Verify `state_builder.py` builds the 15D observation vector correctly and applies the Z-offset conversion.
- [ ] **TODO** — Verify `model_loader.py` loads the DDPG model with compatibility patches for cross-environment inference.
- [ ] **TODO** — Verify VecNormalize statistics are loaded correctly if present.

---

## Medium Priority

- [ ] **TODO** — Verify `main.launch.py` launches bringup + vision + unified planner in the correct order.
- [ ] **TODO** — Verify `mock_environment_node.py` publishes `/vision/target_position` and `/vision/box_detection` topics compatible with the unified planner in vision mode.
- [ ] **TODO** — Verify `best_model.zip` and `vec_normalize_stats.pkl` exist in `models/` and are compatible with Stable-Baselines3.

---

## Architecture Notes

The unified planner node (`drl_unified_planner_node`) is the single active node. It handles:
- Manual input mode: terminal prompt for target/obstacle
- Vision input mode: subscribes to `/vision/target_position`, `/vision/box`, etc.
- Planning: DRL policy inference via `drl_planner_core`
- Execution: calls `/move_cartesian_pose_sequence` on `task_executor`

---

## Debugging Tasks

- If the model fails to load: verify `numpy._core` patch is applied and `custom_objects` are correct for the SB3 version.
- If the trajectory is offset from the robot: check `FRAME_Z_OFFSET = 0.330` is applied consistently.
- If the unified planner publishes no markers: verify TF is available (`base_link → tcp_link`) and the vision topic is publishing.
- If trajectory execution fails: check the service name matches (`/move_cartesian_pose_sequence`).

---

## Documentation Tasks

- [x] Document the complete DRL topology: unified planner handles both manual and vision modes.
- [x] Document the 15D observation vector layout and how the Z-offset is applied.
- [x] Document how to deploy a newly trained policy: copy `best_model.zip` and `vec_normalize_stats.pkl` to `models/`.
- [x] Document the VecNormalize handling.

---

## Completed Cleanup

- [x] Removed `trajectory_test_node`, `manual_trajectory_test_node`, `vision_trajectory_preview_node`, `drl_inference_node`, `drl_action_bridge_node`, `node.py` (dead nodes).
- [x] Removed `trajectory_test.launch.py`, `trajectory_test_no_gazebo.launch.py`, `drl_inference.launch.py`, `drl_action_bridge.launch.py` (isolated launch files).
- [x] Removed legacy visualization topics (`/drl_trajectory_marker`, `/drl_trajectory_poses`, `/visualization/target_marker`, `/visualization/box_marker`).
- [x] Unified around `drl_unified_planner_node`.
- [x] Rewrote `main.launch.py` to launch unified planner + robot bringup + vision.
