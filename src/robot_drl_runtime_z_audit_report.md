# robot_drl Runtime Z Audit Report

Date: 2026-06-24

## 1. Scope

- Task type: audit/inspection only.
- No source code was edited.
- No reward was added.
- Observation, action space, API, services, and config were not changed.
- Only this report file was created.

## 2. Training reference from `struc.txt`

`struc.txt` describes the training project as FRAME_ONLY, not robot-joint, MoveIt, Gazebo, or ROS runtime control.

Key reference points used for comparison:

| Item | Training reference |
|---|---|
| observation_dim | 15 |
| observation order | `[current_pos, target_pos, target-current, rel_obs, obs_size]` |
| obs[0:3] | current_pos, raw meters |
| obs[3:6] | target_pos, raw meters |
| obs[6:9] | `target_pos - current_pos`, raw meters |
| obs[9:12] | `(obstacle_center - current_pos) / workspace_range` |
| obs[12:15] | `obstacle_half_extent / workspace_range` |
| action_dim | 3 |
| action meaning | normalized Cartesian delta in `[-1, 1]` |
| action scale | `delta = action * 0.01 m` |
| update rule | `next_pos = current_pos + delta` |
| deterministic eval | `deterministic=True` by default |
| VecNormalize | not used |
| reward | no Z guard, no path curvature reward |
| target_z behavior | random target Z noted as hard-coded to `0.10` in training |
| workspace/start | start random roughly `[0.25,-0.15,0.10]` to `[0.5,0.15,0.30]`; workspace range used for obstacle normalization is `0.5` |

## 3. `robot_drl` runtime flow

Runtime flow found in code:

```text
manual/vision/DrlPickPlace input
  -> DrlUnifiedPlannerNode builds DrlSceneInput
  -> PlanningScene/manual obstacle selection
  -> DrlTrajectoryPlannerCore.compute_trajectory()
  -> build_observation_15d()
  -> optional normalize_if_needed()
  -> model.predict(... deterministic=True)
  -> delta = action * action_step
  -> next_tcp_drl = current_tcp_drl + delta
  -> optional obstacle safety filter and workspace clip
  -> publish PoseArray
  -> /move_cartesian_pose_sequence
  -> MoveIt computeCartesianPath, or PTP fallback if Cartesian fraction is low
```

Important files/functions:

| Step | File/function | Notes |
|---|---|---|
| model/VecNormalize/config load | `robot_drl/robot_drl/drl_planner_core.py:852`, `load_planner()` | model path is under `share/robot_drl/models/...`; VecNormalize is attempted if file exists |
| observation build | `robot_drl/robot_drl/state_builder.py:26`, `build_observation_15d()` | shape 15, order matches training |
| inference mode | `robot_drl/robot_drl/model_loader.py:120`, `predict()` | calls `model.predict(obs, deterministic=True)` |
| action scale/update | `robot_drl/robot_drl/drl_planner_core.py:713` | `delta = action * self._action_step`; `next_tcp_drl = current_tcp_drl + delta` |
| safety filter | `robot_drl/robot_drl/drl_planner_core.py:721` | can replace model delta for obstacle clearance |
| workspace clamp | `robot_drl/robot_drl/drl_planner_core.py:736` | clips waypoint to workspace min/max |
| append exact target | `robot_drl/robot_drl/drl_planner_core.py:802` | final waypoint may be appended if safe |
| path publish/store | `robot_drl/robot_drl/drl_planner_node_base.py:552` | publishes forward/backward PoseArray and markers |
| pre-execute validation | `robot_drl/robot_drl/drl_planner_node_base.py:832` | finite waypoint and obstacle validation only |
| execute DRL path | `robot_drl/robot_drl/drl_planner_node_base.py:889` | sends waypoints to `/move_cartesian_pose_sequence` |
| MoveIt executed path | `robot_task_executor/src/task_executor_node.cpp:907` and `robot_task_executor/src/planner_utils.cpp:251` | `computeCartesianPath`; PTP fallback if fraction below threshold |
| pick-place target source | `robot_task_manager/src/drl_pickplace_server.cpp:677` and `:732` | pre-pick adds approach height; place uses target_place directly |

Reward is not used in `robot_drl` runtime for control. The runtime is inference plus safety/filter/MoveIt execution.

## 4. Test/training vs ROS runtime comparison

| Item | Training/Test | `robot_drl` runtime | Match? | Risk/Note |
|---|---|---|---|---|
| model path | SAC FRAME_ONLY run from `struc.txt` | default constant is `run1/model/best_model.zip`; launch doc mentions `run/model/best_model.zip`; actual CLI can override `--model` | No / unclear | High risk of loading stale/wrong model unless launch command confirms `--model` |
| algorithm | SAC in `struc.txt` | loader supports SAC/DDPG/TD3 by metadata | Unclear | Need startup log `[model_loader] Loading ...` |
| deterministic | `True` | `model.predict(... deterministic=True)` | Yes | Low risk |
| VecNormalize | disabled | tries default `run1/model/vec_normalize_stats.pkl` if present | Unclear | If stats load with `norm_obs=True`, runtime would normalize unlike `struc.txt`; if load fails, runtime uses raw obs |
| observation_dim | 15 | enforced as 15 | Yes | Low risk |
| observation order | current, target, err, rel_obs, obs_size | same in `build_observation_15d()` | Yes | Low risk |
| unit | meters | code and parameters use meters | Yes | Low direct risk |
| frame | FRAME_ONLY coordinates | `base_link` passed through `FRAME_Z_OFFSET=0.0` | Mostly | Frame transform is identity for Z now; risk depends on upstream target frame correctness |
| target_z | hard-code note around 0.10 | manual/vision/DrlPickPlace can pass arbitrary target Z; place uses true target_place | No / risky | High risk when target Z is outside training distribution or not approach pose |
| action_scale | 0.01 m/step | `action_step` default 0.01; launch workspace uses same | Yes | Low risk unless env config override differs |
| Z sign | `next_z = current_z + action_z * scale` | same add rule | Yes | No sign inversion found |
| policy_path | direct rollout waypoints | stored as `trajectory_forward_base` | Yes, with safety filter | Need min-z log; not currently summarized |
| processed_path | none in `robot_drl` | no spline/smoothing in `robot_drl`; straight segments passed to MoveIt | Mostly | MoveIt computes robot trajectory after waypoints |
| executed_path | environment step in test | `/move_cartesian_pose_sequence` -> MoveIt `computeCartesianPath`, PTP fallback | Different | High risk if MoveIt/PTP fallback path dips in Z |
| runtime validator | training env collision/workspace | obstacle/IK validation exists, but no explicit Z guard, max step, sharp turn validator | Partial | High risk for Z undershoot specifically |

## 5. Observation build audit

| Observation index | Training meaning | `robot_drl` runtime meaning | Match? | Risk |
|---|---|---|---|---|
| 0-2 | current_pos | `tcp` in WORLD/DRL frame from `current_tcp_drl` | Yes | Low |
| 3-5 | target_pos | `target_drl` | Yes | Low |
| 6-8 | target-current | `error = tgt - tcp` | Yes | Low |
| 9-11 | rel_obs normalized | `(oc - tcp) / workspace_range` | Yes | Low |
| 12-14 | obs_size half extent normalized | `obstacle_half_extent / workspace_range`; full size divided by 2 before call | Yes | Low |

No index swap or Z sign inversion was found in the observation builder. Obstacle size is correctly converted from full size to half extent before observation.

Open checks:

- Confirm actual startup log for `workspace_range`, model path, and VecNormalize state.
- Confirm upstream vision/DrlPickPlace target is in `base_link` meters and not camera/mm.

## 6. Action scale and update rule

Runtime matches the training rule:

```python
delta = action * action_step
next_tcp_drl = current_tcp_drl + delta
```

Source: `robot_drl/robot_drl/drl_planner_core.py:713-718`.

No `next_z = current_z - action_z * scale` pattern was found. Action clipping is not explicitly done in runtime, but SB3 policy output should already respect the Box action space for SAC/DDPG/TD3 predict. The safety filter can replace the model delta and workspace clamp can change the final waypoint.

Interpretation logic:

- If `raw_action_z` differs between test and ROS for the same raw obs, suspect model path, VecNormalize, target, deterministic setting, or observation content.
- If `raw_action_z` matches but `next_z` differs, suspect safety filter, workspace clamp, frame conversion, or downstream MoveIt execution.
- If `trajectory_forward_base` does not dip but executed robot path dips, suspect `/move_cartesian_pose_sequence`, `computeCartesianPath`, or PTP fallback.

## 7. Target Z audit

High-risk finding: DrlPickPlace does not use approach height symmetrically.

- Pick: `pre_pick = target_pick`, then `pre_pick.position.z += pick_approach_height_m_`; DRL plans to pre-pick.
- Place: `call_drl_plan_and_execute(target_place.pose, false, ...)`; DRL plans directly to target_place, not pre-place.

Source: `robot_task_manager/src/drl_pickplace_server.cpp:677-692` and `:732-734`.

This matters because `struc.txt` says training target Z had a hard-code note around `0.10`. Runtime DrlPickPlace may pass target place Z such as the true object/place surface height. That can be outside the training distribution or lower than a safe approach pose. It can make ROS runtime look unlike the model test, even when observation order and action scale are correct.

Recommendation: in the next task, log `target_pick.z`, `pre_pick.z`, `target_place.z`, expected `pre_place.z`, and final `manual_default_target.z` sent to `robot_drl`.

## 8. Path stage comparison

Current code has three practical path stages:

| Stage | Where | Can verify now? | Notes |
|---|---|---|---|
| policy path | `trajectory_forward_base` in `DrlTrajectoryPlannerCore` | Partially | First 5 steps logged with action and TCP; no min-z summary |
| processed path | `robot_drl` PoseArray sent to executor | Mostly same as policy path | No smoothing/spline/resample found in `robot_drl` |
| executed path | MoveIt trajectory from `/move_cartesian_pose_sequence` | Not from current logs | `computeCartesianPath` creates joint trajectory; PTP fallback can replace Cartesian path |

Metrics not currently logged as summary:

```text
policy_path_min_z
processed_path_min_z
executed_path_min_z
target_z
z_min_allowed = target_z - margin
min_z_index
```

Recommended log locations:

- After `compute_trajectory()` returns, before validation: `robot_drl/robot_drl/drl_unified_planner_node.py:1054-1061`.
- Before sending poses to executor: `robot_drl/robot_drl/drl_planner_node_base.py:959-970`.
- Inside MoveIt service after `computeCartesianPath`: `robot_task_executor/src/task_executor_node.cpp:1073-1088` and `robot_task_executor/src/planner_utils.cpp:282-292`.

## 9. Sharp turn/path smoothness audit

No built-in max turn angle, max step distance, duplicate waypoint, or segment-too-short audit was found in `robot_drl`.

Available data:

- `trajectory_forward_base` contains all DRL waypoints.
- The executor logs first/last and sampled waypoints, but not turn angle or max step.

Recommended metrics to log in a future task:

```text
max_turn_angle_deg
sharp_turn_index
max_step_distance
max_step_index
duplicate_waypoint_count
short_segment_count
```

Best insertion point: immediately after `result = self._planner.compute_trajectory(...)` in `robot_drl/robot_drl/drl_unified_planner_node.py:1054-1061`.

## 10. Runtime safety audit

Existing validators:

- Workspace validation for start/target input: `DrlTrajectoryPlannerCore._validate_workspace_position()`.
- Workspace clipping during rollout: `_clip_to_workspace()`.
- Obstacle safety filter during rollout: `_safety_filter_delta_base()`.
- Cartesian obstacle validation before publish/execute: `validate_cartesian_path_against_obstacles()`.
- MoveIt IK/collision validation before publish/execute: `_validate_path_with_moveit()`.
- Pre-execute obstacle validation: `_check_execute_ready()`.

Missing for this Z issue:

- No explicit `waypoint.z >= target_z - margin` guard.
- No explicit `policy_path_min_z`/`executed_path_min_z` comparison.
- No max step distance validator.
- No sharp turn angle validator.
- No validation of MoveIt-generated executed Cartesian samples for Z undershoot.

Conclusion: runtime has collision/IK safety, but not a Z-guard validator. For the reported symptom, this is a high runtime risk.

## 11. Findings

### High risk

1. Runtime model path is ambiguous. `config.DEFAULT_MODEL_NAME` points to `run1/model/best_model.zip`, while launch comments mention `run/model/best_model.zip`, and `robot_drl/models/run/model/best_model.zip` was modified on 2026-06-24. This can explain test/runtime mismatch if the test used a different checkpoint.
2. VecNormalize handling is ambiguous. Training reference says no VecNormalize, but runtime attempts to load `run1/model/vec_normalize_stats.pkl` if present. If loaded with `norm_obs=True`, ROS inference input differs from training/test.
3. DrlPickPlace plans to `pre_pick` but plans directly to `target_place`; no `pre_place.z = target_place.z + approach_height` was found.
4. There is no explicit Z undershoot validator. Collision/IK checks can pass even if the path dips below `target_z - margin`.
5. Executed path can differ from DRL waypoints. `/move_cartesian_pose_sequence` uses MoveIt `computeCartesianPath`, and if the Cartesian fraction is low it falls back to PTP waypoint execution.

### Medium risk

1. Safety filter can alter the policy delta, so ROS path is not always raw policy path.
2. Workspace clipping can change waypoint Z near workspace bounds.
3. The runtime accepts target Z from manual, vision, or DrlPickPlace without checking whether it matches training distribution.
4. No sharp-turn or max-step audit is currently logged.

### Low risk

1. Observation order matches `struc.txt`.
2. Action scale/update rule matches `struc.txt`.
3. `model.predict()` uses `deterministic=True`.
4. No Z sign inversion was found in `robot_drl`.
5. No spline/cubic smoothing code was found in `robot_drl` itself.

## 12. Recommendations for next step

Recommended logging only, before changing behavior:

- Startup: log exact model path, detected algorithm, VecNormalize loaded/not loaded, `norm_obs`, `action_step`, workspace min/max.
- Per rollout first/last and summary: raw obs[0:15], model obs[0:15], raw action, delta, next pos, safety-adjusted flag.
- Path summary: `policy_path_min_z`, target Z, min index, max step distance, max turn angle.
- Executor summary: poses sent to `/move_cartesian_pose_sequence`, MoveIt `computeCartesianPath` fraction, whether PTP fallback was used, executed trajectory min TCP Z if available.
- DrlPickPlace target summary: `target_pick`, `pre_pick`, `target_place`, expected `pre_place`, and which target was sent to DRL.

Do not add reward until the runtime mismatch is pinned down. The most likely next diagnostic split is:

```text
same raw observation + same model + deterministic=True
  -> compare raw_action_z in standalone test vs ROS
  -> compare policy_path_min_z vs executed_path_min_z
```

## 13. Commands run

```bash
sed -n '1,240p' codex.md
sed -n '241,520p' codex.md
sed -n '1,260p' struc.txt
find robot_drl -maxdepth 3 -type f | sort
rg -n "predict|deterministic|VecNormalize|normalize|observation|obs|action|scale|0\.01|target|waypoint|interpol|smooth|validate|z|model|best_model|current_pos|workspace|obstacle" robot_drl robot_task_manager -S
nl -ba robot_drl/robot_drl/drl_planner_core.py
nl -ba robot_drl/robot_drl/state_builder.py
nl -ba robot_drl/robot_drl/model_loader.py
nl -ba robot_drl/robot_drl/config.py
nl -ba robot_drl/robot_drl/drl_unified_planner_node.py
nl -ba robot_drl/robot_drl/drl_planner_node_base.py
nl -ba robot_drl/robot_drl/planning_scene_adapter.py
nl -ba robot_task_manager/src/drl_pickplace_server.cpp
nl -ba robot_task_executor/src/task_executor_node.cpp
nl -ba robot_task_executor/src/planner_utils.cpp
nl -ba robot_task_executor/include/robot_task_executor/planner_utils.h
ls -lh robot_drl/models/run*/model/*.zip robot_drl/models/run*/model/*normalize*
colcon build --packages-select robot_drl
colcon test --packages-select robot_drl
colcon test-result --verbose
git status --short
```

Results:

- `colcon build --packages-select robot_drl`: passed, 1 package finished.
- `colcon test --packages-select robot_drl`: no tests ran; command exited with code 5 and printed `NO TESTS RAN`.
- `colcon test-result --verbose`: summary reported `0 tests, 0 errors, 0 failures, 0 skipped`.
- Attempting to inspect `run1/model/vec_normalize_stats.pkl` with system Python failed with a numpy pickle BitGenerator compatibility error; runtime catches VecNormalize load failures and falls back to raw observations, but actual startup log should still be checked.

