# Plan-only audit

## Implemented

- All public action definitions now include `bool execute`.
- Single-step MoveIt and gripper actions perform real planning and skip MoveIt execution when `execute=false`.
- `DrlPickPlace` calls `/drl/plan` and waits for a planned trajectory in plan-only mode, but does not call `/drl/execute_forward`.
- Composite actions pass `execute=false` to sub-actions in plan-only mode, so they do not command robot or gripper motion.
- `RepeatabilityTest` supports `AXIS_Z=2`, validates `axis`, `meas_offset`, `repeat_count`, and `velocity_scale`, and uses `fast_velocity_scale` for non-measurement moves.

## Known limitation

`PickPlace` and `RepeatabilityTest` are composed from child actions whose result interfaces do not return a planned end state. In plan-only mode the server validates every planned segment and never executes motion, but later segments are still planned from the current robot state visible to MoveIt, not from the previous segment's planned terminal state.

Supporting fully staged composite planning would require either returning planned terminal robot state from the child actions or moving composite planning into a shared executor that can seed each next plan from the previous planned end state.
