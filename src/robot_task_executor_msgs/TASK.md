# robot_task_executor_msgs Tasks

## Purpose

This task file tracks work specific to the `robot_task_executor_msgs` package. This package defines all service types consumed by `robot_task_executor` and its clients (`robot_gui`, `robot_drl`, `robot_vision_pipeline`). Service definitions must be stable and consistent across all packages.

## High Priority

- [ ] **TODO: verify** — All 9 service definitions exist and have correct request/response fields: `MoveToNamedTarget`, `MoveToJointTarget`, `MoveToPoseTarget`, `MoveToNamedPoseTarget`, `MoveToCartesianTarget`, `MoveToNamedCartesianTarget`, `MoveCartesianSequence`, `MoveSequence`, `MoveCartesianPoseSequence`.
- [ ] **TODO: verify** — Service field types match the C++ server implementation in `robot_task_executor`: e.g., `MoveToCartesianTarget` has `x`, `y`, `z` (double), `frame_id` (string), `execute` (bool), and response has `success` (bool), `fraction` (double), `message` (string).
- [ ] **TODO: verify** — `MoveCartesianPoseSequence` request has a `PoseStamped[]` array named `poses` and response has `fraction` (double).
- [ ] **TODO: verify** — Generated C++ and Python bindings are produced by `rosidl_default_generators` during the build.
- [ ] **TODO: verify** — All clients (`robot_gui`, `robot_drl`, `robot_vision_pipeline`) import the generated service types correctly.

## Medium Priority

- [ ] **TODO: verify** — The `package.xml` correctly declares `rosidl_interface_packages` member group for automatic type discovery.
- [ ] **TODO: verify** — The package builds with `colcon build --packages-select robot_task_executor_msgs` without errors.
- [ ] **TODO: verify** — No additional service definitions are needed for planned future features.

## Low Priority

- [ ] Add an action definition (`*.action`) for asynchronous trajectory execution with feedback.
- [ ] Add a `GetAvailableTargets` service that returns the list of available named targets from the YAML config.

## Debugging Tasks

- [ ] If service type not found: verify the package was built and sourced before building dependent packages.
- [ ] If Python import fails: verify `rosidl_default_runtime` is in `exec_depend` and `setup.py` is updated.
- [ ] If C++ service client fails to compile: verify the generated headers are in the include path.

## Documentation Tasks

- [ ] Document all 9 service definitions with field descriptions in the package README.
- [ ] Document the version history of service definitions (any change is a breaking API change).
- [ ] Document how to add a new service definition.

## TODO Verify

- [ ] **TODO: verify** — No service definitions are missing that are referenced in other packages.
- [ ] **TODO: verify** — `package.xml` license (`TODO`) is updated.
