#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>

namespace robot_task_manager
{

// codex.md §3.5: single source of truth for the log root, derived from $HOME
// instead of a hardcoded absolute path so it works for any user/machine.
inline std::string logHomeDir()
{
  const char * home = std::getenv("HOME");
  return (home && *home) ? std::string(home) : std::string("/home/minhquang");
}

inline std::string defaultLogRootDir()
{
  return (std::filesystem::path(logHomeDir()) / "ros2_dev" / "Log_robot_data").string();
}

// Kept as a symbol for existing declare_parameter(..., kDefaultLogRootDir)
// call sites; now an absolute, HOME-derived string.
inline const std::string kDefaultLogRootDir = defaultLogRootDir();

inline std::string absoluteLogPath(const std::string & path)
{
  std::filesystem::path p(path.empty() ? defaultLogRootDir() : path);
  if (p.is_relative()) {
    // codex.md §3.4: NEVER resolve relative log paths against the current
    // working directory (that is what caused logs to land in
    // src/Log_robot_data). Resolve against ~/ros2_dev instead.
    p = std::filesystem::path(logHomeDir()) / "ros2_dev" / p;
  }
  return std::filesystem::absolute(p).lexically_normal().string();
}

inline std::string actionMetricsLogDir(const std::string & log_root_dir, const std::string & action_name)
{
  return (std::filesystem::path(absoluteLogPath(log_root_dir)) / "action_metrics" / action_name).string();
}

inline std::string canonicalLogActionName(const std::string & action_name)
{
  static const std::unordered_map<std::string, std::string> kNames{
    {"MovePoseRl", "move_pose_rl"},
    {"move_pose_rl", "move_pose_rl"},
    {"/move_pose_rl", "move_pose_rl"},
    {"DrlPickPlace", "pick_place_rl"},
    {"PickPlaceRL", "pick_place_rl"},
    {"pick_place_rl", "pick_place_rl"},
    {"/drl_pickplace", "pick_place_rl"},
    {"CheckerBoard", "move_checkerboard"},
    {"MoveCheckerBoard", "move_checkerboard"},
    {"move_checkerboard", "move_checkerboard"},
    {"move_checker_board", "move_checkerboard"},
    {"/move_checker_board", "move_checkerboard"},
    {"RepeatabilityTest", "repeatability_test"},
    {"repeatability_test", "repeatability_test"},
    {"/repeatability_test", "repeatability_test"},
    {"PickPlace", "pick_place"},
    {"pick_place", "pick_place"},
    {"/pickplace", "pick_place"},
    {"GoHome", "go_home"},
    {"GoHome2", "go_home"},
    {"gohome", "go_home"},
    {"/gohome", "go_home"},
    {"MoveGripper", "move_gripper"},
    {"move_gripper", "move_gripper"},
    {"/move_gripper", "move_gripper"},
    {"MoveToPoseObstacle", "move_to_pose_obstacle"},
    {"move_to_pose_obstacle", "move_to_pose_obstacle"},
    // codex.md §4/§2: MoveToPose & MoveToPoseCartesian were previously
    // unmapped (kept PascalCase folders); give them canonical snake_case names.
    {"MoveToPose", "move_to_pose"},
    {"move_to_pose", "move_to_pose"},
    {"/move_to_pose", "move_to_pose"},
    {"MoveToPoseCartesian", "move_to_pose_cartesian"},
    {"move_to_pose_cartesian", "move_to_pose_cartesian"},
    {"/move_to_pose_cartesian", "move_to_pose_cartesian"},
    // codex.md §6.8: MoveTargetRl gets its OWN folder (was merged into
    // move_pose_rl).
    {"MoveTargetRl", "move_target_rl"},
    {"move_target_rl", "move_target_rl"},
    {"DrlPlanner", "drl_planner"},
    {"drl_planner", "drl_planner"}
  };
  const auto it = kNames.find(action_name);
  return it == kNames.end() ? action_name : it->second;
}

// codex.md §2: PascalCase display name used as the per-action folder name in
// the normalized 3-group layout.
inline std::string actionDisplayName(const std::string & action_name)
{
  const std::string canonical = canonicalLogActionName(action_name);
  static const std::unordered_map<std::string, std::string> kDisplay{
    {"repeatability_test", "RepeatabilityTest"},
    {"move_checkerboard", "CheckerBoard"},
    {"move_to_pose", "MoveToPose"},
    {"move_to_pose_cartesian", "MoveToPoseCartesian"},
    {"move_pose_rl", "MovePoseRl"},
    {"move_target_rl", "MoveTargetRl"},
    {"move_to_pose_obstacle", "MoveToPoseObstacle"},
    {"drl_planner", "DrlPlanner"},
    {"pick_place", "PickPlace"},
    {"pick_place_rl", "PickPlaceRL"},
    {"go_home", "GoHome"},
    {"move_gripper", "MoveGripper"}
  };
  const auto it = kDisplay.find(canonical);
  return it == kDisplay.end() ? canonical : it->second;
}

// codex.md §2: maps an action to one of the three evaluation groups (or the
// _debug bucket for non-evaluation actions like GoHome/MoveGripper).
inline std::string evalGroupForAction(const std::string & canonical_action_name)
{
  if (canonical_action_name == "repeatability_test" ||
    canonical_action_name == "move_checkerboard" ||
    canonical_action_name == "move_to_pose" ||
    canonical_action_name == "move_to_pose_cartesian")
  {
    return "01_baseline_motion_eval";
  }
  if (canonical_action_name == "move_pose_rl" ||
    canonical_action_name == "move_target_rl" ||
    canonical_action_name == "move_to_pose_obstacle" ||
    canonical_action_name == "drl_planner")
  {
    return "02_rl_motion_eval";
  }
  if (canonical_action_name == "pick_place" ||
    canonical_action_name == "pick_place_rl")
  {
    return "03_task_execution_eval";
  }
  return "_debug";
}

inline std::string normalizedRuntimeMode(const std::string & runtime_mode)
{
  return runtime_mode == "real" ? "real" : "mock";
}

// Coarse planner-type label (baseline vs rl) kept for metadata/CSV. The RL
// planner actions include move_target_rl (split out from move_pose_rl) and
// drl_planner. move_to_pose_obstacle stays "baseline" (it is the MoveIt
// baseline shown alongside RL for comparison).
inline std::string logGroupForAction(const std::string & canonical_action_name)
{
  return (canonical_action_name == "move_pose_rl" ||
    canonical_action_name == "move_target_rl" ||
    canonical_action_name == "pick_place_rl" ||
    canonical_action_name == "drl_planner") ? "rl" : "baseline";
}

inline std::string standardActionLogDir(
  const std::string & log_root_dir,
  const std::string & runtime_mode,
  const std::string & action_name)
{
  // codex.md §2: <root>/<mock|real>/<NN_group>/<ActionNamePascal>/
  const std::string canonical = canonicalLogActionName(action_name);
  return (
    std::filesystem::path(absoluteLogPath(log_root_dir)) /
    normalizedRuntimeMode(runtime_mode) /
    evalGroupForAction(canonical) /
    actionDisplayName(canonical)).string();
}

inline std::string actionMetricsLogDir(
  const std::string & log_root_dir,
  const std::string & runtime_mode,
  const std::string & action_name)
{
  return standardActionLogDir(log_root_dir, runtime_mode, action_name);
}

// codex.md §3.1: recover the unified log root ("…/Log_robot_data") from any
// call/run directory path, so metadata.json can record log_root_dir without
// the logger needing it passed in explicitly.
inline std::string logRootFromPath(const std::string & path)
{
  std::filesystem::path p(path);
  for (auto it = p.begin(); it != p.end(); ++it) {
    if (it->string() == "mock" || it->string() == "real") {
      std::filesystem::path root;
      for (auto jt = p.begin(); jt != it; ++jt) {
        root /= *jt;
      }
      return root.lexically_normal().string();
    }
  }
  return defaultLogRootDir();
}

// codex.md §3.1: mock/real path branch -> hardware_mode string.
inline std::string hardwareModeFromPath(const std::string & path)
{
  std::filesystem::path p(path);
  for (const auto & part : p) {
    if (part.string() == "real") {
      return "real";
    }
    if (part.string() == "mock") {
      return "mock";
    }
  }
  return "mock";
}

inline std::string executorLogBaseDir(const std::string & log_root_dir)
{
  return (std::filesystem::path(absoluteLogPath(log_root_dir)) / "executor_logs").string();
}

inline std::string executorActionLogDir(
  const std::string & log_root_dir,
  const std::string & configured_executor_dir,
  const std::string & action_name)
{
  (void)configured_executor_dir;
  return standardActionLogDir(log_root_dir, "mock", action_name);
}

inline std::string executorActionLogDir(
  const std::string & log_root_dir,
  const std::string & configured_executor_dir,
  const std::string & runtime_mode,
  const std::string & action_name)
{
  (void)configured_executor_dir;
  return standardActionLogDir(log_root_dir, runtime_mode, action_name);
}

inline std::string legacyExecutorActionLogDir(
  const std::string & log_root_dir,
  const std::string & configured_executor_dir,
  const std::string & action_name)
{
  std::filesystem::path base = configured_executor_dir.empty() ?
    std::filesystem::path(executorLogBaseDir(log_root_dir)) :
    std::filesystem::path(absoluteLogPath(configured_executor_dir));
  if (base.filename() != action_name) {
    base /= action_name;
  }
  return base.lexically_normal().string();
}

}  // namespace robot_task_manager
