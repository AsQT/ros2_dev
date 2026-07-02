#pragma once

// ActionMetricsLogger — opt-in, one-CSV-row-per-goal-call summary logger used
// to compare the MoveIt classical baseline (MoveToPoseObstacle) against the
// RL planner actions (MoveTargetRl, MovePoseRl, ...) on equal footing.
//
// Deliberately separate from robot_task_executor::ExecutorExperimentLogger
// (node-wide, gated by a ROS param, used for low-level lifecycle/sample
// rows) and from PerCallTcpLogger (per-sample TCP set-vs-actual pose CSV).
// This logger writes exactly one summary row per action call, gated purely
// by the goal-level `enable_metrics_log` field — never by a node parameter,
// so it can never fire unless the caller explicitly asked for it on that
// specific goal.
//
// One ActionMetricsRow is created per goal execute() (via startCall) and
// finalized exactly once via finish(), which must be called at every exit
// path of execute() (success or failure) so codex.md's "log even on
// failure" requirement always holds. The CSV filename is reserved at
// startCall() time (empty placeholder file) so two calls in the same
// process/second can never collide or overwrite each other; the actual
// header+row content is written once, at finish().

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_task_manager
{

// One row = one action call. Every field defaults to "unknown/not
// applicable" (NaN for doubles, empty for strings, unset optional for
// tri-state bools) so a call that fails before a field is ever computed
// still produces a valid CSV row with that cell blank, per codex.md 5.2.
struct ActionMetricsRow
{
  // -- 5.1 identity ---------------------------------------------------
  std::string timestamp_start_iso;
  std::string timestamp_end_iso;
  std::string action_name;
  std::string planner_type;   // "moveit_baseline" | "rl"
  std::string run_id;
  std::string goal_uuid;
  std::string source;         // "vision" | "fallback" | "manual" | "no_obstacle"

  // -- 5.2 result status ------------------------------------------------
  std::optional<bool> success;
  std::optional<bool> planning_success;
  std::optional<bool> execution_success;
  std::optional<bool> action_result_success;
  std::string failed_stage;
  std::string message;
  std::optional<bool> execute_requested;

  // -- 5.3 timing -------------------------------------------------------
  double planning_time_s = std::numeric_limits<double>::quiet_NaN();
  double execution_time_s = std::numeric_limits<double>::quiet_NaN();
  double total_action_time_s = std::numeric_limits<double>::quiet_NaN();
  double vision_wait_time_s = std::numeric_limits<double>::quiet_NaN();
  double tf_time_s = std::numeric_limits<double>::quiet_NaN();
  double planning_scene_update_time_s = std::numeric_limits<double>::quiet_NaN();
  double drl_plan_time_s = std::numeric_limits<double>::quiet_NaN();
  double drl_execution_wait_time_s = std::numeric_limits<double>::quiet_NaN();
  double moveit_plan_time_s = std::numeric_limits<double>::quiet_NaN();
  double moveit_execute_time_s = std::numeric_limits<double>::quiet_NaN();

  // -- 5.4 pose -----------------------------------------------------------
  double start_x = std::numeric_limits<double>::quiet_NaN();
  double start_y = std::numeric_limits<double>::quiet_NaN();
  double start_z = std::numeric_limits<double>::quiet_NaN();
  double target_x = std::numeric_limits<double>::quiet_NaN();
  double target_y = std::numeric_limits<double>::quiet_NaN();
  double target_z = std::numeric_limits<double>::quiet_NaN();
  double final_x = std::numeric_limits<double>::quiet_NaN();
  double final_y = std::numeric_limits<double>::quiet_NaN();
  double final_z = std::numeric_limits<double>::quiet_NaN();
  double final_position_error_m = std::numeric_limits<double>::quiet_NaN();
  double final_orientation_error_rad = std::numeric_limits<double>::quiet_NaN();
  // codex2.md "move_pose_rl_current_tcp_start": prove the planned
  // trajectory's first waypoint actually starts at the current TCP pose
  // used as start_x/y/z above, not some stale/default pose.
  double first_point_x = std::numeric_limits<double>::quiet_NaN();
  double first_point_y = std::numeric_limits<double>::quiet_NaN();
  double first_point_z = std::numeric_limits<double>::quiet_NaN();
  double distance_start_to_first_point = std::numeric_limits<double>::quiet_NaN();
  std::string start_source;  // e.g. "tf_lookup"

  // -- 5.5 obstacle ------------------------------------------------------
  std::optional<bool> has_obstacle;
  std::string obstacle_source;
  double obstacle_center_x = std::numeric_limits<double>::quiet_NaN();
  double obstacle_center_y = std::numeric_limits<double>::quiet_NaN();
  double obstacle_center_z = std::numeric_limits<double>::quiet_NaN();
  double obstacle_size_x = std::numeric_limits<double>::quiet_NaN();
  double obstacle_size_y = std::numeric_limits<double>::quiet_NaN();
  double obstacle_size_z = std::numeric_limits<double>::quiet_NaN();
  std::string obstacle_class;
  std::string obstacle_frame;
  std::optional<bool> obstacle_added_to_planning_scene;
  std::optional<bool> rl_has_obstacle;
  double manual_default_obstacle_size_x = std::numeric_limits<double>::quiet_NaN();
  double manual_default_obstacle_size_y = std::numeric_limits<double>::quiet_NaN();
  double manual_default_obstacle_size_z = std::numeric_limits<double>::quiet_NaN();
  std::string planning_scene_object_id;
  std::optional<bool> collision_object_added;
  std::optional<bool> collision_object_removed;

  // -- 5.6 trajectory/path -------------------------------------------------
  double trajectory_points = std::numeric_limits<double>::quiet_NaN();
  double path_length_m = std::numeric_limits<double>::quiet_NaN();
  double straight_line_distance_m = std::numeric_limits<double>::quiet_NaN();
  double path_efficiency = std::numeric_limits<double>::quiet_NaN();

  // -- 5.7 clearance --------------------------------------------------------
  double min_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double average_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double start_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double target_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double clearance_violation_count = std::numeric_limits<double>::quiet_NaN();
  double collision_or_inside_obstacle_count = std::numeric_limits<double>::quiet_NaN();
  double safety_margin_m = std::numeric_limits<double>::quiet_NaN();
  double min_clearance_with_margin_m = std::numeric_limits<double>::quiet_NaN();
  std::optional<bool> clearance_ok;

  // -- 5.8 workspace ----------------------------------------------------
  double workspace_violation_count = std::numeric_limits<double>::quiet_NaN();
  double workspace_min_x = std::numeric_limits<double>::quiet_NaN();
  double workspace_min_y = std::numeric_limits<double>::quiet_NaN();
  double workspace_min_z = std::numeric_limits<double>::quiet_NaN();
  double workspace_max_x = std::numeric_limits<double>::quiet_NaN();
  double workspace_max_y = std::numeric_limits<double>::quiet_NaN();
  double workspace_max_z = std::numeric_limits<double>::quiet_NaN();

  // -- 5.9 planner-specific ------------------------------------------------
  std::string rl_model_name;
  std::string rl_algorithm;
  double rl_action_step = std::numeric_limits<double>::quiet_NaN();
  double rl_max_steps = std::numeric_limits<double>::quiet_NaN();
  std::optional<bool> rl_converged;
  double rl_rollout_steps = std::numeric_limits<double>::quiet_NaN();
  double rl_reward_total = std::numeric_limits<double>::quiet_NaN();
  std::string rl_policy_obstacle;

  std::string moveit_planning_group;
  std::string moveit_planner_id;
  double moveit_num_planning_attempts = std::numeric_limits<double>::quiet_NaN();
  double moveit_allowed_planning_time_s = std::numeric_limits<double>::quiet_NaN();
  double moveit_error_code = std::numeric_limits<double>::quiet_NaN();

  // -- internal bookkeeping, managed by ActionMetricsLogger only --------
  std::string internal_csv_path;
  bool internal_finished = false;
};

// -----------------------------------------------------------------------
// Small formatting/utility helpers (mirrors per_call_tcp_logger.hpp style).
// -----------------------------------------------------------------------

inline std::string metricsFormatDouble(double value)
{
  if (!std::isfinite(value)) {
    return "";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << value;
  return out.str();
}

// For fields that are conceptually integer counts but stored as double so
// NaN ("not available") is representable.
inline std::string metricsFormatCount(double value)
{
  if (!std::isfinite(value)) {
    return "";
  }
  return std::to_string(static_cast<long long>(std::llround(value)));
}

inline std::string metricsFormatOptBool(const std::optional<bool> & value)
{
  if (!value.has_value()) {
    return "";
  }
  return *value ? "true" : "false";
}

inline std::string metricsCsvEscape(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped += c;
    }
  }
  escaped += "\"";
  return escaped;
}

inline std::string metricsTimeStamp(const char * fmt)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&now_time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, fmt);
  return out.str();
}

inline std::string metricsIsoNow()
{
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&now_time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
      << "." << std::setw(3) << std::setfill('0') << ms.count();
  return out.str();
}

// Hex-encodes a ROS 2 action GoalUUID (std::array<uint8_t,16>) — used both
// for the CSV goal_uuid column and to help make the CSV filename unique.
inline std::string goalUuidHex(const std::array<uint8_t, 16> & uuid)
{
  std::ostringstream out;
  for (uint8_t b : uuid) {
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
  }
  return out.str();
}

// -----------------------------------------------------------------------
// Shared trajectory / obstacle-clearance metrics — usable by both the
// MoveIt baseline (waypoints from FK on the planned trajectory) and RL
// (waypoints from the cached /drl/forward_trajectory_poses PoseArray),
// since both reduce to a plain vector of Cartesian TCP positions.
// -----------------------------------------------------------------------

struct AabbObstacle
{
  geometry_msgs::msg::Point center;
  geometry_msgs::msg::Vector3 size;
  bool has_obstacle = false;
};

struct TrajectoryMetrics
{
  double path_length_m = std::numeric_limits<double>::quiet_NaN();
  double straight_line_distance_m = std::numeric_limits<double>::quiet_NaN();
  double path_efficiency = std::numeric_limits<double>::quiet_NaN();
  double min_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double average_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double start_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double target_clearance_m = std::numeric_limits<double>::quiet_NaN();
  double min_clearance_with_margin_m = std::numeric_limits<double>::quiet_NaN();
  double clearance_violation_count = std::numeric_limits<double>::quiet_NaN();
  double collision_or_inside_obstacle_count = std::numeric_limits<double>::quiet_NaN();
  std::optional<bool> clearance_ok;
};

// Signed distance from `p` to the AABB [center - size/2, center + size/2]:
// positive when outside (Euclidean distance to the nearest face/edge/
// corner), <= 0 when inside (negative penetration depth along the
// shallowest-overlap axis). Matches codex.md 5.7's definition.
inline double aabbSignedDistance(
  const geometry_msgs::msg::Point & p,
  const geometry_msgs::msg::Point & center,
  const geometry_msgs::msg::Vector3 & size)
{
  const double hx = size.x / 2.0;
  const double hy = size.y / 2.0;
  const double hz = size.z / 2.0;
  const double dx = std::abs(p.x - center.x) - hx;
  const double dy = std::abs(p.y - center.y) - hy;
  const double dz = std::abs(p.z - center.z) - hz;

  const double ox = std::max(dx, 0.0);
  const double oy = std::max(dy, 0.0);
  const double oz = std::max(dz, 0.0);
  const double outside = std::sqrt(ox * ox + oy * oy + oz * oz);
  if (outside > 1e-12) {
    return outside;
  }
  // Inside on every axis: negative penetration depth = distance to the
  // nearest face (least-negative axis overlap).
  return std::max({dx, dy, dz});
}

inline TrajectoryMetrics computeTrajectoryMetrics(
  const std::vector<geometry_msgs::msg::Point> & waypoints,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & target,
  const AabbObstacle * obstacle,
  double safety_margin_m)
{
  TrajectoryMetrics m;

  if (waypoints.size() >= 2) {
    double length = 0.0;
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      const double dx = waypoints[i + 1].x - waypoints[i].x;
      const double dy = waypoints[i + 1].y - waypoints[i].y;
      const double dz = waypoints[i + 1].z - waypoints[i].z;
      length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    if (length > 0.0) {
      m.path_length_m = length;
    }
  }

  {
    const double dx = target.x - start.x;
    const double dy = target.y - start.y;
    const double dz = target.z - start.z;
    m.straight_line_distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  if (std::isfinite(m.path_length_m) && m.path_length_m > 0.0) {
    m.path_efficiency = m.straight_line_distance_m / m.path_length_m;
  }

  if (!obstacle || !obstacle->has_obstacle) {
    m.clearance_ok = true;
    m.clearance_violation_count = 0;
    m.collision_or_inside_obstacle_count = 0;
    return m;
  }

  m.start_clearance_m = aabbSignedDistance(start, obstacle->center, obstacle->size);
  m.target_clearance_m = aabbSignedDistance(target, obstacle->center, obstacle->size);

  if (!waypoints.empty()) {
    double min_c = std::numeric_limits<double>::infinity();
    double sum_c = 0.0;
    int violation = 0;
    int inside = 0;
    for (const auto & wp : waypoints) {
      const double c = aabbSignedDistance(wp, obstacle->center, obstacle->size);
      min_c = std::min(min_c, c);
      sum_c += c;
      if (c <= 0.0) {
        ++inside;
      }
      if (c - safety_margin_m < 0.0) {
        ++violation;
      }
    }
    m.min_clearance_m = min_c;
    m.average_clearance_m = sum_c / static_cast<double>(waypoints.size());
    m.min_clearance_with_margin_m = min_c - safety_margin_m;
    m.clearance_violation_count = violation;
    m.collision_or_inside_obstacle_count = inside;
    m.clearance_ok = (m.min_clearance_with_margin_m >= 0.0);
  }

  return m;
}

// -----------------------------------------------------------------------
// ActionMetricsLogger
// -----------------------------------------------------------------------

class ActionMetricsLogger
{
public:
  // base_dir e.g. "Log_robot_data/action_metrics/MoveToPoseObstacle" — one logger
  // instance per action server, already scoped to that action's subfolder.
  explicit ActionMetricsLogger(std::string base_dir)
  : base_dir_(std::filesystem::absolute(std::filesystem::path(std::move(base_dir))).lexically_normal().string())
  {
    createBaseDir();
  }

  ActionMetricsLogger(std::string base_dir, rclcpp::Logger logger)
  : base_dir_(std::filesystem::absolute(std::filesystem::path(std::move(base_dir))).lexically_normal().string()),
    logger_(std::move(logger)),
    has_logger_(true)
  {
    createBaseDir();
  }

  void createBaseDir()
  {
    std::error_code ec;
    std::filesystem::create_directories(base_dir_, ec);
    if (ec) {
      logWarn("Failed to create log directory: " + base_dir_ + " | " + ec.message());
    }
  }

  // Reserves a unique CSV path (empty placeholder file, so no other call —
  // even in the same second — can ever pick the same name/overwrite it)
  // and returns a fresh row to be filled in by the caller as execute()
  // progresses. Returns nullptr only if the directory/file could not be
  // created; callers must treat that as "logging unavailable" and must
  // NOT fail the action because of it.
  std::shared_ptr<ActionMetricsRow> startCall(
    const std::string & action_name,
    const std::string & planner_type,
    const std::string & goal_uuid_hex)
  {
    auto row = std::make_shared<ActionMetricsRow>();
    row->action_name = action_name;
    row->planner_type = planner_type;
    row->goal_uuid = goal_uuid_hex;
    row->timestamp_start_iso = metricsIsoNow();

    std::lock_guard<std::mutex> lock(mutex_);
    const std::string stamp = metricsTimeStamp("%Y%m%d_%H%M%S");
    const std::string tag = goal_uuid_hex.empty() ?
      ("pid" + std::to_string(::getpid()) + "_" + std::to_string(++next_index_)) :
      goal_uuid_hex.substr(0, 8);

    std::string path;
    for (int attempt = 0; attempt < 1000; ++attempt) {
      std::ostringstream name;
      name << base_dir_ << "/run_" << stamp << "_" << tag;
      if (attempt > 0) {
        name << "_" << attempt;
      }
      name << "_summary.csv";
      if (!std::filesystem::exists(name.str())) {
        path = name.str();
        break;
      }
    }
    if (path.empty()) {
      return nullptr;
    }

    std::ofstream touch(path, std::ios::out | std::ios::trunc);
    if (!touch.is_open()) {
      logWarn("Failed to write log CSV: " + path);
      return nullptr;
    }
    touch.close();

    row->run_id = stamp + "_" + tag;
    row->internal_csv_path = path;
    logInfo("Metrics CSV saved: " + path);
    return row;
  }

  // Single required call at every exit path (success or failure). Writes
  // the header + exactly one data row. Safe to call with a nullptr row
  // (e.g. logging was disabled or startCall() failed) — no-op. Safe to
  // call twice on the same row — the second call is a no-op.
  void finish(const std::shared_ptr<ActionMetricsRow> & row)
  {
    if (!row || row->internal_csv_path.empty() || row->internal_finished) {
      return;
    }
    if (row->timestamp_end_iso.empty()) {
      row->timestamp_end_iso = metricsIsoNow();
    }

    std::ofstream out(row->internal_csv_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      logWarn("Failed to write log CSV: " + row->internal_csv_path);
      return;
    }
    out << header() << rowToCsv(*row);
    out.flush();
    row->internal_finished = true;
  }

private:
  static std::string header()
  {
    return
      "timestamp_start,timestamp_end,action_name,planner_type,run_id,goal_uuid,source,"
      "success,planning_success,execution_success,action_result_success,failed_stage,message,execute_requested,"
      "planning_time_s,execution_time_s,total_action_time_s,vision_wait_time_s,tf_time_s,"
      "planning_scene_update_time_s,drl_plan_time_s,drl_execution_wait_time_s,moveit_plan_time_s,moveit_execute_time_s,"
      "start_x,start_y,start_z,target_x,target_y,target_z,final_x,final_y,final_z,"
      "final_position_error_m,final_orientation_error_rad,"
      "first_point_x,first_point_y,first_point_z,distance_start_to_first_point,start_source,"
      "has_obstacle,obstacle_source,obstacle_center_x,obstacle_center_y,obstacle_center_z,"
      "obstacle_size_x,obstacle_size_y,obstacle_size_z,obstacle_class,obstacle_frame,"
      "obstacle_added_to_planning_scene,rl_has_obstacle,"
      "manual_default_obstacle_size_x,manual_default_obstacle_size_y,manual_default_obstacle_size_z,"
      "planning_scene_object_id,collision_object_added,collision_object_removed,"
      "trajectory_points,path_length_m,straight_line_distance_m,path_efficiency,"
      "min_clearance_m,average_clearance_m,start_clearance_m,target_clearance_m,"
      "clearance_violation_count,collision_or_inside_obstacle_count,safety_margin_m,"
      "min_clearance_with_margin_m,clearance_ok,"
      "workspace_violation_count,workspace_min_x,workspace_min_y,workspace_min_z,"
      "workspace_max_x,workspace_max_y,workspace_max_z,"
      "rl_model_name,rl_algorithm,rl_action_step,rl_max_steps,rl_converged,rl_rollout_steps,"
      "rl_reward_total,rl_policy_obstacle,"
      "moveit_planning_group,moveit_planner_id,moveit_num_planning_attempts,"
      "moveit_allowed_planning_time_s,moveit_error_code\n";
  }

  static std::string rowToCsv(const ActionMetricsRow & r)
  {
    std::ostringstream out;
    out << metricsCsvEscape(r.timestamp_start_iso) << "," << metricsCsvEscape(r.timestamp_end_iso) << ","
        << metricsCsvEscape(r.action_name) << "," << metricsCsvEscape(r.planner_type) << ","
        << metricsCsvEscape(r.run_id) << "," << metricsCsvEscape(r.goal_uuid) << ","
        << metricsCsvEscape(r.source) << ","
        << metricsFormatOptBool(r.success) << "," << metricsFormatOptBool(r.planning_success) << ","
        << metricsFormatOptBool(r.execution_success) << "," << metricsFormatOptBool(r.action_result_success) << ","
        << metricsCsvEscape(r.failed_stage) << "," << metricsCsvEscape(r.message) << ","
        << metricsFormatOptBool(r.execute_requested) << ","
        << metricsFormatDouble(r.planning_time_s) << "," << metricsFormatDouble(r.execution_time_s) << ","
        << metricsFormatDouble(r.total_action_time_s) << "," << metricsFormatDouble(r.vision_wait_time_s) << ","
        << metricsFormatDouble(r.tf_time_s) << "," << metricsFormatDouble(r.planning_scene_update_time_s) << ","
        << metricsFormatDouble(r.drl_plan_time_s) << "," << metricsFormatDouble(r.drl_execution_wait_time_s) << ","
        << metricsFormatDouble(r.moveit_plan_time_s) << "," << metricsFormatDouble(r.moveit_execute_time_s) << ","
        << metricsFormatDouble(r.start_x) << "," << metricsFormatDouble(r.start_y) << ","
        << metricsFormatDouble(r.start_z) << "," << metricsFormatDouble(r.target_x) << ","
        << metricsFormatDouble(r.target_y) << "," << metricsFormatDouble(r.target_z) << ","
        << metricsFormatDouble(r.final_x) << "," << metricsFormatDouble(r.final_y) << ","
        << metricsFormatDouble(r.final_z) << ","
        << metricsFormatDouble(r.final_position_error_m) << "," << metricsFormatDouble(r.final_orientation_error_rad) << ","
        << metricsFormatDouble(r.first_point_x) << "," << metricsFormatDouble(r.first_point_y) << ","
        << metricsFormatDouble(r.first_point_z) << "," << metricsFormatDouble(r.distance_start_to_first_point) << ","
        << metricsCsvEscape(r.start_source) << ","
        << metricsFormatOptBool(r.has_obstacle) << "," << metricsCsvEscape(r.obstacle_source) << ","
        << metricsFormatDouble(r.obstacle_center_x) << "," << metricsFormatDouble(r.obstacle_center_y) << ","
        << metricsFormatDouble(r.obstacle_center_z) << ","
        << metricsFormatDouble(r.obstacle_size_x) << "," << metricsFormatDouble(r.obstacle_size_y) << ","
        << metricsFormatDouble(r.obstacle_size_z) << ","
        << metricsCsvEscape(r.obstacle_class) << "," << metricsCsvEscape(r.obstacle_frame) << ","
        << metricsFormatOptBool(r.obstacle_added_to_planning_scene) << "," << metricsFormatOptBool(r.rl_has_obstacle) << ","
        << metricsFormatDouble(r.manual_default_obstacle_size_x) << ","
        << metricsFormatDouble(r.manual_default_obstacle_size_y) << ","
        << metricsFormatDouble(r.manual_default_obstacle_size_z) << ","
        << metricsCsvEscape(r.planning_scene_object_id) << ","
        << metricsFormatOptBool(r.collision_object_added) << "," << metricsFormatOptBool(r.collision_object_removed) << ","
        << metricsFormatCount(r.trajectory_points) << "," << metricsFormatDouble(r.path_length_m) << ","
        << metricsFormatDouble(r.straight_line_distance_m) << "," << metricsFormatDouble(r.path_efficiency) << ","
        << metricsFormatDouble(r.min_clearance_m) << "," << metricsFormatDouble(r.average_clearance_m) << ","
        << metricsFormatDouble(r.start_clearance_m) << "," << metricsFormatDouble(r.target_clearance_m) << ","
        << metricsFormatCount(r.clearance_violation_count) << ","
        << metricsFormatCount(r.collision_or_inside_obstacle_count) << ","
        << metricsFormatDouble(r.safety_margin_m) << "," << metricsFormatDouble(r.min_clearance_with_margin_m) << ","
        << metricsFormatOptBool(r.clearance_ok) << ","
        << metricsFormatCount(r.workspace_violation_count) << ","
        << metricsFormatDouble(r.workspace_min_x) << "," << metricsFormatDouble(r.workspace_min_y) << ","
        << metricsFormatDouble(r.workspace_min_z) << ","
        << metricsFormatDouble(r.workspace_max_x) << "," << metricsFormatDouble(r.workspace_max_y) << ","
        << metricsFormatDouble(r.workspace_max_z) << ","
        << metricsCsvEscape(r.rl_model_name) << "," << metricsCsvEscape(r.rl_algorithm) << ","
        << metricsFormatCount(r.rl_action_step) << "," << metricsFormatCount(r.rl_max_steps) << ","
        << metricsFormatOptBool(r.rl_converged) << "," << metricsFormatCount(r.rl_rollout_steps) << ","
        << metricsFormatDouble(r.rl_reward_total) << "," << metricsCsvEscape(r.rl_policy_obstacle) << ","
        << metricsCsvEscape(r.moveit_planning_group) << "," << metricsCsvEscape(r.moveit_planner_id) << ","
        << metricsFormatCount(r.moveit_num_planning_attempts) << ","
        << metricsFormatDouble(r.moveit_allowed_planning_time_s) << ","
        << metricsFormatCount(r.moveit_error_code) << "\n";
    return out.str();
  }

  std::string base_dir_;
  std::mutex mutex_;
  uint64_t next_index_ = 0;
  rclcpp::Logger logger_{rclcpp::get_logger("ActionMetricsLogger")};
  bool has_logger_ = false;

  void logInfo(const std::string & message) const
  {
    if (has_logger_) {
      RCLCPP_INFO(logger_, "%s", message.c_str());
    } else {
      std::cerr << message << '\n';
    }
  }

  void logWarn(const std::string & message) const
  {
    if (has_logger_) {
      RCLCPP_WARN(logger_, "%s", message.c_str());
    } else {
      std::cerr << message << '\n';
    }
  }
};

}  // namespace robot_task_manager
