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
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <unistd.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/log_plot_hook.hpp"
#include "robot_task_manager/standard_action_logger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

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

  // Execute-time TCP sampling metrics (codex.md §3.4). Filled by
  // ActionMetricsLogger::stopTcpExecutionSampling() when a server wraps a real
  // execute phase with the TF sampler.
  double actual_path_length_m = std::numeric_limits<double>::quiet_NaN();
  double rmse_tracking_to_rl_path_m = std::numeric_limits<double>::quiet_NaN();
  double rmse_tracking_to_moveit_path_m = std::numeric_limits<double>::quiet_NaN();
  double max_tracking_error_to_rl_path_m = std::numeric_limits<double>::quiet_NaN();
  double max_tracking_error_to_moveit_path_m = std::numeric_limits<double>::quiet_NaN();
  double min_clearance_actual_m = std::numeric_limits<double>::quiet_NaN();

  // codex.md §5: free-form extra fields (e.g. MoveTargetRl vision provenance)
  // dumped verbatim into metadata.json.
  std::vector<std::pair<std::string, std::string>> extra_metadata;

  // -- internal bookkeeping, managed by ActionMetricsLogger only --------
  std::string internal_csv_path;
  std::string internal_call_dir;
  std::string action_call_id;
  std::string parent_action_call_id;
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

inline std::array<double, 3> metricsQuatToRpy(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  if (quat.length2() <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }
  quat.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

inline double metricsPointDistance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
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

struct TcpExecutionSample
{
  double t_s = 0.0;
  std::string stage;
  geometry_msgs::msg::Pose pose;
  double distance_to_target = std::numeric_limits<double>::quiet_NaN();
  double tracking_error_to_rl_path_m = std::numeric_limits<double>::quiet_NaN();
  double tracking_error_to_moveit_path_m = std::numeric_limits<double>::quiet_NaN();
  double distance_to_obstacle = std::numeric_limits<double>::quiet_NaN();
  double path_length_actual_so_far_m = 0.0;
  geometry_msgs::msg::Point nearest_rl_plan;
  geometry_msgs::msg::Point nearest_moveit;
  bool has_nearest_rl = false;
  bool has_nearest_moveit = false;
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
    createRunDir();
  }

  ActionMetricsLogger(std::string base_dir, rclcpp::Logger logger)
  : base_dir_(std::filesystem::absolute(std::filesystem::path(std::move(base_dir))).lexically_normal().string()),
    logger_(std::move(logger)),
    has_logger_(true)
  {
    createRunDir();
  }

  // codex.md §6: hardware provenance for metadata.json. Optional.
  void setHardwareInfo(const std::string & use_mock, const std::string & hardware_plugin)
  {
    use_mock_ = use_mock;
    hardware_plugin_ = hardware_plugin;
  }

  // codex.md §5/§7: toggle post-finish verification plots (default true).
  void setPlotsEnabled(bool enabled) {plots_enabled_ = enabled;}

  void createRunDir()
  {
    const std::string stamp = metricsTimeStamp("%Y%m%d_%H%M%S") + "_" + std::to_string(::getpid());
    for (int suffix = 0; suffix < 1000; ++suffix) {
      std::ostringstream candidate;
      candidate << base_dir_ << "/run_" << stamp;
      if (suffix > 0) {
        candidate << "_" << std::setw(3) << std::setfill('0') << suffix;
      }
      std::error_code ec;
      if (std::filesystem::exists(candidate.str())) {
        continue;
      }
      std::filesystem::create_directories(candidate.str(), ec);
      if (!ec) {
        run_dir_ = candidate.str();
        run_id_ = std::filesystem::path(candidate.str()).filename().string();
        return;
      }
    }
    logWarn("Failed to create standard action metrics run directory: " + base_dir_);
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
    if (run_dir_.empty()) {
      return nullptr;
    }
    const uint64_t index = ++next_index_;
    std::ostringstream call_name;
    call_name << "call_" << std::setw(4) << std::setfill('0') << index;
    const std::string call_dir = (std::filesystem::path(run_dir_) / call_name.str()).string();
    std::error_code ec;
    std::filesystem::create_directories(call_dir, ec);
    if (ec) {
      logWarn("Failed to create metrics call directory: " + call_dir + " | " + ec.message());
      return nullptr;
    }
    const std::string path = (std::filesystem::path(call_dir) / "summary.csv").string();

    std::ofstream touch(path, std::ios::out | std::ios::trunc);
    if (!touch.is_open()) {
      logWarn("Failed to write log CSV: " + path);
      return nullptr;
    }
    touch.close();

    row->run_id = run_id_;
    row->action_call_id = call_name.str();
    row->internal_call_dir = call_dir;
    row->internal_csv_path = path;
    writeMetadata(*row);
    appendEvent(*row, "start", "action_start", std::nullopt, "metrics call created");
    writeActionSpecificPlaceholders(*row);
    logInfo("Metrics call directory: " + call_dir);
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
    appendEvent(
      *row,
      row->failed_stage.empty() ? "done" : row->failed_stage,
      "action_result",
      row->success,
      row->message);

    std::ofstream out(row->internal_csv_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      logWarn("Failed to write log CSV: " + row->internal_csv_path);
      return;
    }
    out << header() << rowToCsv(*row);
    out.flush();

    // codex.md §5.3: obstacle.csv now has a REAL writer (fed from the row the
    // server already populated), replacing the old header-only placeholder.
    // A no-obstacle call is recorded honestly with data_available=false.
    writeObstacleCsv(*row);
    // codex.md §5: trajectory_metrics.csv (real) + honest data_available markers
    // for the execution-sampling files this logger cannot yet produce.
    writeTrajectoryMetricsCsv(*row);
    writeExecutionMarkers(*row);
    row->internal_finished = true;

    // codex.md §5/§7: fire verification plots (detached, never blocks/fails).
    runLogPlotsAsync(logger_, row->internal_call_dir, plots_enabled_);
  }

  // codex.md §2.4/§5: key-value trajectory metrics derived from the row.
  void writeTrajectoryMetricsCsv(const ActionMetricsRow & row)
  {
    if (row.internal_call_dir.empty()) {
      return;
    }
    std::ofstream out(std::filesystem::path(row.internal_call_dir) / "trajectory_metrics.csv",
      std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << "metric_name,value,unit,description\n";
    const auto r = [&out](const std::string & n, double v, const std::string & u,
        const std::string & d) {
      out << n << "," << metricsFormatDouble(v) << "," << u << "," << d << "\n";
    };
    r("rl_num_points", row.trajectory_points, "count", "RL planned waypoint count");
    r("rl_path_length_m", row.path_length_m, "m", "RL planned path length");
    r("straight_line_distance_m", row.straight_line_distance_m, "m", "start->target straight line");
    r("path_efficiency_rl", row.path_efficiency, "ratio", "straight/rl_path");
    r("final_position_error_m", row.final_position_error_m, "m", "final TCP position error");
    r("final_orientation_error_rad", row.final_orientation_error_rad, "rad", "final TCP orientation error");
    r("min_clearance_plan_m", row.min_clearance_m, "m", "min obstacle clearance on planned path");
    r("rl_planning_time_s", row.drl_plan_time_s, "s", "DRL planning time");
    r("moveit_planning_time_s", row.moveit_plan_time_s, "s", "MoveIt planning time");
    r("execution_time_s", row.execution_time_s, "s", "execution time");
    r("total_time_s", row.total_action_time_s, "s", "total action time");
    r("actual_path_length_m", row.actual_path_length_m, "m", "execute-time sampled TCP path length");
    r("rmse_tracking_to_rl_path_m", row.rmse_tracking_to_rl_path_m, "m", "actual TCP RMSE to nearest RL path waypoint");
    r("rmse_tracking_to_moveit_path_m", row.rmse_tracking_to_moveit_path_m, "m", "actual TCP RMSE to nearest MoveIt path waypoint");
    r("max_tracking_error_to_rl_path_m", row.max_tracking_error_to_rl_path_m, "m", "max actual TCP distance to nearest RL path waypoint");
    r("max_tracking_error_to_moveit_path_m", row.max_tracking_error_to_moveit_path_m, "m", "max actual TCP distance to nearest MoveIt path waypoint");
    r("min_clearance_actual_m", row.min_clearance_actual_m, "m", "min execute-time obstacle clearance from sampled TCP");
  }

  // codex.md §5.5/§5.6: files that require execute-time joint/TCP sampling,
  // which this logger does not yet perform. Written honestly with
  // data_available=false + empty_reason (never header-only-unexplained).
  void writeExecutionMarkers(const ActionMetricsRow & row)
  {
    if (row.internal_call_dir.empty()) {
      return;
    }
    const bool plan_only = row.execute_requested.has_value() && !(*row.execute_requested);
    const auto dir = std::filesystem::path(row.internal_call_dir);
    // Accurate, non-generic reasons (codex.md §3.3): RL uses the DRL planner
    // (no MoveIt joint path); execute-time TCP sampling is done server-side and
    // not by this summary-only logger.
    const std::string moveit_reason = plan_only ?
      "planning_only_no_moveit_execution" :
      "drl_planner_used_no_moveit_joint_path_available";
    const std::string tcp_reason = plan_only ?
      "planning_only_no_tcp_execution" :
      "execute_tcp_tf_lookup_unavailable_or_no_samples";
    {
      const auto path = dir / "moveit_execution_path.csv";
      std::ofstream out(path, std::ios::out | (hasRealDataRows(path) ? std::ios::app : std::ios::trunc));
      if (out.is_open() && !hasRealDataRows(path)) {
        out << "data_available,empty_reason,moveit_point_index,t_from_start_s,x,y,z,roll,pitch,yaw,"
               "q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,source_rl_point_index\n";
        out << "false," << moveit_reason << ",,,,,,,,,,,,,,,,,,,,,\n";
      }
    }
    {
      const auto path = dir / "tcp_tracking.csv";
      if (hasRealDataRows(path)) {
        // Server-side execute sampler already wrote real TF samples; keep them.
      } else {
      std::ofstream out(path, std::ios::out | std::ios::trunc);
      if (out.is_open()) {
        out << "data_available,empty_reason,t_s,stage,tcp_x_actual,tcp_y_actual,tcp_z_actual,"
               "tcp_roll_actual,tcp_pitch_actual,tcp_yaw_actual,target_x,target_y,target_z,"
               "target_roll,target_pitch,target_yaw,distance_to_target,nearest_rl_plan_x,"
               "nearest_rl_plan_y,nearest_rl_plan_z,nearest_moveit_x,nearest_moveit_y,"
               "nearest_moveit_z,tracking_error_to_rl_path_m,tracking_error_to_moveit_path_m,"
               "distance_to_obstacle,path_length_actual_so_far_m\n";
        out << "false," << tcp_reason << ",,,,,,,,,,,,,,,,,,,,,,,,,\n";
      }
      }
    }
    {
      const auto path = dir / "error_tracking.csv";
      if (hasRealDataRows(path)) {
        // Server-side execute sampler already wrote real TF samples; keep them.
      } else {
      std::ofstream out(path, std::ios::out | std::ios::trunc);
      if (out.is_open()) {
        out << "data_available,empty_reason,t_s,stage,distance_to_target,"
               "tracking_error_to_rl_path_m,tracking_error_to_moveit_path_m,distance_to_obstacle\n";
        out << "false," << tcp_reason << ",,,,,,\n";
      }
      }
    }
    // codex.md §4.2 (task group): object_tracking.csv. No continuous object
    // pose source is wired yet, so this is an honest data_available=false marker.
    if (canonicalLogActionName(row.action_name) == "pick_place_rl") {
      std::ofstream out(dir / "object_tracking.csv", std::ios::out | std::ios::trunc);
      if (out.is_open()) {
        out << "data_available,empty_reason,t_s,phase,object_id,object_source,"
               "object_x,object_y,object_z,object_roll,object_pitch,object_yaw,"
               "object_confidence,object_attached,object_dropped\n";
        out << "false,no_continuous_object_tracking_source,,,,,,,,,,,,,\n";
      }
    }
  }

  // codex.md §5.3: one-row obstacle summary for this call (no server changes —
  // uses the ActionMetricsRow fields the server filled during planning).
  void writeObstacleCsv(const ActionMetricsRow & row)
  {
    if (row.internal_call_dir.empty()) {
      return;
    }
    const std::filesystem::path path =
      std::filesystem::path(row.internal_call_dir) / "obstacle.csv";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << "has_obstacle,data_available,empty_reason,obstacle_source,obstacle_frame,"
           "obstacle_x,obstacle_y,obstacle_z,obstacle_size_x,obstacle_size_y,obstacle_size_z,"
           "obstacle_min_x,obstacle_max_x,obstacle_min_y,obstacle_max_y,obstacle_min_z,obstacle_max_z,"
           "safety_margin_m,min_clearance_plan_m,min_clearance_actual_m,collision_detected,collision_stage\n";

    const bool has_obs = row.has_obstacle.value_or(false);
    if (!has_obs) {
      out << "false,false,no_obstacle_for_this_call,,,,,,,,,,,,,,,,,,,\n";
      return;
    }
    const double hx = row.obstacle_size_x / 2.0;
    const double hy = row.obstacle_size_y / 2.0;
    const double hz = row.obstacle_size_z / 2.0;
    const auto minf = [](double c, double h) {
      return std::isfinite(c) && std::isfinite(h) ? metricsFormatDouble(c - h) : std::string();
    };
    const auto maxf = [](double c, double h) {
      return std::isfinite(c) && std::isfinite(h) ? metricsFormatDouble(c + h) : std::string();
    };
    out << "true,true,,"
        << metricsCsvEscape(row.obstacle_source) << ","
        << metricsCsvEscape(row.obstacle_frame) << ","
        << metricsFormatDouble(row.obstacle_center_x) << ","
        << metricsFormatDouble(row.obstacle_center_y) << ","
        << metricsFormatDouble(row.obstacle_center_z) << ","
        << metricsFormatDouble(row.obstacle_size_x) << ","
        << metricsFormatDouble(row.obstacle_size_y) << ","
        << metricsFormatDouble(row.obstacle_size_z) << ","
        << minf(row.obstacle_center_x, hx) << "," << maxf(row.obstacle_center_x, hx) << ","
        << minf(row.obstacle_center_y, hy) << "," << maxf(row.obstacle_center_y, hy) << ","
        << minf(row.obstacle_center_z, hz) << "," << maxf(row.obstacle_center_z, hz) << ","
        << metricsFormatDouble(row.safety_margin_m) << ","
        << metricsFormatDouble(row.min_clearance_m) << ","
        << metricsFormatDouble(row.min_clearance_actual_m) << ","
        << collisionDetected(row) << ","
        << metricsCsvEscape(row.failed_stage) << "\n";
  }

  void writeRlInput15d(
    const ActionMetricsRow & row,
    const std::string & filename,
    const std::string & phase,
    const std::vector<double> & raw_values,
    const std::vector<double> & normalized_values,
    const std::string & source)
  {
    if (row.internal_call_dir.empty() || raw_values.size() != 15 ||
      normalized_values.size() != 15)
    {
      return;
    }
    static const std::array<const char *, 15> kNames = {
      "tcp_x", "tcp_y", "tcp_z",
      "target_x", "target_y", "target_z",
      "err_x", "err_y", "err_z",
      "rel_obs_x", "rel_obs_y", "rel_obs_z",
      "obs_size_x", "obs_size_y", "obs_size_z"
    };
    const std::filesystem::path path = std::filesystem::path(row.internal_call_dir) / filename;
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      logWarn("Failed to write RL input CSV: " + path.string());
      return;
    }
    out << "phase,input_index,input_name,raw_value,normalized_value,unit,source\n";
    for (size_t i = 0; i < kNames.size(); ++i) {
      const std::string unit = i < 9 ? "m" : "normalized";
      out << metricsCsvEscape(phase) << ","
          << i << ","
          << metricsCsvEscape(kNames[i]) << ","
          << metricsFormatDouble(raw_values[i]) << ","
          << metricsFormatDouble(normalized_values[i]) << ","
          << metricsCsvEscape(unit) << ","
          << metricsCsvEscape(source) << "\n";
    }
  }

  void writePlanningTrajectory(
    const ActionMetricsRow & row,
    const std::string & filename,
    const std::string & phase,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const geometry_msgs::msg::Point & target,
    const AabbObstacle * obstacle,
    bool include_phase_column)
  {
    if (row.internal_call_dir.empty() || waypoints.empty()) {
      return;
    }
    const std::filesystem::path path = std::filesystem::path(row.internal_call_dir) / filename;
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      logWarn("Failed to write planning trajectory CSV: " + path.string());
      return;
    }
    if (include_phase_column) {
      out << "phase,waypoint_index,x,y,z,qx,qy,qz,qw,step_distance_m,"
             "cumulative_path_length_m,distance_to_target_m,clearance_to_obstacle_m\n";
    } else {
      out << "waypoint_index,timestamp_iso,x,y,z,qx,qy,qz,qw,step_distance_m,"
             "cumulative_path_length_m,distance_to_target_m,clearance_to_obstacle_m,workspace_valid\n";
    }

    double cumulative = 0.0;
    const std::string timestamp = metricsIsoNow();
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto & pose = waypoints[i];
      double step = 0.0;
      if (i > 0) {
        const auto & prev = waypoints[i - 1].position;
        const double dx = pose.position.x - prev.x;
        const double dy = pose.position.y - prev.y;
        const double dz = pose.position.z - prev.z;
        step = std::sqrt(dx * dx + dy * dy + dz * dz);
        cumulative += step;
      }
      const double tx = pose.position.x - target.x;
      const double ty = pose.position.y - target.y;
      const double tz = pose.position.z - target.z;
      const double distance_to_target = std::sqrt(tx * tx + ty * ty + tz * tz);
      const double clearance =
        (obstacle && obstacle->has_obstacle) ?
        aabbSignedDistance(pose.position, obstacle->center, obstacle->size) :
        std::numeric_limits<double>::quiet_NaN();
      const bool finite_position =
        std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
        std::isfinite(pose.position.z);

      if (include_phase_column) {
        out << metricsCsvEscape(phase) << ",";
      }
      out << i << ",";
      if (!include_phase_column) {
        out << metricsCsvEscape(timestamp) << ",";
      }
      out << metricsFormatDouble(pose.position.x) << ","
          << metricsFormatDouble(pose.position.y) << ","
          << metricsFormatDouble(pose.position.z) << ","
          << metricsFormatDouble(pose.orientation.x) << ","
          << metricsFormatDouble(pose.orientation.y) << ","
          << metricsFormatDouble(pose.orientation.z) << ","
          << metricsFormatDouble(pose.orientation.w) << ","
          << metricsFormatDouble(step) << ","
          << metricsFormatDouble(cumulative) << ","
          << metricsFormatDouble(distance_to_target) << ","
          << metricsFormatDouble(clearance);
      if (!include_phase_column) {
        out << "," << (finite_position ? "true" : "false");
      }
      out << "\n";
    }
  }

  using PoseLookupFn = std::function<bool(geometry_msgs::msg::PoseStamped &, std::string &)>;

  struct TcpExecutionSamplingHandle
  {
    std::shared_ptr<ActionMetricsRow> row;
    std::string stage;
    geometry_msgs::msg::Pose target_pose;
    std::vector<geometry_msgs::msg::Pose> rl_plan;
    std::vector<geometry_msgs::msg::Pose> moveit_path;
    AabbObstacle obstacle;
    bool has_obstacle = false;
    PoseLookupFn lookup_pose;
    double sample_period_s = 0.02;
    std::chrono::steady_clock::time_point start_time;
    std::atomic<bool> stop{false};
    std::mutex samples_mutex;
    std::vector<TcpExecutionSample> samples;
    std::thread worker;
  };

  std::shared_ptr<TcpExecutionSamplingHandle> startTcpExecutionSampling(
    const std::shared_ptr<ActionMetricsRow> & row,
    const std::string & stage,
    const geometry_msgs::msg::Pose & target_pose,
    const std::vector<geometry_msgs::msg::Pose> & rl_plan,
    const std::vector<geometry_msgs::msg::Pose> & moveit_path,
    const AabbObstacle * obstacle,
    const PoseLookupFn & lookup_pose,
    double sample_rate_hz = 50.0)
  {
    if (!row || row->internal_call_dir.empty() || !lookup_pose) {
      return nullptr;
    }
    auto handle = std::make_shared<TcpExecutionSamplingHandle>();
    handle->row = row;
    handle->stage = stage;
    handle->target_pose = target_pose;
    handle->rl_plan = rl_plan;
    handle->moveit_path = moveit_path;
    if (obstacle && obstacle->has_obstacle) {
      handle->obstacle = *obstacle;
      handle->has_obstacle = true;
    }
    handle->lookup_pose = lookup_pose;
    handle->sample_period_s = 1.0 / std::max(1.0, sample_rate_hz);
    handle->start_time = std::chrono::steady_clock::now();
    handle->worker = std::thread([this, handle]() {sampleTcpExecutionLoop(handle);});
    return handle;
  }

  void stopTcpExecutionSampling(const std::shared_ptr<TcpExecutionSamplingHandle> & handle)
  {
    if (!handle) {
      return;
    }
    handle->stop.store(true);
    if (handle->worker.joinable()) {
      handle->worker.join();
    }
    std::vector<TcpExecutionSample> samples;
    {
      std::lock_guard<std::mutex> lock(handle->samples_mutex);
      samples = handle->samples;
    }
    writeTcpExecutionSamples(*handle, samples);
  }

  void writeMoveItExecutionPath(
    const ActionMetricsRow & row,
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::vector<geometry_msgs::msg::Pose> & tcp_path,
    const std::string & empty_reason)
  {
    if (row.internal_call_dir.empty()) {
      return;
    }
    const auto path = std::filesystem::path(row.internal_call_dir) / "moveit_execution_path.csv";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << "data_available,empty_reason,moveit_point_index,t_from_start_s,x,y,z,roll,pitch,yaw,"
           "q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,source_rl_point_index\n";
    const auto & points = trajectory.joint_trajectory.points;
    if (points.empty()) {
      out << "false," << metricsCsvEscape(empty_reason) << ",,,,,,,,,,,,,,,,,,,,,\n";
      return;
    }
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & p = points[i];
      const double t = static_cast<double>(p.time_from_start.sec) +
        static_cast<double>(p.time_from_start.nanosec) * 1e-9;
      geometry_msgs::msg::Pose tcp_pose;
      const bool has_tcp = i < tcp_path.size();
      if (has_tcp) {
        tcp_pose = tcp_path[i];
      }
      const auto rpy = metricsQuatToRpy(tcp_pose.orientation);
      out << "true,," << i << "," << metricsFormatDouble(t) << ","
          << (has_tcp ? metricsFormatDouble(tcp_pose.position.x) : std::string()) << ","
          << (has_tcp ? metricsFormatDouble(tcp_pose.position.y) : std::string()) << ","
          << (has_tcp ? metricsFormatDouble(tcp_pose.position.z) : std::string()) << ","
          << (has_tcp ? metricsFormatDouble(rpy[0]) : std::string()) << ","
          << (has_tcp ? metricsFormatDouble(rpy[1]) : std::string()) << ","
          << (has_tcp ? metricsFormatDouble(rpy[2]) : std::string());
      for (int k = 0; k < 6; ++k) {
        out << "," << (k < static_cast<int>(p.positions.size()) ? metricsFormatDouble(p.positions[k]) : std::string());
      }
      for (int k = 0; k < 6; ++k) {
        out << "," << (k < static_cast<int>(p.velocities.size()) ? metricsFormatDouble(p.velocities[k]) : std::string());
      }
      out << ",not_applicable\n";
    }
  }

private:
  static std::string header()
  {
    return
      "runtime_mode,log_group,action_name,run_id,action_call_id,parent_action_call_id,goal_uuid,"
      "start_timestamp_iso,end_timestamp_iso,execute_requested,success,failed_stage,failure_reason,message,total_time_s,"
      "planning_time_rl_s,planning_time_moveit_s,total_planning_time_s,execution_time_s,"
      "rl_waypoint_count,moveit_waypoint_count,path_length_rl_m,path_length_moveit_m,"
      "straight_line_distance_m,path_efficiency,final_position_error_m,final_orientation_error_rad,"
      "target_x,target_y,target_z,target_qx,target_qy,target_qz,target_qw,"
      "start_tcp_x,start_tcp_y,start_tcp_z,start_tcp_qx,start_tcp_qy,start_tcp_qz,start_tcp_qw,"
      "final_tcp_x,final_tcp_y,final_tcp_z,final_tcp_qx,final_tcp_qy,final_tcp_qz,final_tcp_qw,"
      "has_obstacle,obstacle_x,obstacle_y,obstacle_z,obstacle_size_x,obstacle_size_y,obstacle_size_z,"
      "min_obstacle_clearance_m,avg_obstacle_clearance_m,collision_detected,workspace_violation,"
      "rl_final_distance_m,rl_rollout_steps,rl_reward_total,rl_action_norm_mean,rl_path_curvature,"
      "rl_smoothness,policy_model_path,algorithm,"
      "timestamp_start,timestamp_end,metrics_action_name,metrics_planner_type,metrics_run_id,metrics_goal_uuid,metrics_source,"
      "metrics_success,planning_success,execution_success,action_result_success,metrics_failed_stage,metrics_message,metrics_execute_requested,"
      "planning_time_s,metrics_execution_time_s,total_action_time_s,vision_wait_time_s,tf_time_s,"
      "planning_scene_update_time_s,drl_plan_time_s,drl_execution_wait_time_s,moveit_plan_time_s,moveit_execute_time_s,"
      "start_x,start_y,start_z,metrics_target_x,metrics_target_y,metrics_target_z,final_x,final_y,final_z,"
      "metrics_final_position_error_m,metrics_final_orientation_error_rad,"
      "first_point_x,first_point_y,first_point_z,distance_start_to_first_point,start_source,"
      "metrics_has_obstacle,obstacle_source,obstacle_center_x,obstacle_center_y,obstacle_center_z,"
      "metrics_obstacle_size_x,metrics_obstacle_size_y,metrics_obstacle_size_z,obstacle_class,obstacle_frame,"
      "obstacle_added_to_planning_scene,rl_has_obstacle,"
      "manual_default_obstacle_size_x,manual_default_obstacle_size_y,manual_default_obstacle_size_z,"
      "planning_scene_object_id,collision_object_added,collision_object_removed,"
      "trajectory_points,path_length_m,metrics_straight_line_distance_m,metrics_path_efficiency,"
      "min_clearance_m,average_clearance_m,start_clearance_m,target_clearance_m,"
      "clearance_violation_count,collision_or_inside_obstacle_count,safety_margin_m,"
      "min_clearance_with_margin_m,clearance_ok,"
      "workspace_violation_count,workspace_min_x,workspace_min_y,workspace_min_z,"
      "workspace_max_x,workspace_max_y,workspace_max_z,"
      "rl_model_name,rl_algorithm,rl_action_step,rl_max_steps,rl_converged,metrics_rl_rollout_steps,"
      "metrics_rl_reward_total,rl_policy_obstacle,"
      "moveit_planning_group,moveit_planner_id,moveit_num_planning_attempts,"
      "moveit_allowed_planning_time_s,moveit_error_code,"
      "actual_path_length_m,rmse_tracking_to_rl_path_m,rmse_tracking_to_moveit_path_m,"
      "max_tracking_error_to_rl_path_m,max_tracking_error_to_moveit_path_m,min_clearance_actual_m\n";
  }

  static std::string rowToCsv(const ActionMetricsRow & r)
  {
    std::ostringstream out;
    const std::string canonical = canonicalLogActionName(r.action_name);
    out << metricsCsvEscape(runtimeModeFromPath(r.internal_call_dir)) << ","
        << metricsCsvEscape(logGroupForAction(canonical)) << ","
        << metricsCsvEscape(canonical) << ","
        << metricsCsvEscape(r.run_id) << ","
        << metricsCsvEscape(r.action_call_id) << ","
        << metricsCsvEscape(r.parent_action_call_id) << ","
        << metricsCsvEscape(r.goal_uuid) << ","
        << metricsCsvEscape(r.timestamp_start_iso) << ","
        << metricsCsvEscape(r.timestamp_end_iso) << ","
        << metricsFormatOptBool(r.execute_requested) << ","
        << metricsFormatOptBool(r.success) << ","
        << metricsCsvEscape(r.failed_stage) << ","
        << metricsCsvEscape(r.failed_stage) << ","
        << metricsCsvEscape(r.message) << ","
        << metricsFormatDouble(r.total_action_time_s) << ","
        << metricsFormatDouble(r.drl_plan_time_s) << ","
        << metricsFormatDouble(r.moveit_plan_time_s) << ","
        << metricsFormatDouble(firstFinite({r.planning_time_s, r.drl_plan_time_s + r.moveit_plan_time_s})) << ","
        << metricsFormatDouble(r.execution_time_s) << ","
        << metricsFormatCount(r.trajectory_points) << ","
        << ","  // moveit_waypoint_count: not carried by this row today.
        << metricsFormatDouble(r.path_length_m) << ","
        << ","  // path_length_moveit_m
        << metricsFormatDouble(r.straight_line_distance_m) << ","
        << metricsFormatDouble(r.path_efficiency) << ","
        << metricsFormatDouble(r.final_position_error_m) << ","
        << metricsFormatDouble(r.final_orientation_error_rad) << ","
        << metricsFormatDouble(r.target_x) << "," << metricsFormatDouble(r.target_y) << ","
        << metricsFormatDouble(r.target_z) << ",,,,,"
        << metricsFormatDouble(r.start_x) << "," << metricsFormatDouble(r.start_y) << ","
        << metricsFormatDouble(r.start_z) << ",,,,,"
        << metricsFormatDouble(r.final_x) << "," << metricsFormatDouble(r.final_y) << ","
        << metricsFormatDouble(r.final_z) << ",,,,,"
        << metricsFormatOptBool(r.has_obstacle) << ","
        << metricsFormatDouble(r.obstacle_center_x) << ","
        << metricsFormatDouble(r.obstacle_center_y) << ","
        << metricsFormatDouble(r.obstacle_center_z) << ","
        << metricsFormatDouble(r.obstacle_size_x) << ","
        << metricsFormatDouble(r.obstacle_size_y) << ","
        << metricsFormatDouble(r.obstacle_size_z) << ","
        << metricsFormatDouble(r.min_clearance_m) << ","
        << metricsFormatDouble(r.average_clearance_m) << ","
        << collisionDetected(r) << ","
        << workspaceViolated(r) << ","
        << ","  // rl_final_distance_m: planner status string not parsed into the row.
        << metricsFormatCount(r.rl_rollout_steps) << ","
        << metricsFormatDouble(r.rl_reward_total) << ","
        << ",,,"  // action_norm_mean, curvature, smoothness unavailable here.
        << metricsCsvEscape(r.rl_model_name) << ","
        << metricsCsvEscape(r.rl_algorithm) << ",";

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
        << metricsFormatCount(r.moveit_error_code) << ","
        << metricsFormatDouble(r.actual_path_length_m) << ","
        << metricsFormatDouble(r.rmse_tracking_to_rl_path_m) << ","
        << metricsFormatDouble(r.rmse_tracking_to_moveit_path_m) << ","
        << metricsFormatDouble(r.max_tracking_error_to_rl_path_m) << ","
        << metricsFormatDouble(r.max_tracking_error_to_moveit_path_m) << ","
        << metricsFormatDouble(r.min_clearance_actual_m) << "\n";
    return out.str();
  }

  std::string base_dir_;
  std::string run_dir_;
  std::string run_id_;
  std::string use_mock_ = "unknown";
  std::string hardware_plugin_ = "unknown";
  bool plots_enabled_ = true;  // codex.md §5
  std::mutex mutex_;
  uint64_t next_index_ = 0;
  rclcpp::Logger logger_{rclcpp::get_logger("ActionMetricsLogger")};
  bool has_logger_ = false;

  static double nearestPathError(
    const geometry_msgs::msg::Point & p,
    const std::vector<geometry_msgs::msg::Pose> & path,
    geometry_msgs::msg::Point & nearest,
    bool & has_nearest)
  {
    has_nearest = false;
    if (path.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto & pose : path) {
      const double d = metricsPointDistance(p, pose.position);
      if (d < best) {
        best = d;
        nearest = pose.position;
        has_nearest = true;
      }
    }
    return has_nearest ? best : std::numeric_limits<double>::quiet_NaN();
  }

  void sampleTcpExecutionLoop(const std::shared_ptr<TcpExecutionSamplingHandle> & handle)
  {
    geometry_msgs::msg::Point last_point;
    bool have_last = false;
    double path_length = 0.0;
    while (rclcpp::ok() && !handle->stop.load()) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      std::string error;
      if (handle->lookup_pose(pose_stamped, error)) {
        TcpExecutionSample sample;
        sample.t_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - handle->start_time).count();
        sample.stage = handle->stage;
        sample.pose = pose_stamped.pose;
        sample.distance_to_target =
          metricsPointDistance(sample.pose.position, handle->target_pose.position);
        sample.tracking_error_to_rl_path_m = nearestPathError(
          sample.pose.position, handle->rl_plan, sample.nearest_rl_plan, sample.has_nearest_rl);
        sample.tracking_error_to_moveit_path_m = nearestPathError(
          sample.pose.position, handle->moveit_path, sample.nearest_moveit, sample.has_nearest_moveit);
        if (handle->has_obstacle) {
          sample.distance_to_obstacle =
            aabbSignedDistance(sample.pose.position, handle->obstacle.center, handle->obstacle.size);
        }
        if (have_last) {
          path_length += metricsPointDistance(sample.pose.position, last_point);
        }
        sample.path_length_actual_so_far_m = path_length;
        last_point = sample.pose.position;
        have_last = true;
        {
          std::lock_guard<std::mutex> lock(handle->samples_mutex);
          handle->samples.push_back(sample);
        }
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(handle->sample_period_s));
    }
  }

  void writeTcpExecutionSamples(
    const TcpExecutionSamplingHandle & handle,
    const std::vector<TcpExecutionSample> & samples)
  {
    if (!handle.row || handle.row->internal_call_dir.empty()) {
      return;
    }
    const auto dir = std::filesystem::path(handle.row->internal_call_dir);
    const auto tcp_path = dir / "tcp_tracking.csv";
    const auto err_path = dir / "error_tracking.csv";
    const bool append_tcp = hasRealDataRows(tcp_path);
    const bool append_err = hasRealDataRows(err_path);
    {
      std::ofstream out(tcp_path, std::ios::out | (append_tcp ? std::ios::app : std::ios::trunc));
      if (!out.is_open()) {
        return;
      }
      if (!append_tcp) {
        out << "data_available,empty_reason,t_s,stage,tcp_x_actual,tcp_y_actual,tcp_z_actual,"
               "tcp_roll_actual,tcp_pitch_actual,tcp_yaw_actual,target_x,target_y,target_z,"
               "target_roll,target_pitch,target_yaw,distance_to_target,nearest_rl_plan_x,"
               "nearest_rl_plan_y,nearest_rl_plan_z,nearest_moveit_x,nearest_moveit_y,"
               "nearest_moveit_z,tracking_error_to_rl_path_m,tracking_error_to_moveit_path_m,"
               "distance_to_obstacle,path_length_actual_so_far_m\n";
      }
      if (samples.empty()) {
        out << "false,execute_tcp_tf_lookup_unavailable,,,,,,,,,,,,,,,,,,,,,,,,,\n";
      } else {
        const auto target_rpy = metricsQuatToRpy(handle.target_pose.orientation);
        for (const auto & s : samples) {
          const auto rpy = metricsQuatToRpy(s.pose.orientation);
          out << "true,,"
              << metricsFormatDouble(s.t_s) << ","
              << metricsCsvEscape(s.stage) << ","
              << metricsFormatDouble(s.pose.position.x) << ","
              << metricsFormatDouble(s.pose.position.y) << ","
              << metricsFormatDouble(s.pose.position.z) << ","
              << metricsFormatDouble(rpy[0]) << ","
              << metricsFormatDouble(rpy[1]) << ","
              << metricsFormatDouble(rpy[2]) << ","
              << metricsFormatDouble(handle.target_pose.position.x) << ","
              << metricsFormatDouble(handle.target_pose.position.y) << ","
              << metricsFormatDouble(handle.target_pose.position.z) << ","
              << metricsFormatDouble(target_rpy[0]) << ","
              << metricsFormatDouble(target_rpy[1]) << ","
              << metricsFormatDouble(target_rpy[2]) << ","
              << metricsFormatDouble(s.distance_to_target) << ","
              << (s.has_nearest_rl ? metricsFormatDouble(s.nearest_rl_plan.x) : std::string()) << ","
              << (s.has_nearest_rl ? metricsFormatDouble(s.nearest_rl_plan.y) : std::string()) << ","
              << (s.has_nearest_rl ? metricsFormatDouble(s.nearest_rl_plan.z) : std::string()) << ","
              << (s.has_nearest_moveit ? metricsFormatDouble(s.nearest_moveit.x) : std::string()) << ","
              << (s.has_nearest_moveit ? metricsFormatDouble(s.nearest_moveit.y) : std::string()) << ","
              << (s.has_nearest_moveit ? metricsFormatDouble(s.nearest_moveit.z) : std::string()) << ","
              << metricsFormatDouble(s.tracking_error_to_rl_path_m) << ","
              << metricsFormatDouble(s.tracking_error_to_moveit_path_m) << ","
              << metricsFormatDouble(s.distance_to_obstacle) << ","
              << metricsFormatDouble(s.path_length_actual_so_far_m) << "\n";
        }
      }
    }
    {
      std::ofstream out(err_path, std::ios::out | (append_err ? std::ios::app : std::ios::trunc));
      if (!out.is_open()) {
        return;
      }
      if (!append_err) {
        out << "data_available,empty_reason,t_s,stage,distance_to_target,"
               "tracking_error_to_rl_path_m,tracking_error_to_moveit_path_m,distance_to_obstacle\n";
      }
      if (samples.empty()) {
        out << "false,execute_tcp_tf_lookup_unavailable,,,,,,\n";
      } else {
        for (const auto & s : samples) {
          out << "true,,"
              << metricsFormatDouble(s.t_s) << ","
              << metricsCsvEscape(s.stage) << ","
              << metricsFormatDouble(s.distance_to_target) << ","
              << metricsFormatDouble(s.tracking_error_to_rl_path_m) << ","
              << metricsFormatDouble(s.tracking_error_to_moveit_path_m) << ","
              << metricsFormatDouble(s.distance_to_obstacle) << "\n";
        }
      }
    }

    if (!samples.empty()) {
      auto & row = *handle.row;
      row.actual_path_length_m = std::isfinite(row.actual_path_length_m) ?
        row.actual_path_length_m + samples.back().path_length_actual_so_far_m :
        samples.back().path_length_actual_so_far_m;
      auto accumulate = [](const std::vector<TcpExecutionSample> & values, auto field) {
        double sum_sq = 0.0;
        double max_v = 0.0;
        int count = 0;
        for (const auto & s : values) {
          const double v = field(s);
          if (std::isfinite(v)) {
            sum_sq += v * v;
            max_v = std::max(max_v, v);
            ++count;
          }
        }
        return std::tuple<double, double, int>{
          count > 0 ? std::sqrt(sum_sq / static_cast<double>(count)) :
            std::numeric_limits<double>::quiet_NaN(),
          count > 0 ? max_v : std::numeric_limits<double>::quiet_NaN(),
          count};
      };
      const auto [rl_rmse, rl_max, rl_count] = accumulate(
        samples, [](const TcpExecutionSample & s) {return s.tracking_error_to_rl_path_m;});
      const auto [mv_rmse, mv_max, mv_count] = accumulate(
        samples, [](const TcpExecutionSample & s) {return s.tracking_error_to_moveit_path_m;});
      if (rl_count > 0) {
        row.rmse_tracking_to_rl_path_m = rl_rmse;
        row.max_tracking_error_to_rl_path_m = rl_max;
      }
      if (mv_count > 0) {
        row.rmse_tracking_to_moveit_path_m = mv_rmse;
        row.max_tracking_error_to_moveit_path_m = mv_max;
      }
      for (const auto & s : samples) {
        if (std::isfinite(s.distance_to_obstacle)) {
          row.min_clearance_actual_m = std::isfinite(row.min_clearance_actual_m) ?
            std::min(row.min_clearance_actual_m, s.distance_to_obstacle) :
            s.distance_to_obstacle;
        }
      }
    }
  }

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

  static double firstFinite(std::initializer_list<double> values)
  {
    for (double value : values) {
      if (std::isfinite(value)) {
        return value;
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  static bool hasRealDataRows(const std::filesystem::path & path)
  {
    std::ifstream in(path);
    if (!in.is_open()) {
      return false;
    }
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
      if (line.rfind("true,", 0) == 0) {
        return true;
      }
    }
    return false;
  }

  static std::string collisionDetected(const ActionMetricsRow & r)
  {
    if (std::isfinite(r.collision_or_inside_obstacle_count)) {
      return r.collision_or_inside_obstacle_count > 0.0 ? "true" : "false";
    }
    return "";
  }

  static std::string workspaceViolated(const ActionMetricsRow & r)
  {
    if (std::isfinite(r.workspace_violation_count)) {
      return r.workspace_violation_count > 0.0 ? "true" : "false";
    }
    return "";
  }

  static std::string runtimeModeFromPath(const std::string & path)
  {
    const std::filesystem::path p(path);
    for (const auto & part : p) {
      const auto s = part.string();
      if (s == "mock" || s == "real") {
        return s;
      }
    }
    return "mock";
  }

  void appendEvent(
    const ActionMetricsRow & row,
    const std::string & stage,
    const std::string & event_type,
    const std::optional<bool> & success,
    const std::string & message) const
  {
    if (row.internal_call_dir.empty()) {
      return;
    }
    const std::filesystem::path path = std::filesystem::path(row.internal_call_dir) / "events.csv";
    const bool new_file = !std::filesystem::exists(path);
    std::ofstream out(path, std::ios::out | std::ios::app);
    if (!out.is_open()) {
      return;
    }
    if (new_file) {
      out << "timestamp_iso,t_rel_sec,stage,event_type,success,message\n";
    }
    out << metricsCsvEscape(metricsIsoNow()) << ",,"
        << metricsCsvEscape(stage) << ","
        << metricsCsvEscape(event_type) << ",";
    if (success.has_value()) {
      out << (*success ? "true" : "false");
    }
    out << "," << metricsCsvEscape(message) << "\n";
  }

  void writeMetadata(const ActionMetricsRow & row) const
  {
    const std::filesystem::path path = std::filesystem::path(row.internal_call_dir) / "metadata.json";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    const std::string canonical = canonicalLogActionName(row.action_name);
    const std::string hw_mode = hardwareModeFromPath(row.internal_call_dir);
    out << "{\n"
        << "  \"runtime_mode\": \"" << hw_mode << "\",\n"
        // codex.md §3.1 fields:
        << "  \"hardware_mode\": \"" << hw_mode << "\",\n"
        << "  \"use_mock\": \"" << use_mock_ << "\",\n"
        << "  \"hardware_plugin\": \"" << hardware_plugin_ << "\",\n"
        << "  \"log_root_dir\": \"" << logRootFromPath(row.internal_call_dir) << "\",\n"
        << "  \"evaluation_group\": \"" << evalGroupForAction(canonical) << "\",\n"
        << "  \"data_status\": \"see summary.csv (success/data columns)\",\n"
        << "  \"sample_rate_hz\": \"\",\n"
        << "  \"log_group\": \"" << logGroupForAction(canonical) << "\",\n"
        << "  \"action_name\": \"" << canonical << "\",\n"
        << "  \"run_id\": \"" << row.run_id << "\",\n"
        << "  \"action_call_id\": \"" << row.action_call_id << "\",\n"
        << "  \"parent_action_call_id\": \"" << row.parent_action_call_id << "\",\n"
        << "  \"goal_uuid\": \"" << row.goal_uuid << "\",\n"
        << "  \"robot_model\": \"\",\n"
        << "  \"base_frame\": \"\",\n"
        << "  \"tcp_frame\": \"\",\n"
        << "  \"hardware_backend\": \"\",\n"
        << "  \"vision_source\": \"" << row.source << "\",\n"
        << "  \"planner_type\": \"" << row.planner_type << "\",\n"
        << "  \"model_path\": \"" << row.rl_model_name << "\",\n"
        << "  \"created_by_node\": \"action_metrics_logger\",\n";
    for (const auto & kv : row.extra_metadata) {
      out << "  \"" << kv.first << "\": \"" << kv.second << "\",\n";
    }
    out << "  \"launch_context\": \"\"\n"
        << "}\n";
  }

  void writeActionSpecificPlaceholders(const ActionMetricsRow & row) const
  {
    const auto dir = std::filesystem::path(row.internal_call_dir);
    const auto canonical = canonicalLogActionName(row.action_name);
    const auto write = [&dir](const std::string & name, const std::string & header) {
      std::ofstream out(dir / name, std::ios::out | std::ios::trunc);
      if (out.is_open()) {
        out << header << "\n";
      }
    };
    // codex.md §9: only create placeholder files that have a REAL writer
    // reached during a normal (plan-success) call. Files that never had a
    // writer (planning_moveit_from_rl.csv, trajectory_tracking.csv,
    // obstacle.csv, object_obstacle.csv) are no longer created as header-only
    // stubs. rl_input_15d*/planning_* below are filled by
    // writeRlInput15d()/writePlanningTrajectory() when the plan succeeds.
    // codex.md §5.2: rl_input_15d.csv is the RL observation file; renamed to
    // rl_observation.csv (schema unchanged: obs_index/name/raw/normalized/...).
    if (canonical == "move_pose_rl" || canonical == "move_target_rl") {
      write("rl_observation.csv", "phase,input_index,input_name,raw_value,normalized_value,unit,source");
      write("rl_planning_path.csv", "waypoint_index,timestamp_iso,x,y,z,qx,qy,qz,qw,step_distance_m,cumulative_path_length_m,distance_to_target_m,clearance_to_obstacle_m,workspace_valid");
    } else if (canonical == "pick_place_rl") {
      write("rl_observation_pick.csv", "phase,input_index,input_name,raw_value,normalized_value,unit,source");
      write("rl_observation_place.csv", "phase,input_index,input_name,raw_value,normalized_value,unit,source");
      write("planning_pick.csv", "phase,waypoint_index,x,y,z,qx,qy,qz,qw,step_distance_m,cumulative_path_length_m,distance_to_target_m,clearance_to_obstacle_m");
      write("planning_place.csv", "phase,waypoint_index,x,y,z,qx,qy,qz,qw,step_distance_m,cumulative_path_length_m,distance_to_target_m,clearance_to_obstacle_m");
    }
  }
};

}  // namespace robot_task_manager
