#pragma once

// PerCallTcpLogger — shared, per-call TCP set/actual CSV logger.
//
// Generalized from the per-call CSV logger originally written just for
// /pickplace (PickPlaceTcpLogger, still living standalone in
// pickplace_server.cpp — deliberately left untouched, see
// Report/per_action_tcp_logger_report.md section 2 for why). This class is
// the same idea made reusable for any other action server that wants an
// opt-in ("enable_tcp_log=true" on the goal), one-file-per-call CSV of TCP
// set-vs-actual pose: MoveToPose, MoveToPoseCartesian, RepeatabilityTest,
// CheckerBoard.
//
// Deliberately NOT a generic "LogContext" — no callback registries, no
// plugin stages. Just: start a call, tag a stage, log an event, sample in
// the background, finish the call. Any action-specific data (goal fields,
// loop indices, etc.) is passed in as an already-formatted JSON string in
// `metadata_json`, kept constant for the whole call — callers build that
// string however is convenient for them.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <map>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/log_plot_hook.hpp"
#include "robot_task_manager/standard_action_logger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"

namespace robot_task_manager
{

// codex.md §2.2/§2.3: quaternion -> roll/pitch/yaw (rad). RPY is the primary
// TCP orientation representation in the logs; quaternions are internal only.
inline void quatToRpy(
  const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  if (tq.length2() <= 1e-12) {
    roll = pitch = yaw = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  tq.normalize();
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
}

class PerCallTcpLogger
{
public:
  // Repeatability-specific summary values that only the action server knows
  // (goal fields + per-repeat success/fail accounting). Passed in explicitly
  // via setRepeatabilitySummary() so writeSummary() can fill summary.csv
  // instead of leaving those columns blank.
  struct RepeatabilitySummaryInfo
  {
    bool valid = false;
    int axis = -1;
    int repeat_count = 0;
    double offset_m = std::numeric_limits<double>::quiet_NaN();
    int success_count = 0;
    int failed_count = 0;
  };

  struct Call
  {
    uint32_t call_index = 0;
    std::string csv_filename;
    std::string csv_path;
    std::string call_dir;
    std::string action_call_id;
    std::ofstream csv;
    rclcpp::Time start_time;
    std::string start_iso;
    std::atomic<bool> sampling{false};
    std::thread sample_thread;
    std::mutex state_mutex;
    std::string current_stage;
    geometry_msgs::msg::Pose current_set_pose;
    bool has_set_pose = false;
    uint64_t row_count = 0;
    std::string metadata_json;

    // codex.md §2.1/§2.3/§2.4: extra evaluation files, written in parallel with
    // the TCP tracking file during sampling.
    std::ofstream error_csv;   // error_tracking.csv
    std::ofstream joint_csv;   // joint_tracking.csv
    bool joint_available = false;  // saw at least one /joint_states sample

    // Running stats for trajectory_metrics.csv / summary.csv (accumulated over
    // samples so no second pass over the file is needed).
    uint64_t err_count = 0;
    double sum_pos_err2 = 0.0;
    double sum_ori_err2 = 0.0;
    double max_pos_err = 0.0;
    double max_ori_err = 0.0;
    double sum_joint_err2 = 0.0;
    double max_joint_err = 0.0;
    uint64_t joint_err_count = 0;
    double path_length_actual = 0.0;
    bool has_last_actual = false;
    geometry_msgs::msg::Point last_actual_pos;
    double final_pos_err = std::numeric_limits<double>::quiet_NaN();
    double final_ori_err = std::numeric_limits<double>::quiet_NaN();
    geometry_msgs::msg::Point first_actual_pos;
    bool has_first_actual = false;

    // Explicit repeat index for RepeatabilityTest, set by the action server
    // before each repeat loop. Rows are routed to repeat_<index>.csv based on
    // this value instead of parsing it out of the stage string.
    std::atomic<int> repeat_index{1};
    RepeatabilitySummaryInfo repeat_summary;
  };

  PerCallTcpLogger(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    std::string log_dir,
    double sample_rate_hz,
    std::string base_frame,
    std::string tcp_frame,
    std::string file_prefix,
    std::string action_name)
  : node_(node),
    tf_buffer_(tf_buffer),
    log_dir_(std::filesystem::absolute(std::filesystem::path(std::move(log_dir))).lexically_normal().string()),
    sample_rate_hz_(sample_rate_hz > 0.0 ? sample_rate_hz : 50.0),
    base_frame_(std::move(base_frame)),
    tcp_frame_(std::move(tcp_frame)),
    file_prefix_(std::move(file_prefix)),
    action_name_(std::move(action_name))
  {
    prepareRunDir();
    // codex.md §2.1: subscribe to /joint_states for the actual joint positions
    // and velocities logged in joint_tracking.csv. Best-effort: if nothing is
    // published, joint_tracking.csv is written with data_available=false.
    if (node_) {
      auto qos = rclcpp::SensorDataQoS();
      joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_mutex_);
          latest_joint_ = *msg;
          have_joint_ = true;
        });
      // codex.md §4: q_set/dq_set source. MoveItExecutor publishes the planned
      // joint trajectory here; we interpolate the setpoint by elapsed time.
      planned_traj_sub_ = node_->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        "/robot_task_manager/last_planned_joint_trajectory", rclcpp::QoS(10).reliable(),
        [this](const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(planned_mutex_);
          latest_planned_traj_ = *msg;
          planned_recv_time_ = node_->get_clock()->now();
          have_planned_ = true;
        });
    }
  }

  ~PerCallTcpLogger() = default;

  // codex.md §6: hardware provenance for metadata.json. Optional; defaults keep
  // "unknown" if the server does not call this.
  void setHardwareInfo(const std::string & use_mock, const std::string & hardware_plugin)
  {
    use_mock_ = use_mock;
    hardware_plugin_ = hardware_plugin;
  }

  // codex.md §5/§7: enable/disable the post-finish verification plots. Default
  // is true (mock/sim). Servers may set it from the enable_log_plots param.
  void setPlotsEnabled(bool enabled) {plots_enabled_ = enabled;}

  std::shared_ptr<Call> startCall(const std::string & metadata_json = "")
  {
    if (!ready_) {
      return nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto call = std::make_shared<Call>();
    call->call_index = ++next_call_index_;
    call->action_call_id = callId(call->call_index);
    call->start_time = node_->get_clock()->now();
    call->start_iso = isoNow();
    call->metadata_json = metadata_json;

    call->call_dir = (std::filesystem::path(run_dir_) / call->action_call_id).string();
    std::error_code ec;
    std::filesystem::create_directories(call->call_dir, ec);
    if (ec) {
      RCLCPP_WARN(node_->get_logger(), "Failed to create call log directory: %s",
        call->call_dir.c_str());
      return nullptr;
    }
    call->csv_filename = trackingFilename();
    call->csv_path = (std::filesystem::path(call->call_dir) / call->csv_filename).string();

    call->csv.open(call->csv_path, std::ios::out | std::ios::trunc);
    if (!call->csv.is_open()) {
      RCLCPP_WARN(node_->get_logger(), "Failed to write log CSV: %s",
        call->csv_path.c_str());
      RCLCPP_WARN(node_->get_logger(), "PerCallTcpLogger(%s): cannot open %s",
        action_name_.c_str(), call->csv_path.c_str());
      return nullptr;
    }
    writeHeader(call->csv);
    writeMetadata(*call);
    appendEvent(*call, "start", "action_start", "", "Per-call TCP log created");
    writePlaceholders(*call);

    // codex.md §2.3: error_tracking.csv (per-sample error time series).
    call->error_csv.open((std::filesystem::path(call->call_dir) / "error_tracking.csv").string(),
      std::ios::out | std::ios::trunc);
    if (call->error_csv.is_open()) {
      call->error_csv << "t_s,stage,joint_error_norm,max_abs_joint_error,"
                         "tcp_x_error,tcp_y_error,tcp_z_error,tcp_roll_error,tcp_pitch_error,tcp_yaw_error,"
                         "tcp_position_error_norm,tcp_orientation_error_norm,distance_to_target\n";
      call->error_csv.flush();
    }
    // codex.md §2.1: joint_tracking.csv from /joint_states (actual + velocity).
    // CheckerBoard keeps its own server-managed joint_tracking.csv, so skip it
    // here to avoid two writers on the same file.
    if (canonicalActionName() != "move_checkerboard") {
      call->joint_csv.open((std::filesystem::path(call->call_dir) / "joint_tracking.csv").string(),
        std::ios::out | std::ios::trunc);
      if (call->joint_csv.is_open()) {
        call->joint_csv << "t_s,stage,"
                           "q1_set,q2_set,q3_set,q4_set,q5_set,q6_set,"
                           "q1_actual,q2_actual,q3_actual,q4_actual,q5_actual,q6_actual,"
                           "q1_error,q2_error,q3_error,q4_error,q5_error,q6_error,"
                           "dq1_set,dq2_set,dq3_set,dq4_set,dq5_set,dq6_set,"
                           "dq1_actual,dq2_actual,dq3_actual,dq4_actual,dq5_actual,dq6_actual,"
                           "dq1_error,dq2_error,dq3_error,dq4_error,dq5_error,dq6_error,"
                           "joint_error_norm,max_abs_joint_error\n";
        call->joint_csv.flush();
      }
    }
    RCLCPP_INFO(node_->get_logger(), "Executor CSV saved: %s", call->csv_path.c_str());

    // Seed stage/set-pose immediately: no sample should ever have an empty
    // stage or a blank set pose. Default stage is "<prefix>_start"; the
    // initial "set" TCP pose is wherever the robot actually is right now
    // (the implicit target while no explicit motion goal is active yet).
    {
      std::lock_guard<std::mutex> state_lock(call->state_mutex);
      call->current_stage = file_prefix_ + "_start";
    }
    geometry_msgs::msg::Pose seed_pose;
    if (tryLookupActual(seed_pose)) {
      std::lock_guard<std::mutex> state_lock(call->state_mutex);
      call->current_set_pose = seed_pose;
      call->has_set_pose = true;
    }
    return call;
  }

  // Changes only the stage label, keeping whatever set-pose is already
  // tracked (useful for stages that don't have a new TCP target, e.g. a
  // settle/wait period, or a gripper-only action).
  void setStage(const std::shared_ptr<Call> & call, const std::string & stage)
  {
    if (!call) {
      return;
    }
    std::lock_guard<std::mutex> lock(call->state_mutex);
    call->current_stage = stage;
  }

  void updateStage(
    const std::shared_ptr<Call> & call,
    const std::string & stage,
    const geometry_msgs::msg::Pose & set_pose)
  {
    if (!call) {
      return;
    }
    std::lock_guard<std::mutex> lock(call->state_mutex);
    call->current_stage = stage;
    call->current_set_pose = set_pose;
    call->has_set_pose = true;
  }

  // Sets the active repeat index for RepeatabilityTest. Call this once at the
  // start of each repeat loop so every subsequent sample/event row (and the
  // repeat_index CSV column) is attributed to the correct repeat.
  void setRepeatIndex(const std::shared_ptr<Call> & call, int index)
  {
    if (!call) {
      return;
    }
    call->repeat_index.store(index >= 1 ? index : 1);
  }

  // Provides the repeatability summary accounting (axis/offset/repeat counts)
  // that only the action server knows, so finishCall()/writeSummary() can fill
  // the repeatability columns of summary.csv.
  void setRepeatabilitySummary(
    const std::shared_ptr<Call> & call,
    const RepeatabilitySummaryInfo & info)
  {
    if (!call) {
      return;
    }
    std::lock_guard<std::mutex> lock(call->state_mutex);
    call->repeat_summary = info;
    call->repeat_summary.valid = true;
  }

  void logEvent(
    const std::shared_ptr<Call> & call,
    const std::string & stage,
    const std::string & status,
    const std::string & message,
    const geometry_msgs::msg::Pose * set_pose_override = nullptr)
  {
    if (!call) {
      return;
    }
    geometry_msgs::msg::Pose set_pose;
    bool has_set = false;
    if (set_pose_override) {
      set_pose = *set_pose_override;
      has_set = true;
    } else {
      std::lock_guard<std::mutex> lock(call->state_mutex);
      if (call->has_set_pose) {
        set_pose = call->current_set_pose;
        has_set = true;
      }
    }
    writeRow(call, "event", stage, has_set ? &set_pose : nullptr, status, message);
    appendEvent(*call, stage, status, "", message);
  }

  void startSampling(const std::shared_ptr<Call> & call)
  {
    if (!call || call->sampling.load()) {
      return;
    }
    call->sampling.store(true);
    call->sample_thread = std::thread(&PerCallTcpLogger::sampleLoop, this, call);
  }

  void stopSampling(const std::shared_ptr<Call> & call)
  {
    if (!call) {
      return;
    }
    call->sampling.store(false);
    if (call->sample_thread.joinable()) {
      call->sample_thread.join();
    }
  }

  void finishCall(
    const std::shared_ptr<Call> & call,
    const std::string & status,
    bool success,
    const std::string & message)
  {
    if (!call) {
      return;
    }
    stopSampling(call);

    geometry_msgs::msg::Pose set_pose;
    bool has_set = false;
    {
      std::lock_guard<std::mutex> lock(call->state_mutex);
      if (call->has_set_pose) {
        set_pose = call->current_set_pose;
        has_set = true;
      }
    }
    writeRow(call, "summary", file_prefix_ + "_end", has_set ? &set_pose : nullptr, status, message, &success);
    call->csv.flush();
    call->csv.close();
    if (call->error_csv.is_open()) {
      call->error_csv.flush();
      call->error_csv.close();
    }
    if (call->joint_csv.is_open()) {
      call->joint_csv.flush();
      call->joint_csv.close();
    }

    const std::string end_iso = isoNow();
    const double duration_sec = (node_->get_clock()->now() - call->start_time).seconds();
    writeTrajectoryMetrics(*call, duration_sec);
    writeSummary(*call, end_iso, duration_sec, status, success, message);
    appendEvent(*call, file_prefix_ + "_end", "action_result", success ? "true" : "false", message);
    appendIndexRow(*call, end_iso, duration_sec, status, success, message);

    // codex.md §5/§7: fire the verification plots (detached, never blocks/fails).
    runLogPlotsAsync(node_->get_logger(), call->call_dir, plots_enabled_);
  }

  // codex.md §2.4: key-value trajectory metrics derived from the accumulated
  // per-sample stats (no second pass over the CSV).
  void writeTrajectoryMetrics(const Call & call, double /*duration_sec*/)
  {
    std::ofstream out(std::filesystem::path(call.call_dir) / "trajectory_metrics.csv",
      std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    const double n = static_cast<double>(std::max<uint64_t>(1, call.err_count));
    const double rmse_pos = std::sqrt(call.sum_pos_err2 / n);
    const double rmse_ori = std::sqrt(call.sum_ori_err2 / n);
    double straight = std::numeric_limits<double>::quiet_NaN();
    if (call.has_first_actual && call.has_last_actual) {
      const double dx = call.last_actual_pos.x - call.first_actual_pos.x;
      const double dy = call.last_actual_pos.y - call.first_actual_pos.y;
      const double dz = call.last_actual_pos.z - call.first_actual_pos.z;
      straight = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    const double eff = (std::isfinite(straight) && call.path_length_actual > 1e-9) ?
      straight / call.path_length_actual : std::numeric_limits<double>::quiet_NaN();
    out << "metric_name,value,unit,description\n";
    const auto row = [&out](const std::string & n2, double v, const std::string & u,
        const std::string & d) {
      out << n2 << "," << formatDouble(v) << "," << u << "," << d << "\n";
    };
    row("actual_path_length_m", call.path_length_actual, "m", "sum of actual TCP step distances");
    row("straight_line_distance_m", straight, "m", "first->last actual TCP position");
    row("path_efficiency", eff, "ratio", "straight_line/actual_path");
    row("final_position_error_m", call.final_pos_err, "m", "last sample set-vs-actual position error");
    row("final_orientation_error_rad", call.final_ori_err, "rad", "last sample orientation error");
    row("rmse_tcp_position_m", call.err_count ? rmse_pos : std::numeric_limits<double>::quiet_NaN(), "m", "RMSE of position error over samples");
    row("rmse_tcp_orientation_rad", call.err_count ? rmse_ori : std::numeric_limits<double>::quiet_NaN(), "rad", "RMSE of orientation error over samples");
    const double rmse_joint = call.joint_err_count ?
      std::sqrt(call.sum_joint_err2 / static_cast<double>(call.joint_err_count)) :
      std::numeric_limits<double>::quiet_NaN();
    row("rmse_joint_mean", rmse_joint, "rad", "RMSE of joint-error-norm (set from planned trajectory)");
    row("max_tcp_position_error_m", call.err_count ? call.max_pos_err : std::numeric_limits<double>::quiet_NaN(), "m", "max position error over samples");
    row("max_abs_joint_error", call.joint_err_count ? call.max_joint_err : std::numeric_limits<double>::quiet_NaN(), "rad", "max |joint error| over samples (NaN if no plan published)");
    row("settling_time_s", std::numeric_limits<double>::quiet_NaN(), "s", "not computed in this version");
    if (canonicalActionName() == "repeatability_test" && call.repeat_summary.valid) {
      row("repeat_count", call.repeat_summary.repeat_count, "count", "requested repeats");
      row("repeatability_max_error_m", call.max_pos_err, "m", "max position error across repeats");
    }
  }

private:
  bool tryLookupActual(geometry_msgs::msg::Pose & out, std::string * warning = nullptr)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, tcp_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      out.position.x = tf.transform.translation.x;
      out.position.y = tf.transform.translation.y;
      out.position.z = tf.transform.translation.z;
      out.orientation = tf.transform.rotation;
      return true;
    } catch (const std::exception & e) {
      if (warning) {
        *warning = std::string("TF unavailable: ") + e.what();
      }
      return false;
    }
  }

  void sampleLoop(std::shared_ptr<Call> call)
  {
    const double period_sec = 1.0 / sample_rate_hz_;
    while (call->sampling.load() && rclcpp::ok()) {
      const auto loop_start = std::chrono::steady_clock::now();

      std::string stage;
      geometry_msgs::msg::Pose set_pose;
      bool has_set = false;
      {
        std::lock_guard<std::mutex> lock(call->state_mutex);
        stage = call->current_stage;
        if (call->has_set_pose) {
          set_pose = call->current_set_pose;
          has_set = true;
        }
      }

      try {
        writeRow(call, "sample", stage, has_set ? &set_pose : nullptr, "", "");
      } catch (const std::exception &) {
        // Skip this one sample and keep going.
      }

      const auto elapsed = std::chrono::steady_clock::now() - loop_start;
      const auto sleep_for = std::chrono::duration<double>(period_sec) - elapsed;
      if (sleep_for > std::chrono::duration<double>(0)) {
        std::this_thread::sleep_for(sleep_for);
      }
    }
  }

  void writeRow(
    const std::shared_ptr<Call> & call,
    const std::string &,
    const std::string & stage,
    const geometry_msgs::msg::Pose * set_pose,
    const std::string & status,
    const std::string & message,
    const bool * success = nullptr)
  {
    geometry_msgs::msg::Pose actual_pose;
    std::string tf_warning;
    const bool have_actual = tryLookupActual(actual_pose, &tf_warning);

    const double time_sec = (node_->get_clock()->now() - call->start_time).seconds();
    const bool is_repeatability = canonicalActionName() == "repeatability_test";
    const int repeat_index = is_repeatability ? std::max(1, call->repeat_index.load()) : 0;

    // --- codex.md §2.2: xyz + RPY set/actual/error -----------------------
    double sr = kNan, sp = kNan, sy = kNan, ar = kNan, ap = kNan, ay = kNan;
    if (set_pose) {
      quatToRpy(set_pose->orientation, sr, sp, sy);
    }
    if (have_actual) {
      quatToRpy(actual_pose.orientation, ar, ap, ay);
    }
    double ex = kNan, ey = kNan, ez = kNan, er = kNan, ep = kNan, eyaw = kNan;
    double pos_norm = kNan, ori_err = kNan, dist_to_target = kNan;
    if (set_pose && have_actual) {
      ex = actual_pose.position.x - set_pose->position.x;
      ey = actual_pose.position.y - set_pose->position.y;
      ez = actual_pose.position.z - set_pose->position.z;
      er = ar - sr; ep = ap - sp; eyaw = ay - sy;
      pos_norm = std::sqrt(ex * ex + ey * ey + ez * ez);
      ori_err = orientationErrorRad(set_pose->orientation, actual_pose.orientation);
      dist_to_target = pos_norm;
    }

    // --- actual joints (codex.md §2.1) + planned setpoint (codex.md §4) ---
    std::vector<double> qpos, qvel;
    bool joints_ok = false;
    currentJoints(qpos, qvel, joints_ok);
    std::vector<double> qset, dqset;
    bool set_ok = false;
    {
      double elapsed = 0.0;
      {
        std::lock_guard<std::mutex> plock(planned_mutex_);
        if (have_planned_) {
          elapsed = (node_->get_clock()->now() - planned_recv_time_).seconds();
        }
      }
      plannedSetpoint(elapsed, qset, dqset, set_ok);
    }
    // Per-joint error + norm (only where both set and actual are finite).
    std::vector<double> qerr(6, std::numeric_limits<double>::quiet_NaN());
    double joint_err_norm = std::numeric_limits<double>::quiet_NaN();
    double joint_max_abs = std::numeric_limits<double>::quiet_NaN();
    if (set_ok && joints_ok) {
      double sumsq = 0.0, maxa = 0.0;
      int cnt = 0;
      for (int k = 0; k < 6; ++k) {
        if (std::isfinite(qset[k]) && std::isfinite(qpos[k])) {
          qerr[k] = qpos[k] - qset[k];
          sumsq += qerr[k] * qerr[k];
          maxa = std::max(maxa, std::abs(qerr[k]));
          ++cnt;
        }
      }
      if (cnt > 0) {
        joint_err_norm = std::sqrt(sumsq);
        joint_max_abs = maxa;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (std::isfinite(joint_err_norm)) {
      call->sum_joint_err2 += joint_err_norm * joint_err_norm;
      call->max_joint_err = std::max(call->max_joint_err, joint_max_abs);
      ++call->joint_err_count;
    }

    // Accumulate path length + running error stats for trajectory_metrics/summary.
    if (have_actual) {
      if (call->has_last_actual) {
        const double dx = actual_pose.position.x - call->last_actual_pos.x;
        const double dy = actual_pose.position.y - call->last_actual_pos.y;
        const double dz = actual_pose.position.z - call->last_actual_pos.z;
        call->path_length_actual += std::sqrt(dx * dx + dy * dy + dz * dz);
      } else {
        call->first_actual_pos = actual_pose.position;
        call->has_first_actual = true;
      }
      call->last_actual_pos = actual_pose.position;
      call->has_last_actual = true;
    }
    if (std::isfinite(pos_norm)) {
      ++call->err_count;
      call->sum_pos_err2 += pos_norm * pos_norm;
      call->max_pos_err = std::max(call->max_pos_err, pos_norm);
      call->final_pos_err = pos_norm;
    }
    if (std::isfinite(ori_err)) {
      call->sum_ori_err2 += ori_err * ori_err;
      call->max_ori_err = std::max(call->max_ori_err, ori_err);
      call->final_ori_err = ori_err;
    }
    if (joints_ok) {
      call->joint_available = true;
    }

    const double path_so_far = call->path_length_actual;

    // --- main tcp_tracking row -------------------------------------------
    std::ostringstream row;
    row << formatDouble(time_sec) << ",";
    if (is_repeatability) {
      row << repeat_index << ",";
    }
    row << csvEscape(stage) << ","
        << f(set_pose ? set_pose->position.x : kNan) << "," << f(set_pose ? set_pose->position.y : kNan) << ","
        << f(set_pose ? set_pose->position.z : kNan) << "," << f(sr) << "," << f(sp) << "," << f(sy) << ","
        << f(have_actual ? actual_pose.position.x : kNan) << "," << f(have_actual ? actual_pose.position.y : kNan) << ","
        << f(have_actual ? actual_pose.position.z : kNan) << "," << f(ar) << "," << f(ap) << "," << f(ay) << ","
        << f(ex) << "," << f(ey) << "," << f(ez) << "," << f(er) << "," << f(ep) << "," << f(eyaw) << ","
        << f(pos_norm) << "," << f(ori_err) << "," << f(dist_to_target) << "," << f(path_so_far) << ","
        << f(pos_norm) << "," << f(ori_err) << ",";  // position_error_m,orientation_error_rad (parser aliases)

    std::string full_message = message;
    if (!tf_warning.empty()) {
      full_message = full_message.empty() ? tf_warning : (full_message + " | " + tf_warning);
    }
    row << csvEscape(status) << ",";
    if (success) {
      row << (*success ? "true" : "false");
    }
    row << "," << csvEscape(full_message);

    // --- error_tracking.csv (codex.md §2.3) ------------------------------
    if (call->error_csv.is_open()) {
      call->error_csv << formatDouble(time_sec) << "," << csvEscape(stage) << ","
                      << f(joint_err_norm) << "," << f(joint_max_abs) << ","
                      << f(ex) << "," << f(ey) << "," << f(ez) << ","
                      << f(er) << "," << f(ep) << "," << f(eyaw) << ","
                      << f(pos_norm) << "," << f(ori_err) << "," << f(dist_to_target) << "\n";
      call->error_csv.flush();
    }
    // --- joint_tracking.csv (codex.md §2.1 + §4) -------------------------
    // q_actual/dq_actual from /joint_states; q_set/dq_set interpolated from the
    // planned trajectory (empty when no plan was published for this action).
    if (call->joint_csv.is_open()) {
      call->joint_csv << formatDouble(time_sec) << "," << csvEscape(stage);
      for (int k = 0; k < 6; ++k) {call->joint_csv << "," << f(qset[k]);}     // q_set
      for (int k = 0; k < 6; ++k) {call->joint_csv << "," << f(qpos[k]);}     // q_actual
      for (int k = 0; k < 6; ++k) {call->joint_csv << "," << f(qerr[k]);}     // q_error
      for (int k = 0; k < 6; ++k) {call->joint_csv << "," << f(dqset[k]);}    // dq_set
      for (int k = 0; k < 6; ++k) {call->joint_csv << "," << f(qvel[k]);}     // dq_actual
      for (int k = 0; k < 6; ++k) {
        const double de = (std::isfinite(dqset[k]) && std::isfinite(qvel[k])) ?
          (qvel[k] - dqset[k]) : std::numeric_limits<double>::quiet_NaN();
        call->joint_csv << "," << f(de);                                      // dq_error
      }
      call->joint_csv << "," << f(joint_err_norm) << "," << f(joint_max_abs) << "\n";
      call->joint_csv.flush();
    }

    if (is_repeatability && repeat_index > 1) {
      appendRepeatabilityRow(*call, repeat_index, row.str());
      ++call->row_count;
      return;
    }
    call->csv << row.str() << "\n";
    call->csv.flush();
    ++call->row_count;
  }

  void writeHeader(std::ofstream & csv)
  {
    csv << "time_s,";
    if (canonicalActionName() == "repeatability_test") {
      csv << "repeat_index,";
    }
    // codex.md §2.2: RPY is the primary orientation representation.
    csv << "stage,"
           "tcp_x_set,tcp_y_set,tcp_z_set,tcp_roll_set,tcp_pitch_set,tcp_yaw_set,"
           "tcp_x_actual,tcp_y_actual,tcp_z_actual,tcp_roll_actual,tcp_pitch_actual,tcp_yaw_actual,"
           "tcp_x_error,tcp_y_error,tcp_z_error,tcp_roll_error,tcp_pitch_error,tcp_yaw_error,"
           "tcp_position_error_norm,tcp_orientation_error_norm,distance_to_target,path_length_actual_so_far,"
           "position_error_m,orientation_error_rad,"  // aliases (repeatability stats parser)
           "status,success,message\n";
    csv.flush();
  }

  void appendRepeatabilityRow(const Call & call, int repeat_index, const std::string & row)
  {
    std::ostringstream filename;
    filename << "repeat_" << std::setw(4) << std::setfill('0') << repeat_index << ".csv";
    const auto path = std::filesystem::path(call.call_dir) / filename.str();
    const bool new_file = !std::filesystem::exists(path);
    std::ofstream out(path, std::ios::out | std::ios::app);
    if (!out.is_open()) {
      return;
    }
    if (new_file) {
      writeHeader(out);
    }
    out << row << "\n";
    out.flush();
  }

  void prepareRunDir()
  {
    const std::string stamp = timeStamp("%Y%m%d_%H%M%S") + "_" + std::to_string(::getpid());
    for (int suffix = 0; suffix < 1000; ++suffix) {
      std::ostringstream candidate;
      candidate << log_dir_ << "/run_" << stamp;
      if (suffix > 0) {
        candidate << "_" << std::setw(3) << std::setfill('0') << suffix;
      }
      const std::string dir = candidate.str();
      if (std::filesystem::exists(dir)) {
        continue;
      }
      std::error_code ec;
      if (!std::filesystem::create_directories(dir, ec) && ec) {
        continue;
      }
      run_dir_ = dir;
      index_path_ = run_dir_ + "/index.csv";
      std::ofstream idx(index_path_, std::ios::out | std::ios::trunc);
      if (idx.is_open()) {
        idx << "call_index,action_name,call_dir,start_time,end_time,duration_sec,"
               "status,success,message,row_count\n";
      }
      ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "PerCallTcpLogger(%s) run dir: %s",
        action_name_.c_str(), run_dir_.c_str());
      return;
    }
    RCLCPP_WARN(node_->get_logger(), "Failed to create log directory: %s",
      log_dir_.c_str());
    RCLCPP_WARN(node_->get_logger(), "PerCallTcpLogger(%s): could not create a unique run dir under %s",
      action_name_.c_str(), log_dir_.c_str());
  }

  void appendIndexRow(
    const Call & call,
    const std::string & end_iso,
    double duration_sec,
    const std::string & status,
    bool success,
    const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream idx(index_path_, std::ios::out | std::ios::app);
    if (!idx.is_open()) {
      return;
    }
    idx << std::setw(4) << std::setfill('0') << call.call_index << ","
        << csvEscape(action_name_) << ","
        << csvEscape(call.action_call_id) << ","
        << call.start_iso << ","
        << end_iso << ","
        << formatDouble(duration_sec) << ","
        << csvEscape(status) << ","
        << (success ? "true" : "false") << ","
        << csvEscape(message) << ","
        << call.row_count << "\n";
    idx.flush();
  }

  static double orientationErrorRad(
    const geometry_msgs::msg::Quaternion & a,
    const geometry_msgs::msg::Quaternion & b)
  {
    tf2::Quaternion qa(a.x, a.y, a.z, a.w);
    tf2::Quaternion qb(b.x, b.y, b.z, b.w);
    if (qa.length2() <= 1e-12 || qb.length2() <= 1e-12) {
      return M_PI;
    }
    qa.normalize();
    qb.normalize();
    const double dot = std::abs(qa.dot(qb));
    return 2.0 * std::acos(std::clamp(dot, -1.0, 1.0));
  }

  static std::string formatDouble(double value)
  {
    if (!std::isfinite(value)) {
      return "";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << value;
    return out.str();
  }

  // Short alias used by the RPY tracking rows; NaN -> empty cell.
  static std::string f(double value) {return formatDouble(value);}
  static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

  static std::string csvEscape(const std::string & value)
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

  static std::string timeStamp(const char * fmt)
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&now_time, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, fmt);
    return out.str();
  }

  static std::string isoNow()
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

  static std::string callId(uint32_t index)
  {
    std::ostringstream out;
    out << "call_" << std::setw(4) << std::setfill('0') << index;
    return out.str();
  }

  std::string canonicalActionName() const
  {
    return canonicalLogActionName(action_name_);
  }

  std::string trackingFilename() const
  {
    const auto canonical = canonicalActionName();
    if (canonical == "move_checkerboard") {
      return "tcp_tracking.csv";
    }
    if (canonical == "repeatability_test") {
      return "tcp_tracking.csv";
    }
    if (canonical == "pick_place") {
      return "trajectory_tracking.csv";
    }
    // codex.md §4.1: standardize the baseline motion tracking file name.
    return "tcp_tracking.csv";
  }

  std::string runtimeModeFromRunDir() const
  {
    for (const auto & part : std::filesystem::path(run_dir_)) {
      const auto s = part.string();
      if (s == "mock" || s == "real") {
        return s;
      }
    }
    return "mock";
  }

  void appendEvent(
    const Call & call,
    const std::string & stage,
    const std::string & event_type,
    const std::string & success,
    const std::string & message)
  {
    const auto path = std::filesystem::path(call.call_dir) / "events.csv";
    const bool new_file = !std::filesystem::exists(path);
    std::ofstream out(path, std::ios::out | std::ios::app);
    if (!out.is_open()) {
      return;
    }
    if (new_file) {
      out << "timestamp_iso,t_rel_sec,stage,event_type,success,message\n";
    }
    const double time_sec = (node_->get_clock()->now() - call.start_time).seconds();
    out << csvEscape(isoNow()) << "," << formatDouble(time_sec) << ","
        << csvEscape(stage) << "," << csvEscape(event_type) << ","
        << success << "," << csvEscape(message) << "\n";
  }

  void writeMetadata(const Call & call)
  {
    const auto path = std::filesystem::path(call.call_dir) / "metadata.json";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    const auto canonical = canonicalActionName();
    const std::string hw_mode = hardwareModeFromPath(run_dir_);
    out << "{\n"
        << "  \"runtime_mode\": \"" << hw_mode << "\",\n"
        // codex.md §3.1 fields:
        << "  \"hardware_mode\": \"" << hw_mode << "\",\n"
        << "  \"use_mock\": \"" << use_mock_ << "\",\n"
        << "  \"hardware_plugin\": \"" << hardware_plugin_ << "\",\n"
        << "  \"log_root_dir\": \"" << logRootFromPath(run_dir_) << "\",\n"
        << "  \"evaluation_group\": \"" << evalGroupForAction(canonical) << "\",\n"
        << "  \"data_status\": \"see summary.csv\",\n"
        << "  \"sample_rate_hz\": \"" << sample_rate_hz_ << "\",\n"
        << "  \"log_group\": \"" << logGroupForAction(canonical) << "\",\n"
        << "  \"action_name\": \"" << canonical << "\",\n"
        << "  \"run_id\": \"" << std::filesystem::path(run_dir_).filename().string() << "\",\n"
        << "  \"action_call_id\": \"" << call.action_call_id << "\",\n"
        << "  \"parent_action_call_id\": \"\",\n"
        << "  \"goal_uuid\": \"\",\n"
        << "  \"robot_model\": \"\",\n"
        << "  \"base_frame\": \"" << base_frame_ << "\",\n"
        << "  \"tcp_frame\": \"" << tcp_frame_ << "\",\n"
        << "  \"hardware_backend\": \"\",\n"
        << "  \"vision_source\": \"\",\n"
        << "  \"planner_type\": \"baseline\",\n"
        << "  \"model_path\": \"\",\n"
        << "  \"created_by_node\": \"" << node_->get_name() << "\",\n"
        << "  \"launch_context\": \"\"\n"
        << "}\n";
  }

  void writeSummary(
    const Call & call,
    const std::string & end_iso,
    double duration_sec,
    const std::string & status,
    bool success,
    const std::string & message)
  {
    const auto canonical = canonicalActionName();
    std::ofstream out(std::filesystem::path(call.call_dir) / "summary.csv", std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << standardSummaryHeader();
    if (canonical == "move_checkerboard") {
      out << ",waypoint_count,execution_time_s,mean_joint_error_norm,max_joint_error_norm,rmse_joint_error_norm,"
             "mean_tcp_position_error_m,max_tcp_position_error_m,rmse_tcp_position_error_m,"
             "mean_tcp_orientation_error_rad,max_tcp_orientation_error_rad,rmse_tcp_orientation_error_rad";
    } else if (canonical == "repeatability_test") {
      out << ",axis,repeat_count,offset_m,success_count,failed_count,mean_position_error_m,max_position_error_m,"
             "rmse_position_error_m,mean_orientation_error_rad,max_orientation_error_rad,rmse_orientation_error_rad,"
             "repeatability_position_std_m,repeatability_orientation_std_rad,"
             "mean_joint_error_norm_rad,max_joint_error_norm_rad,rmse_joint_error_norm_rad,"
             "repeatability_joint_std_rad";
    }
    // codex.md §2.5: shared evaluation columns for every baseline motion call.
    out << ",execute_requested,planning_only,final_position_error_m,final_orientation_error_rad,"
           "rmse_tcp_position_m,rmse_tcp_orientation_rad,max_tcp_position_error_m,"
           "actual_path_length_m,straight_line_distance_m,path_efficiency,joint_data_available";
    out << "\n";
    out << runtimeModeFromRunDir() << "," << logGroupForAction(canonical) << "," << canonical << ","
        << csvEscape(std::filesystem::path(run_dir_).filename().string()) << ","
        << call.action_call_id << ",,,"
        << call.start_iso << "," << end_iso << ",,"
        << (success ? "true" : "false") << ","
        << (success ? "" : csvEscape(status)) << ","
        << (success ? "" : csvEscape(status)) << ","
        << csvEscape(message) << ","
        << formatDouble(duration_sec);
    if (canonical == "repeatability_test") {
      const auto & info = call.repeat_summary;
      const auto pos_stats = computePoseErrorStats(call, "position_error_m");
      const auto ori_stats = computePoseErrorStats(call, "orientation_error_rad");
      const auto joint_stats = computeJointErrorStats(call);

      // axis
      out << ",";
      if (info.valid && info.axis >= 0) {
        out << info.axis;
      }
      // repeat_count
      out << ",";
      if (info.valid) {
        out << info.repeat_count;
      }
      // offset_m
      out << "," << (info.valid ? formatDouble(info.offset_m) : "");
      // success_count
      out << ",";
      if (info.valid) {
        out << info.success_count;
      }
      // failed_count: on an unsuccessful call, every repeat not counted as a
      // success (the one that failed plus any never attempted) is a failure, so
      // the requested repeat_count is always fully accounted for.
      out << ",";
      if (info.valid) {
        const int failed_count = success
          ? info.failed_count
          : std::max(info.failed_count, info.repeat_count - info.success_count);
        out << failed_count;
      }
      // position error stats
      out << "," << formatDouble(pos_stats.mean)
          << "," << formatDouble(pos_stats.max)
          << "," << formatDouble(pos_stats.rmse)
          // orientation error stats
          << "," << formatDouble(ori_stats.mean)
          << "," << formatDouble(ori_stats.max)
          << "," << formatDouble(ori_stats.rmse)
          // repeatability spread (std of final per-repeat error)
          << "," << formatDouble(pos_stats.stddev)
          << "," << formatDouble(ori_stats.stddev)
          // joint error stats
          << "," << formatDouble(joint_stats.mean)
          << "," << formatDouble(joint_stats.max)
          << "," << formatDouble(joint_stats.rmse)
          << "," << formatDouble(joint_stats.stddev);
    }
    // codex.md §2.5: shared evaluation values (aligned with the header above).
    {
      const double n = static_cast<double>(std::max<uint64_t>(1, call.err_count));
      const double rmse_pos = call.err_count ? std::sqrt(call.sum_pos_err2 / n) :
        std::numeric_limits<double>::quiet_NaN();
      const double rmse_ori = call.err_count ? std::sqrt(call.sum_ori_err2 / n) :
        std::numeric_limits<double>::quiet_NaN();
      double straight = std::numeric_limits<double>::quiet_NaN();
      if (call.has_first_actual && call.has_last_actual) {
        const double dx = call.last_actual_pos.x - call.first_actual_pos.x;
        const double dy = call.last_actual_pos.y - call.first_actual_pos.y;
        const double dz = call.last_actual_pos.z - call.first_actual_pos.z;
        straight = std::sqrt(dx * dx + dy * dy + dz * dz);
      }
      const double eff = (std::isfinite(straight) && call.path_length_actual > 1e-9) ?
        straight / call.path_length_actual : std::numeric_limits<double>::quiet_NaN();
      out << ",," // execute_requested,planning_only: not tracked by this logger
          << "," << formatDouble(call.final_pos_err)
          << "," << formatDouble(call.final_ori_err)
          << "," << formatDouble(rmse_pos)
          << "," << formatDouble(rmse_ori)
          << "," << formatDouble(call.err_count ? call.max_pos_err : std::numeric_limits<double>::quiet_NaN())
          << "," << formatDouble(call.path_length_actual)
          << "," << formatDouble(straight)
          << "," << formatDouble(eff)
          << "," << (call.joint_available ? "true" : "false");
    }
    out << "\n";
  }

  struct JointErrorStats
  {
    double mean = std::numeric_limits<double>::quiet_NaN();
    double max = std::numeric_limits<double>::quiet_NaN();
    double rmse = std::numeric_limits<double>::quiet_NaN();
    double stddev = std::numeric_limits<double>::quiet_NaN();
  };

  // Splits a CSV line into fields, honoring double-quoted fields (which may
  // contain commas). Good enough for the columns we read (numeric + simple
  // messages) — matches how the rows were written by csvEscape().
  static std::vector<std::string> splitCsvLine(const std::string & line)
  {
    std::vector<std::string> fields;
    std::string field;
    bool in_quotes = false;
    for (size_t i = 0; i < line.size(); ++i) {
      const char c = line[i];
      if (in_quotes) {
        if (c == '"') {
          if (i + 1 < line.size() && line[i + 1] == '"') {
            field += '"';
            ++i;
          } else {
            in_quotes = false;
          }
        } else {
          field += c;
        }
      } else if (c == '"') {
        in_quotes = true;
      } else if (c == ',') {
        fields.push_back(field);
        field.clear();
      } else {
        field += c;
      }
    }
    fields.push_back(field);
    return fields;
  }

  // Computes mean/max/rmse and (population) stddev over the *final* measured
  // value of `column` for each repeat_*.csv file — one representative sample
  // per repeat, mirroring the viewer's "final error" definition. This keeps the
  // repeatability_*_std_* columns meaningful (spread across repeats) rather than
  // conflating them with within-motion tracking noise.
  JointErrorStats computePoseErrorStats(const Call & call, const std::string & column) const
  {
    JointErrorStats stats;
    std::vector<std::string> repeat_files;
    std::error_code ec;
    for (const auto & entry :
      std::filesystem::directory_iterator(call.call_dir, ec))
    {
      if (ec) {
        break;
      }
      const auto name = entry.path().filename().string();
      if (name.rfind("repeat_", 0) == 0 &&
        entry.path().extension() == ".csv" &&
        name.find("_joint.csv") == std::string::npos)
      {
        repeat_files.push_back(entry.path().string());
      }
    }
    std::sort(repeat_files.begin(), repeat_files.end());

    std::vector<double> values;
    for (const auto & path : repeat_files) {
      std::ifstream in(path);
      if (!in.is_open()) {
        continue;
      }
      std::string line;
      if (!std::getline(in, line)) {
        continue;
      }
      const auto header = splitCsvLine(line);
      size_t col = header.size();
      for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == column) {
          col = i;
          break;
        }
      }
      if (col >= header.size()) {
        continue;
      }
      double last_value = std::numeric_limits<double>::quiet_NaN();
      bool found = false;
      while (std::getline(in, line)) {
        const auto fields = splitCsvLine(line);
        if (col >= fields.size() || fields[col].empty()) {
          continue;
        }
        try {
          const double v = std::stod(fields[col]);
          if (std::isfinite(v)) {
            last_value = v;
            found = true;
          }
        } catch (const std::exception &) {
          continue;
        }
      }
      if (found) {
        values.push_back(last_value);
      }
    }
    if (values.empty()) {
      return stats;
    }
    double sum = 0.0;
    double sum_sq = 0.0;
    stats.max = values.front();
    for (const double v : values) {
      sum += v;
      sum_sq += v * v;
      stats.max = std::max(stats.max, v);
    }
    const double n = static_cast<double>(values.size());
    stats.mean = sum / n;
    stats.rmse = std::sqrt(sum_sq / n);
    double var = 0.0;
    for (const double v : values) {
      const double d = v - stats.mean;
      var += d * d;
    }
    stats.stddev = std::sqrt(var / n);
    return stats;
  }

  JointErrorStats computeJointErrorStats(const Call & call) const
  {
    JointErrorStats stats;
    const auto path = std::filesystem::path(call.call_dir) / "joint_tracking.csv";
    std::ifstream in(path);
    if (!in.is_open()) {
      return stats;
    }
    std::string line;
    std::getline(in, line);  // header
    std::vector<double> values;
    while (std::getline(in, line)) {
      const auto pos = line.rfind(',');
      if (pos == std::string::npos || pos + 1 >= line.size()) {
        continue;
      }
      try {
        const double v = std::stod(line.substr(pos + 1));
        if (std::isfinite(v)) {
          values.push_back(v);
        }
      } catch (const std::exception &) {
        continue;
      }
    }
    if (values.empty()) {
      return stats;
    }
    double sum = 0.0;
    double sum_sq = 0.0;
    stats.max = values.front();
    for (const double v : values) {
      sum += v;
      sum_sq += v * v;
      stats.max = std::max(stats.max, v);
    }
    const double n = static_cast<double>(values.size());
    stats.mean = sum / n;
    stats.rmse = std::sqrt(sum_sq / n);
    double var = 0.0;
    for (const double v : values) {
      const double d = v - stats.mean;
      var += d * d;
    }
    stats.stddev = std::sqrt(var / n);
    return stats;
  }

  void writePlaceholders(const Call & call)
  {
    const auto canonical = canonicalActionName();
    if (canonical == "move_checkerboard") {
      std::ofstream joints(std::filesystem::path(call.call_dir) / "joint_tracking.csv", std::ios::out | std::ios::trunc);
      if (joints.is_open()) {
        joints << "time_s,stage,joint_1_set_rad,joint_2_set_rad,joint_3_set_rad,joint_4_set_rad,joint_5_set_rad,joint_6_set_rad,"
                  "joint_1_actual_rad,joint_2_actual_rad,joint_3_actual_rad,joint_4_actual_rad,joint_5_actual_rad,joint_6_actual_rad,"
                  "joint_1_error_rad,joint_2_error_rad,joint_3_error_rad,joint_4_error_rad,joint_5_error_rad,joint_6_error_rad,joint_error_norm_rad\n";
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string log_dir_;
  double sample_rate_hz_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string file_prefix_;
  std::string action_name_;

  std::mutex mutex_;
  std::string run_dir_;
  std::string index_path_;
  uint32_t next_call_index_ = 0;
  bool ready_ = false;

  // codex.md §2.1: latest /joint_states snapshot (actual positions/velocities).
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::mutex joint_mutex_;
  sensor_msgs::msg::JointState latest_joint_;
  bool have_joint_ = false;

  // codex.md §4: latest planned joint trajectory (the q_set/dq_set source).
  rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr planned_traj_sub_;
  std::mutex planned_mutex_;
  moveit_msgs::msg::RobotTrajectory latest_planned_traj_;
  rclcpp::Time planned_recv_time_;
  bool have_planned_ = false;

  // Interpolated joint setpoint (position + velocity) at `elapsed_sec` into the
  // latest planned trajectory, reported in joint_order_ order. `ok` is false if
  // no planned trajectory is available. Returns NaN for joints not in the plan.
  void plannedSetpoint(double elapsed_sec, std::vector<double> & qset,
    std::vector<double> & dqset, bool & ok)
  {
    qset.assign(6, std::numeric_limits<double>::quiet_NaN());
    dqset.assign(6, std::numeric_limits<double>::quiet_NaN());
    std::lock_guard<std::mutex> lock(planned_mutex_);
    const auto & jt = latest_planned_traj_.joint_trajectory;
    ok = have_planned_ && !jt.points.empty() && jt.joint_names.size() >= 1;
    if (!ok) {
      return;
    }
    // Locate the two points bracketing elapsed_sec (clamped to trajectory ends).
    auto tsec = [](const builtin_interfaces::msg::Duration & d) {
      return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
    };
    const size_t n = jt.points.size();
    size_t hi = n - 1;
    for (size_t i = 0; i < n; ++i) {
      if (tsec(jt.points[i].time_from_start) >= elapsed_sec) {hi = i; break;}
    }
    const size_t lo = hi == 0 ? 0 : hi - 1;
    const double t_lo = tsec(jt.points[lo].time_from_start);
    const double t_hi = tsec(jt.points[hi].time_from_start);
    const double a = (hi == lo || t_hi <= t_lo) ? 0.0 :
      std::clamp((elapsed_sec - t_lo) / (t_hi - t_lo), 0.0, 1.0);
    // Map planned joint names -> joint_order_ slots (fall back to plan order).
    std::unordered_map<std::string, size_t> order_idx;
    for (size_t k = 0; k < joint_order_.size(); ++k) {order_idx[joint_order_[k]] = k;}
    for (size_t j = 0; j < jt.joint_names.size(); ++j) {
      size_t slot = j < 6 ? j : 6;
      const auto it = order_idx.find(jt.joint_names[j]);
      if (it != order_idx.end()) {slot = it->second;}
      if (slot >= 6) {continue;}
      const auto & plo = jt.points[lo];
      const auto & phi = jt.points[hi];
      if (j < plo.positions.size() && j < phi.positions.size()) {
        qset[slot] = plo.positions[j] + a * (phi.positions[j] - plo.positions[j]);
      }
      if (j < plo.velocities.size() && j < phi.velocities.size()) {
        dqset[slot] = plo.velocities[j] + a * (phi.velocities[j] - plo.velocities[j]);
      }
    }
  }

  // codex.md §6: hardware provenance for metadata.json.
  std::string use_mock_ = "unknown";
  std::string hardware_plugin_ = "unknown";
  bool plots_enabled_ = true;  // codex.md §5

  // Ordered joint names (arm joints) for joint_tracking.csv columns q1..q6.
  // Filled lazily from the first /joint_states seen so the six arm joints are
  // reported in a stable order regardless of message ordering.
  std::vector<std::string> joint_order_;

  // Returns actual position/velocity for the arm joints in joint_order_ order.
  // Any missing joint yields NaN. `ok` is false if no joint state seen yet.
  void currentJoints(std::vector<double> & pos, std::vector<double> & vel, bool & ok)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    ok = have_joint_;
    pos.assign(6, std::numeric_limits<double>::quiet_NaN());
    vel.assign(6, std::numeric_limits<double>::quiet_NaN());
    if (!have_joint_) {
      return;
    }
    if (joint_order_.empty()) {
      // Prefer names containing "joint" and sort for stability; cap at 6.
      std::vector<std::string> names = latest_joint_.name;
      std::sort(names.begin(), names.end());
      for (const auto & n : names) {
        if (joint_order_.size() >= 6) {
          break;
        }
        if (n.find("joint") != std::string::npos || n.find("Joint") != std::string::npos) {
          joint_order_.push_back(n);
        }
      }
      if (joint_order_.empty()) {  // fallback: first up-to-6 names as-is
        for (size_t i = 0; i < latest_joint_.name.size() && joint_order_.size() < 6; ++i) {
          joint_order_.push_back(latest_joint_.name[i]);
        }
      }
    }
    std::unordered_map<std::string, size_t> idx;
    for (size_t i = 0; i < latest_joint_.name.size(); ++i) {
      idx[latest_joint_.name[i]] = i;
    }
    for (size_t k = 0; k < joint_order_.size() && k < 6; ++k) {
      const auto it = idx.find(joint_order_[k]);
      if (it == idx.end()) {
        continue;
      }
      if (it->second < latest_joint_.position.size()) {
        pos[k] = latest_joint_.position[it->second];
      }
      if (it->second < latest_joint_.velocity.size()) {
        vel[k] = latest_joint_.velocity[it->second];
      }
    }
  }
};

}  // namespace robot_task_manager
