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

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"

namespace robot_task_manager
{

class PerCallTcpLogger
{
public:
  struct Call
  {
    uint32_t call_index = 0;
    std::string csv_filename;
    std::string csv_path;
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
  }

  ~PerCallTcpLogger() = default;

  std::shared_ptr<Call> startCall(const std::string & metadata_json = "")
  {
    if (!ready_) {
      return nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto call = std::make_shared<Call>();
    call->call_index = ++next_call_index_;
    call->start_time = node_->get_clock()->now();
    call->start_iso = isoNow();
    call->metadata_json = metadata_json;

    std::ostringstream name;
    name << file_prefix_ << "_" << std::setw(4) << std::setfill('0') << call->call_index
         << "_" << timeStamp("%Y%m%d_%H%M%S") << ".csv";
    call->csv_filename = name.str();
    call->csv_path = run_dir_ + "/" + call->csv_filename;

    call->csv.open(call->csv_path, std::ios::out | std::ios::trunc);
    if (!call->csv.is_open()) {
      RCLCPP_WARN(node_->get_logger(), "Failed to write log CSV: %s",
        call->csv_path.c_str());
      RCLCPP_WARN(node_->get_logger(), "PerCallTcpLogger(%s): cannot open %s",
        action_name_.c_str(), call->csv_path.c_str());
      return nullptr;
    }
    writeHeader(call->csv);
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

    const std::string end_iso = isoNow();
    const double duration_sec = (node_->get_clock()->now() - call->start_time).seconds();
    appendIndexRow(*call, end_iso, duration_sec, status, success, message);
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
    const std::string & row_type,
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

    std::ostringstream row;
    row << formatDouble(time_sec) << ","
        << csvEscape(stage) << ","
        << csvEscape(row_type) << ",";

    if (set_pose) {
      row << formatDouble(set_pose->position.x) << "," << formatDouble(set_pose->position.y) << ","
          << formatDouble(set_pose->position.z) << "," << formatDouble(set_pose->orientation.x) << ","
          << formatDouble(set_pose->orientation.y) << "," << formatDouble(set_pose->orientation.z) << ","
          << formatDouble(set_pose->orientation.w) << ",";
    } else {
      row << ",,,,,,,";
    }

    if (have_actual) {
      row << formatDouble(actual_pose.position.x) << "," << formatDouble(actual_pose.position.y) << ","
          << formatDouble(actual_pose.position.z) << "," << formatDouble(actual_pose.orientation.x) << ","
          << formatDouble(actual_pose.orientation.y) << "," << formatDouble(actual_pose.orientation.z) << ","
          << formatDouble(actual_pose.orientation.w) << ",";
    } else {
      row << ",,,,,,,";
    }

    if (set_pose && have_actual) {
      const double ex = actual_pose.position.x - set_pose->position.x;
      const double ey = actual_pose.position.y - set_pose->position.y;
      const double ez = actual_pose.position.z - set_pose->position.z;
      const double pos_norm = std::sqrt(ex * ex + ey * ey + ez * ez);
      const double ori_err = orientationErrorRad(set_pose->orientation, actual_pose.orientation);
      row << formatDouble(ex) << "," << formatDouble(ey) << "," << formatDouble(ez) << ","
          << formatDouble(pos_norm) << "," << formatDouble(ori_err) << ",";
    } else {
      row << ",,,,,";
    }

    row << csvEscape(status) << ",";
    if (success) {
      row << (*success ? "true" : "false");
    }
    row << ",";
    std::string full_message = message;
    if (!tf_warning.empty()) {
      full_message = full_message.empty() ? tf_warning : (full_message + " | " + tf_warning);
    }
    row << csvEscape(full_message) << ","
        << call->call_index << ","
        << csvEscape(call->metadata_json);

    std::lock_guard<std::mutex> lock(mutex_);
    call->csv << row.str() << "\n";
    call->csv.flush();
    ++call->row_count;
  }

  void writeHeader(std::ofstream & csv)
  {
    csv << "time_sec,stage,row_type,"
           "set_x,set_y,set_z,set_qx,set_qy,set_qz,set_qw,"
           "actual_x,actual_y,actual_z,actual_qx,actual_qy,actual_qz,actual_qw,"
           "error_x,error_y,error_z,error_pos_norm,error_ori_rad,"
           "status,success,message,call_index,metadata_json\n";
    csv.flush();
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
      if (!idx.is_open()) {
        return;
      }
      idx << "call_index,action_name,csv_file,start_time,end_time,duration_sec,"
             "status,success,message,row_count\n";
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
        << csvEscape(call.csv_filename) << ","
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
};

}  // namespace robot_task_manager
