#pragma once

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_task_manager/log_paths.hpp"

namespace robot_task_manager
{

inline std::string standardCsvEscape(const std::string & value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    escaped += c == '"' ? "\"\"" : std::string(1, c);
  }
  escaped += "\"";
  return escaped;
}

inline std::string standardFormatDouble(double value)
{
  if (!std::isfinite(value)) {
    return "";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << value;
  return out.str();
}

inline std::string standardIsoNow()
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

inline std::string standardTimeStamp(const char * fmt)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&now_time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, fmt);
  return out.str();
}

inline std::string standardSummaryHeader()
{
  return
    "runtime_mode,log_group,action_name,run_id,action_call_id,parent_action_call_id,"
    "goal_uuid,start_timestamp_iso,end_timestamp_iso,execute_requested,success,"
    "failed_stage,failure_reason,message,total_time_s";
}

struct StandardCallInfo
{
  std::filesystem::path call_dir;
  std::string run_id;
  std::string action_call_id;
  std::string parent_action_call_id;
  std::string goal_uuid;
  std::string start_iso;
  rclcpp::Time start_time;
};

struct StandardSummary
{
  std::string goal_uuid;
  std::string start_iso;
  std::string end_iso;
  std::optional<bool> execute_requested;
  std::optional<bool> success;
  std::string failed_stage;
  std::string failure_reason;
  std::string message;
  double total_time_s = std::numeric_limits<double>::quiet_NaN();
  std::string extra_header;
  std::string extra_values;
};

class StandardActionLogger
{
public:
  StandardActionLogger(
    const rclcpp::Node::SharedPtr & node,
    std::string log_root_dir,
    std::string runtime_mode,
    std::string action_name,
    std::string robot_model = "",
    std::string base_frame = "",
    std::string tcp_frame = "",
    std::string hardware_backend = "",
    std::string vision_source = "",
    std::string planner_type = "",
    std::string model_path = "")
  : node_(node),
    log_root_dir_(std::move(log_root_dir)),
    runtime_mode_(normalizedRuntimeMode(runtime_mode)),
    action_name_(canonicalLogActionName(action_name)),
    log_group_(logGroupForAction(action_name_)),
    robot_model_(std::move(robot_model)),
    base_frame_(std::move(base_frame)),
    tcp_frame_(std::move(tcp_frame)),
    hardware_backend_(std::move(hardware_backend)),
    vision_source_(std::move(vision_source)),
    planner_type_(std::move(planner_type)),
    model_path_(std::move(model_path))
  {
    prepareRunDir();
  }

  std::shared_ptr<StandardCallInfo> startCall(
    const std::string & goal_uuid = "",
    const std::string & parent_action_call_id = "")
  {
    if (!ready_) {
      return nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto call = std::make_shared<StandardCallInfo>();
    call->run_id = run_id_;
    call->action_call_id = "call_" + zeroPad(++next_call_index_, 4);
    call->parent_action_call_id = parent_action_call_id;
    call->goal_uuid = goal_uuid;
    call->start_iso = standardIsoNow();
    call->start_time = node_ ? node_->get_clock()->now() : rclcpp::Clock().now();
    call->call_dir = run_dir_ / call->action_call_id;
    std::error_code ec;
    std::filesystem::create_directories(call->call_dir, ec);
    if (ec) {
      warn("Failed to create call log directory: " + call->call_dir.string() + " | " + ec.message());
      return nullptr;
    }
    writeMetadata(*call);
    appendEvent(*call, "start", "action_start", std::nullopt, "standard log call created", 0.0);
    return call;
  }

  void appendEvent(
    const StandardCallInfo & call,
    const std::string & stage,
    const std::string & event_type,
    std::optional<bool> success,
    const std::string & message,
    double t_rel_sec = std::numeric_limits<double>::quiet_NaN())
  {
    const auto path = call.call_dir / "events.csv";
    const bool new_file = !std::filesystem::exists(path);
    std::ofstream out(path, std::ios::out | std::ios::app);
    if (!out.is_open()) {
      return;
    }
    if (new_file) {
      out << "timestamp_iso,t_rel_sec,stage,event_type,success,message\n";
    }
    out << standardCsvEscape(standardIsoNow()) << ","
        << standardFormatDouble(t_rel_sec) << ","
        << standardCsvEscape(stage) << ","
        << standardCsvEscape(event_type) << ",";
    if (success.has_value()) {
      out << (*success ? "true" : "false");
    }
    out << "," << standardCsvEscape(message) << "\n";
  }

  void writeSummary(const StandardCallInfo & call, const StandardSummary & summary)
  {
    const auto path = call.call_dir / "summary.csv";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      warn("Failed to write summary.csv: " + path.string());
      return;
    }
    out << standardSummaryHeader();
    if (!summary.extra_header.empty()) {
      out << "," << summary.extra_header;
    }
    out << "\n";

    out << standardCsvEscape(runtime_mode_) << ","
        << standardCsvEscape(log_group_) << ","
        << standardCsvEscape(action_name_) << ","
        << standardCsvEscape(call.run_id) << ","
        << standardCsvEscape(call.action_call_id) << ","
        << standardCsvEscape(call.parent_action_call_id) << ","
        << standardCsvEscape(summary.goal_uuid.empty() ? call.goal_uuid : summary.goal_uuid) << ","
        << standardCsvEscape(summary.start_iso.empty() ? call.start_iso : summary.start_iso) << ","
        << standardCsvEscape(summary.end_iso.empty() ? standardIsoNow() : summary.end_iso) << ",";
    if (summary.execute_requested.has_value()) {
      out << (*summary.execute_requested ? "true" : "false");
    }
    out << ",";
    if (summary.success.has_value()) {
      out << (*summary.success ? "true" : "false");
    }
    out << ","
        << standardCsvEscape(summary.failed_stage) << ","
        << standardCsvEscape(summary.failure_reason) << ","
        << standardCsvEscape(summary.message) << ","
        << standardFormatDouble(summary.total_time_s);
    if (!summary.extra_values.empty()) {
      out << "," << summary.extra_values;
    }
    out << "\n";
  }

  void writePlaceholderCsv(const StandardCallInfo & call, const std::string & filename, const std::string & header)
  {
    const auto path = call.call_dir / filename;
    if (std::filesystem::exists(path)) {
      return;
    }
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (out.is_open()) {
      out << header << "\n";
    }
  }

  const std::filesystem::path & runDir() const {return run_dir_;}
  const std::string & runId() const {return run_id_;}

private:
  void prepareRunDir()
  {
    const auto action_dir = std::filesystem::path(
      standardActionLogDir(log_root_dir_, runtime_mode_, action_name_));
    const auto stamp = standardTimeStamp("%Y%m%d_%H%M%S") + "_" + std::to_string(::getpid());
    for (int suffix = 0; suffix < 1000; ++suffix) {
      std::ostringstream candidate;
      candidate << "run_" << stamp;
      if (suffix > 0) {
        candidate << "_" << std::setw(3) << std::setfill('0') << suffix;
      }
      auto dir = action_dir / candidate.str();
      if (std::filesystem::exists(dir)) {
        continue;
      }
      std::error_code ec;
      std::filesystem::create_directories(dir, ec);
      if (ec) {
        continue;
      }
      run_dir_ = dir;
      run_id_ = candidate.str();
      ready_ = true;
      return;
    }
    warn("Failed to create standard log run directory under: " + action_dir.string());
  }

  void writeMetadata(const StandardCallInfo & call)
  {
    const auto path = call.call_dir / "metadata.json";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << "{\n"
        << "  \"runtime_mode\": \"" << jsonEscape(runtime_mode_) << "\",\n"
        // codex.md §3.1 fields (GoHome/MoveGripper live under _debug):
        << "  \"hardware_mode\": \"" << jsonEscape(runtime_mode_) << "\",\n"
        << "  \"use_mock\": \"unknown\",\n"
        << "  \"hardware_plugin\": \"unknown\",\n"
        << "  \"log_root_dir\": \"" << jsonEscape(logRootFromPath(run_dir_.string())) << "\",\n"
        << "  \"evaluation_group\": \"" << jsonEscape(evalGroupForAction(action_name_)) << "\",\n"
        << "  \"data_status\": \"see summary.csv\",\n"
        << "  \"log_group\": \"" << jsonEscape(log_group_) << "\",\n"
        << "  \"action_name\": \"" << jsonEscape(action_name_) << "\",\n"
        << "  \"run_id\": \"" << jsonEscape(call.run_id) << "\",\n"
        << "  \"action_call_id\": \"" << jsonEscape(call.action_call_id) << "\",\n"
        << "  \"parent_action_call_id\": \"" << jsonEscape(call.parent_action_call_id) << "\",\n"
        << "  \"goal_uuid\": \"" << jsonEscape(call.goal_uuid) << "\",\n"
        << "  \"robot_model\": \"" << jsonEscape(robot_model_) << "\",\n"
        << "  \"base_frame\": \"" << jsonEscape(base_frame_) << "\",\n"
        << "  \"tcp_frame\": \"" << jsonEscape(tcp_frame_) << "\",\n"
        << "  \"hardware_backend\": \"" << jsonEscape(hardware_backend_) << "\",\n"
        << "  \"vision_source\": \"" << jsonEscape(vision_source_) << "\",\n"
        << "  \"planner_type\": \"" << jsonEscape(planner_type_) << "\",\n"
        << "  \"model_path\": \"" << jsonEscape(model_path_) << "\",\n"
        << "  \"created_by_node\": \"" << jsonEscape(node_ ? node_->get_name() : "") << "\",\n"
        << "  \"launch_context\": \"\"\n"
        << "}\n";
  }

  static std::string zeroPad(uint64_t value, int width)
  {
    std::ostringstream out;
    out << std::setw(width) << std::setfill('0') << value;
    return out.str();
  }

  static std::string jsonEscape(const std::string & value)
  {
    std::string out;
    for (const char c : value) {
      if (c == '\\' || c == '"') {
        out += '\\';
      }
      out += c;
    }
    return out;
  }

  void warn(const std::string & message) const
  {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string log_root_dir_;
  std::string runtime_mode_;
  std::string action_name_;
  std::string log_group_;
  std::string robot_model_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string hardware_backend_;
  std::string vision_source_;
  std::string planner_type_;
  std::string model_path_;
  std::filesystem::path run_dir_;
  std::string run_id_;
  std::mutex mutex_;
  uint64_t next_call_index_ = 0;
  bool ready_ = false;
};

}  // namespace robot_task_manager
