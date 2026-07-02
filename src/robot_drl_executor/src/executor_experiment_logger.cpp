// Local copy of robot_task_executor's ExecutorExperimentLogger (codex2.md
// section 14: robot_drl_executor must not reach into ../robot_task_executor
// at build time). Kept in the `robot_task_executor` namespace deliberately
// — this is a separate executable from task_executor_node, so there is no
// ODR/link conflict, and it avoids touching robot_drl_executor_node.cpp's
// existing `robot_task_executor::ExecutorExperimentLogger` usage.
#include "robot_drl_executor/executor_experiment_logger.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace robot_task_executor
{
namespace
{

constexpr double kMissingJointStateWarnSec = 1.0;

std::string time_stamp_string(const char* suffix)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  localtime_r(&now_time, &tm);
  char buffer[64];
  std::strftime(buffer, sizeof(buffer), suffix, &tm);
  return std::string(buffer);
}

bool has_valid_stamp(const rclcpp::Time& stamp)
{
  return stamp.nanoseconds() > 0;
}

}  // namespace

ExecutorExperimentLogger::ExecutorExperimentLogger(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const std::string& log_dir,
    const double sample_rate_hz,
    const std::string& base_frame,
    const std::string& tcp_frame)
: node_(node),
  tf_buffer_(tf_buffer),
  logger_(node->get_logger()),
  base_frame_(base_frame),
  tcp_frame_(tcp_frame),
  sample_rate_hz_(sample_rate_hz > 0.0 ? sample_rate_hz : 50.0)
{
  try
  {
    enabled_ = prepare_output(log_dir);
    if (!enabled_)
    {
      return;
    }

    write_header();
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          joint_state_callback(msg);
        });

    RCLCPP_INFO(logger_, "Executor experiment logger enabled: %s", csv_path_.c_str());
  }
  catch (const std::exception& ex)
  {
    enabled_ = false;
    RCLCPP_WARN(logger_, "Executor logger disabled after init error: %s", ex.what());
  }
}

ExecutorExperimentLogger::~ExecutorExperimentLogger()
{
  try
  {
    stop_sampling(0);
    if (sampling_thread_.joinable())
    {
      sampling_thread_.join();
    }
    if (csv_.is_open())
    {
      csv_.flush();
      csv_.close();
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(logger_, "Executor logger shutdown warning: %s", ex.what());
  }
}

uint64_t ExecutorExperimentLogger::start_call(
    const std::string& action_name,
    const std::string& execute_mode,
    const std::string& note)
{
  if (!enabled_)
  {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const uint64_t action_call_id = next_action_call_id_++;
  Session session;
  session.action_call_id = action_call_id;
  session.action_name = action_name;
  session.execute_mode = execute_mode;
  session.start_iso = iso_now();
  session.start_time = node_->get_clock()->now();
  sessions_[action_call_id] = session;

  Row row = base_row(sessions_[action_call_id], "execute_start");
  row["timestamp_start_iso"] = session.start_iso;
  row["execute_mode"] = execute_mode;
  row["status"] = "started";
  row["note"] = note;
  write_row(row);
  return action_call_id;
}

void ExecutorExperimentLogger::log_ref_waypoint(
    const uint64_t action_call_id,
    const size_t ref_index,
    const geometry_msgs::msg::Pose& pose,
    const rclcpp::Time& ref_time,
    const std::string& note)
{
  if (!enabled_ || action_call_id == 0)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sessions_.find(action_call_id);
  if (it == sessions_.end())
  {
    return;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  quaternion_to_rpy(pose.orientation, roll, pitch, yaw);

  auto& session = it->second;
  session.ref_waypoints_count = std::max(session.ref_waypoints_count, ref_index + 1);

  Row row = base_row(session, "ref_waypoint");
  row["ref_index"] = std::to_string(ref_index);
  row["ref_timestamp"] = has_valid_stamp(ref_time) ? format_double(ref_time.seconds()) : "";
  row["ref_x"] = format_double(pose.position.x);
  row["ref_y"] = format_double(pose.position.y);
  row["ref_z"] = format_double(pose.position.z);
  row["ref_qx"] = format_double(pose.orientation.x);
  row["ref_qy"] = format_double(pose.orientation.y);
  row["ref_qz"] = format_double(pose.orientation.z);
  row["ref_qw"] = format_double(pose.orientation.w);
  row["ref_roll"] = format_double(roll);
  row["ref_pitch"] = format_double(pitch);
  row["ref_yaw"] = format_double(yaw);
  row["note"] = note;
  write_row(row);
}

void ExecutorExperimentLogger::log_joint_command(
    const uint64_t action_call_id,
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& note)
{
  if (!enabled_ || action_call_id == 0)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sessions_.find(action_call_id);
  if (it == sessions_.end())
  {
    return;
  }

  auto& session = it->second;
  const auto& joint_trajectory = trajectory.joint_trajectory;
  const auto command_stamp = node_->get_clock()->now();

  for (size_t point_index = 0; point_index < joint_trajectory.points.size(); ++point_index)
  {
    const auto& point = joint_trajectory.points[point_index];
    JointCommandPoint command;
    command.time_from_start_sec = duration_sec(point.time_from_start);

    Row row = base_row(session, "joint_command");
    row["joint_cmd_timestamp"] = format_double(command_stamp.seconds());
    row["joint_cmd_index"] = std::to_string(session.joint_command_points_count + point_index);
    row["joint_cmd_time_from_start_sec"] = format_double(command.time_from_start_sec);
    row["note"] = note;

    for (size_t expected_index = 0; expected_index < joint_names_.size(); ++expected_index)
    {
      const auto joint_it = std::find(
          joint_trajectory.joint_names.begin(),
          joint_trajectory.joint_names.end(),
          joint_names_[expected_index]);
      if (joint_it == joint_trajectory.joint_names.end())
      {
        continue;
      }
      const size_t source_index = static_cast<size_t>(
          std::distance(joint_trajectory.joint_names.begin(), joint_it));

      const std::string joint_number = std::to_string(expected_index + 1);
      if (source_index < point.positions.size())
      {
        command.positions[expected_index] = point.positions[source_index];
        command.has_position[expected_index] = true;
        row["joint_" + joint_number + "_cmd"] = format_double(point.positions[source_index]);
      }
      if (source_index < point.velocities.size())
      {
        command.velocities[expected_index] = point.velocities[source_index];
        command.has_velocity[expected_index] = true;
        row["joint_" + joint_number + "_cmd_velocity"] =
            format_double(point.velocities[source_index]);
      }
      if (source_index < point.accelerations.size())
      {
        command.accelerations[expected_index] = point.accelerations[source_index];
        command.has_acceleration[expected_index] = true;
        row["joint_" + joint_number + "_cmd_acceleration"] =
            format_double(point.accelerations[source_index]);
      }
    }

    session.command_points.push_back(command);
    write_row(row);
  }

  session.joint_command_points_count += joint_trajectory.points.size();
}

void ExecutorExperimentLogger::start_sampling(
    const uint64_t action_call_id,
    const std::string& execute_mode,
    const std::vector<geometry_msgs::msg::Pose>& refs)
{
  if (!enabled_ || action_call_id == 0)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_sampling_id_ != 0)
    {
      stop_sampling_locked(active_sampling_id_);
    }

    auto it = sessions_.find(action_call_id);
    if (it == sessions_.end())
    {
      return;
    }
    it->second.execute_mode = execute_mode;
    it->second.refs = refs;
    it->second.ref_waypoints_count = std::max(it->second.ref_waypoints_count, refs.size());
    active_sampling_id_ = action_call_id;
    sampling_stop_requested_ = false;
  }

  if (sampling_thread_.joinable())
  {
    sampling_thread_.join();
  }
  sampling_thread_ = std::thread([this, action_call_id]()
  {
    sample_loop(action_call_id);
  });
}

void ExecutorExperimentLogger::stop_sampling(const uint64_t action_call_id)
{
  if (!enabled_)
  {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_sampling_id_ == 0)
    {
      sampling_stop_requested_ = true;
    }
    else if (action_call_id != 0 && active_sampling_id_ != action_call_id)
    {
      return;
    }
    else
    {
      stop_sampling_locked(active_sampling_id_);
    }
  }

  if (sampling_thread_.joinable())
  {
    sampling_thread_.join();
  }
}

void ExecutorExperimentLogger::log_summary(
    const uint64_t action_call_id,
    const std::string& status,
    const bool success,
    const std::string& message,
    const double fraction,
    const std::string& note)
{
  if (!enabled_ || action_call_id == 0)
  {
    return;
  }

  stop_sampling(action_call_id);

  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sessions_.find(action_call_id);
  if (it == sessions_.end())
  {
    return;
  }

  const auto now = node_->get_clock()->now();
  const auto& session = it->second;
  Row row = base_row(session, "execute_summary");
  row["timestamp_start_iso"] = session.start_iso;
  row["timestamp_end_iso"] = iso_now();
  row["duration_sec"] = format_double(elapsed_sec(session.start_time, now));
  row["status"] = status;
  row["success"] = format_bool(success);
  row["message"] = message;
  row["fraction"] = format_double(fraction);
  row["actual_samples_count"] = std::to_string(session.actual_samples_count);
  row["joint_command_points_count"] = std::to_string(session.joint_command_points_count);
  row["ref_waypoints_count"] = std::to_string(session.ref_waypoints_count);
  row["mean_joint_error_norm"] = format_double(mean(session.joint_error_norms));
  row["max_joint_error_norm"] = format_double(max_value(session.joint_error_norms));
  row["rmse_joint_error_norm"] = format_double(rmse(session.joint_error_norms));
  row["final_joint_error_norm"] = session.has_final_joint_error_norm
      ? format_double(session.final_joint_error_norm) : "";
  row["mean_tcp_position_error_norm"] = format_double(mean(session.tcp_position_error_norms));
  row["max_tcp_position_error_norm"] = format_double(max_value(session.tcp_position_error_norms));
  row["rmse_tcp_position_error_norm"] = format_double(rmse(session.tcp_position_error_norms));
  row["final_tcp_position_error_norm"] = session.has_final_tcp_position_error_norm
      ? format_double(session.final_tcp_position_error_norm) : "";
  row["note"] = note;
  write_row(row);
  csv_.flush();
}

uint64_t ExecutorExperimentLogger::log_lifecycle_event(
    const std::string& action_name,
    const std::string& row_type,
    const std::string& stage_name,
    const std::string& status,
    const std::string& message,
    const std::string& metadata_json,
    uint64_t existing_call_id)
{
  if (!enabled_)
  {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  uint64_t action_call_id = existing_call_id;
  if (action_call_id == 0)
  {
    action_call_id = next_action_call_id_++;
    Session session;
    session.action_call_id = action_call_id;
    session.action_name = action_name;
    session.start_iso = iso_now();
    session.start_time = node_->get_clock()->now();
    sessions_[action_call_id] = session;
  }

  auto it = sessions_.find(action_call_id);
  if (it == sessions_.end())
  {
    return 0;
  }

  Row row = base_row(it->second, row_type);
  row["stage_name"] = stage_name;
  row["status"] = status;
  row["message"] = message;
  row["metadata_json"] = metadata_json;
  write_row(row);
  csv_.flush();
  return action_call_id;
}

bool ExecutorExperimentLogger::prepare_output(const std::string& log_dir)
{
  const std::string abs_log_dir =
      std::filesystem::absolute(std::filesystem::path(log_dir)).lexically_normal().string();
  // Multiple MoveItExecutor-owning processes (gohome_server, move_to_pose_server,
  // move_pose_cartesian_server, checker_board_server, task_executor_node, ...)
  // can each construct their own logger within the same wall-clock second at
  // launch. A plain timestamp-based run_id would then collide across
  // *independent processes*, and a check-then-mkdir/truncate-open is not
  // atomic across processes, so the second opener silently truncates the
  // first's still-open file. Folding in the PID makes run_id unique per
  // process regardless of timestamp resolution, removing the race outright.
  const std::string stamp =
      time_stamp_string("%Y%m%d_%H%M%S") + "_" + std::to_string(::getpid());
  for (int suffix = 0; suffix < 1000; ++suffix)
  {
    std::ostringstream suffix_stream;
    if (suffix > 0)
    {
      suffix_stream << "_" << std::setw(3) << std::setfill('0') << suffix;
    }
    run_id_ = "run_" + stamp + suffix_stream.str();
    const std::string run_dir = abs_log_dir + "/" + run_id_;
    const std::string csv_name = "executor_log_" + stamp + suffix_stream.str() + ".csv";
    csv_path_ = run_dir + "/" + csv_name;

    if (std::filesystem::exists(run_dir) || std::filesystem::exists(csv_path_))
    {
      continue;
    }

    if (!mkdirs(run_dir))
    {
      RCLCPP_WARN(logger_, "Failed to create log directory: %s", run_dir.c_str());
      RCLCPP_WARN(logger_, "Executor logger cannot create directory: %s", run_dir.c_str());
      return false;
    }

    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open())
    {
      RCLCPP_WARN(logger_, "Failed to write log CSV: %s", csv_path_.c_str());
      RCLCPP_WARN(logger_, "Executor logger cannot open CSV: %s", csv_path_.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "Executor CSV saved: %s", csv_path_.c_str());
    return true;
  }

  RCLCPP_WARN(logger_, "Failed to create log directory: %s", abs_log_dir.c_str());
  RCLCPP_WARN(logger_, "Executor logger could not find a unique run directory in %s", abs_log_dir.c_str());
  return false;
}

void ExecutorExperimentLogger::write_header()
{
  const auto header = csv_header();
  for (size_t i = 0; i < header.size(); ++i)
  {
    if (i > 0)
    {
      csv_ << ",";
    }
    csv_ << header[i];
  }
  csv_ << "\n";
  csv_.flush();
}

void ExecutorExperimentLogger::write_row(const Row& row)
{
  if (!csv_.is_open())
  {
    return;
  }

  const auto header = csv_header();
  for (size_t i = 0; i < header.size(); ++i)
  {
    if (i > 0)
    {
      csv_ << ",";
    }
    const auto value_it = row.find(header[i]);
    csv_ << csv_escape(value_it == row.end() ? "" : value_it->second);
  }
  csv_ << "\n";
}

ExecutorExperimentLogger::Row ExecutorExperimentLogger::base_row(
    const Session& session,
    const std::string& row_type) const
{
  Row row;
  row["run_id"] = run_id_;
  row["action_call_id"] = std::to_string(session.action_call_id);
  row["row_type"] = row_type;
  row["action_name"] = session.action_name;
  row["timestamp_start_iso"] = session.start_iso;
  row["t_rel_sec"] = format_double(elapsed_sec(session.start_time, node_->get_clock()->now()));
  row["execute_mode"] = session.execute_mode;
  row["base_frame"] = base_frame_;
  row["tcp_frame"] = tcp_frame_;
  row["sample_rate_hz"] = format_double(sample_rate_hz_);
  return row;
}

void ExecutorExperimentLogger::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_joint_state_ = msg;
}

void ExecutorExperimentLogger::sample_loop(const uint64_t action_call_id)
{
  const double period_sec = 1.0 / std::max(sample_rate_hz_, 1.0);
  const auto sleep_duration = std::chrono::duration<double>(period_sec);
  auto last_missing_warn = std::chrono::steady_clock::now() -
      std::chrono::duration<double>(kMissingJointStateWarnSec);

  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sampling_stop_requested_ || active_sampling_id_ != action_call_id)
      {
        break;
      }
    }

    write_actual_sample(action_call_id);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_joint_state_)
      {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_missing_warn).count() >=
            kMissingJointStateWarnSec)
        {
          RCLCPP_WARN(logger_, "Executor logger has not received /joint_states yet.");
          last_missing_warn = now;
        }
      }
    }

    std::this_thread::sleep_for(sleep_duration);
  }
}

void ExecutorExperimentLogger::write_actual_sample(const uint64_t action_call_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto session_it = sessions_.find(action_call_id);
  if (session_it == sessions_.end())
  {
    return;
  }

  auto& session = session_it->second;
  Row row = base_row(session, "actual_sample");
  const rclcpp::Time now = node_->get_clock()->now();
  const double rel_time = elapsed_sec(session.start_time, now);
  row["t_rel_sec"] = format_double(rel_time);

  std::string note;
  std::array<double, 6> actual_positions {};
  std::array<bool, 6> has_actual_position {};

  if (latest_joint_state_)
  {
    row["joint_state_timestamp"] =
        format_double(rclcpp::Time(latest_joint_state_->header.stamp).seconds());
    for (size_t expected_index = 0; expected_index < joint_names_.size(); ++expected_index)
    {
      const auto joint_it = std::find(
          latest_joint_state_->name.begin(),
          latest_joint_state_->name.end(),
          joint_names_[expected_index]);
      if (joint_it == latest_joint_state_->name.end())
      {
        continue;
      }
      const size_t source_index = static_cast<size_t>(
          std::distance(latest_joint_state_->name.begin(), joint_it));
      const std::string joint_number = std::to_string(expected_index + 1);
      if (source_index < latest_joint_state_->position.size())
      {
        actual_positions[expected_index] = latest_joint_state_->position[source_index];
        has_actual_position[expected_index] = true;
        row["joint_" + joint_number + "_position"] =
            format_double(latest_joint_state_->position[source_index]);
      }
      if (source_index < latest_joint_state_->velocity.size())
      {
        row["joint_" + joint_number + "_velocity"] =
            format_double(latest_joint_state_->velocity[source_index]);
      }
      if (source_index < latest_joint_state_->effort.size())
      {
        row["joint_" + joint_number + "_effort"] =
            format_double(latest_joint_state_->effort[source_index]);
      }
    }
  }
  else
  {
    note += "missing_joint_state";
  }

  geometry_msgs::msg::Pose actual_tcp_pose;
  bool has_tcp = false;
  try
  {
    const auto transform = tf_buffer_->lookupTransform(
        base_frame_, tcp_frame_, tf2::TimePointZero, tf2::durationFromSec(0.01));
    actual_tcp_pose.position.x = transform.transform.translation.x;
    actual_tcp_pose.position.y = transform.transform.translation.y;
    actual_tcp_pose.position.z = transform.transform.translation.z;
    actual_tcp_pose.orientation = transform.transform.rotation;
    has_tcp = true;

    double actual_roll = 0.0;
    double actual_pitch = 0.0;
    double actual_yaw = 0.0;
    quaternion_to_rpy(actual_tcp_pose.orientation, actual_roll, actual_pitch, actual_yaw);
    row["tcp_actual_timestamp"] = format_double(rclcpp::Time(transform.header.stamp).seconds());
    row["tcp_actual_x"] = format_double(actual_tcp_pose.position.x);
    row["tcp_actual_y"] = format_double(actual_tcp_pose.position.y);
    row["tcp_actual_z"] = format_double(actual_tcp_pose.position.z);
    row["tcp_actual_qx"] = format_double(actual_tcp_pose.orientation.x);
    row["tcp_actual_qy"] = format_double(actual_tcp_pose.orientation.y);
    row["tcp_actual_qz"] = format_double(actual_tcp_pose.orientation.z);
    row["tcp_actual_qw"] = format_double(actual_tcp_pose.orientation.w);
    row["tcp_actual_roll"] = format_double(actual_roll);
    row["tcp_actual_pitch"] = format_double(actual_pitch);
    row["tcp_actual_yaw"] = format_double(actual_yaw);
  }
  catch (const std::exception& ex)
  {
    if (!note.empty())
    {
      note += ";";
    }
    note += "missing_tcp_tf:";
    note += ex.what();
  }

  if (!session.command_points.empty())
  {
    size_t nearest_index = 0;
    double nearest_dt = std::numeric_limits<double>::max();
    for (size_t i = 0; i < session.command_points.size(); ++i)
    {
      const double dt = std::abs(session.command_points[i].time_from_start_sec - rel_time);
      if (dt < nearest_dt)
      {
        nearest_dt = dt;
        nearest_index = i;
      }
    }
    row["nearest_joint_cmd_index"] = std::to_string(nearest_index);
    const auto& command = session.command_points[nearest_index];
    double sum_sq = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (has_actual_position[i] && command.has_position[i])
      {
        const double error = actual_positions[i] - command.positions[i];
        row["joint_" + std::to_string(i + 1) + "_error"] = format_double(error);
        sum_sq += error * error;
        ++count;
      }
    }
    if (count > 0)
    {
      const double norm = std::sqrt(sum_sq);
      row["joint_error_norm"] = format_double(norm);
      session.joint_error_norms.push_back(norm);
      session.final_joint_error_norm = norm;
      session.has_final_joint_error_norm = true;
    }
  }

  if (has_tcp && !session.refs.empty())
  {
    size_t nearest_ref_index = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < session.refs.size(); ++i)
    {
      const auto& ref = session.refs[i];
      const double dx = actual_tcp_pose.position.x - ref.position.x;
      const double dy = actual_tcp_pose.position.y - ref.position.y;
      const double dz = actual_tcp_pose.position.z - ref.position.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_ref_index = i;
      }
    }

    const auto& ref = session.refs[nearest_ref_index];
    double ref_roll = 0.0;
    double ref_pitch = 0.0;
    double ref_yaw = 0.0;
    quaternion_to_rpy(ref.orientation, ref_roll, ref_pitch, ref_yaw);
    row["nearest_ref_index"] = std::to_string(nearest_ref_index);
    row["ref_index"] = std::to_string(nearest_ref_index);
    row["ref_x"] = format_double(ref.position.x);
    row["ref_y"] = format_double(ref.position.y);
    row["ref_z"] = format_double(ref.position.z);
    row["ref_qx"] = format_double(ref.orientation.x);
    row["ref_qy"] = format_double(ref.orientation.y);
    row["ref_qz"] = format_double(ref.orientation.z);
    row["ref_qw"] = format_double(ref.orientation.w);
    row["ref_roll"] = format_double(ref_roll);
    row["ref_pitch"] = format_double(ref_pitch);
    row["ref_yaw"] = format_double(ref_yaw);

    const double dx = actual_tcp_pose.position.x - ref.position.x;
    const double dy = actual_tcp_pose.position.y - ref.position.y;
    const double dz = actual_tcp_pose.position.z - ref.position.z;
    const double position_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    row["tcp_position_error_x"] = format_double(dx);
    row["tcp_position_error_y"] = format_double(dy);
    row["tcp_position_error_z"] = format_double(dz);
    row["tcp_position_error_norm"] = format_double(position_norm);
    session.tcp_position_error_norms.push_back(position_norm);
    session.final_tcp_position_error_norm = position_norm;
    session.has_final_tcp_position_error_norm = true;

    double error_roll = 0.0;
    double error_pitch = 0.0;
    double error_yaw = 0.0;
    quaternion_angle_error_roll_pitch_yaw(
        actual_tcp_pose.orientation, ref.orientation, error_roll, error_pitch, error_yaw);
    row["tcp_orientation_error_roll"] = format_double(error_roll);
    row["tcp_orientation_error_pitch"] = format_double(error_pitch);
    row["tcp_orientation_error_yaw"] = format_double(error_yaw);
  }

  row["note"] = note;
  ++session.actual_samples_count;
  write_row(row);
}

void ExecutorExperimentLogger::stop_sampling_locked(const uint64_t action_call_id)
{
  if (active_sampling_id_ == action_call_id)
  {
    sampling_stop_requested_ = true;
    active_sampling_id_ = 0;
  }
}

std::vector<std::string> ExecutorExperimentLogger::csv_header()
{
  return {
      "run_id", "action_call_id", "row_type", "action_name", "timestamp_start_iso",
      "timestamp_end_iso", "t_rel_sec", "execute_mode", "status", "success", "message",
      "duration_sec", "fraction", "joint_state_timestamp",
      "joint_1_position", "joint_2_position", "joint_3_position", "joint_4_position",
      "joint_5_position", "joint_6_position",
      "joint_1_velocity", "joint_2_velocity", "joint_3_velocity", "joint_4_velocity",
      "joint_5_velocity", "joint_6_velocity",
      "joint_1_effort", "joint_2_effort", "joint_3_effort", "joint_4_effort",
      "joint_5_effort", "joint_6_effort",
      "joint_cmd_timestamp", "joint_cmd_index", "joint_cmd_time_from_start_sec",
      "joint_1_cmd", "joint_2_cmd", "joint_3_cmd", "joint_4_cmd", "joint_5_cmd",
      "joint_6_cmd",
      "joint_1_cmd_velocity", "joint_2_cmd_velocity", "joint_3_cmd_velocity",
      "joint_4_cmd_velocity", "joint_5_cmd_velocity", "joint_6_cmd_velocity",
      "joint_1_cmd_acceleration", "joint_2_cmd_acceleration", "joint_3_cmd_acceleration",
      "joint_4_cmd_acceleration", "joint_5_cmd_acceleration", "joint_6_cmd_acceleration",
      "nearest_joint_cmd_index",
      "joint_1_error", "joint_2_error", "joint_3_error", "joint_4_error",
      "joint_5_error", "joint_6_error", "joint_error_norm",
      "tcp_actual_timestamp", "tcp_actual_x", "tcp_actual_y", "tcp_actual_z",
      "tcp_actual_qx", "tcp_actual_qy", "tcp_actual_qz", "tcp_actual_qw",
      "tcp_actual_roll", "tcp_actual_pitch", "tcp_actual_yaw",
      "ref_index", "ref_timestamp", "ref_x", "ref_y", "ref_z",
      "ref_qx", "ref_qy", "ref_qz", "ref_qw", "ref_roll", "ref_pitch", "ref_yaw",
      "nearest_ref_index",
      "tcp_position_error_x", "tcp_position_error_y", "tcp_position_error_z",
      "tcp_position_error_norm",
      "tcp_orientation_error_roll", "tcp_orientation_error_pitch",
      "tcp_orientation_error_yaw",
      "actual_samples_count", "joint_command_points_count", "ref_waypoints_count",
      "mean_joint_error_norm", "max_joint_error_norm", "rmse_joint_error_norm",
      "final_joint_error_norm",
      "mean_tcp_position_error_norm", "max_tcp_position_error_norm",
      "rmse_tcp_position_error_norm", "final_tcp_position_error_norm",
      "base_frame", "tcp_frame", "sample_rate_hz", "note",
      "stage_name", "metadata_json"};
}

std::string ExecutorExperimentLogger::csv_escape(const std::string& value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos)
  {
    return value;
  }

  std::string escaped = "\"";
  for (const char c : value)
  {
    if (c == '"')
    {
      escaped += "\"\"";
    }
    else
    {
      escaped += c;
    }
  }
  escaped += "\"";
  return escaped;
}

std::string ExecutorExperimentLogger::format_double(const double value)
{
  if (!std::isfinite(value))
  {
    return "";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(9) << value;
  return out.str();
}

std::string ExecutorExperimentLogger::format_bool(const bool value)
{
  return value ? "true" : "false";
}

std::string ExecutorExperimentLogger::iso_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm {};
  localtime_r(&now_time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
      << "." << std::setw(3) << std::setfill('0') << ms.count();
  return out.str();
}

double ExecutorExperimentLogger::elapsed_sec(
    const rclcpp::Time& start,
    const rclcpp::Time& end)
{
  return (end - start).seconds();
}

double ExecutorExperimentLogger::duration_sec(
    const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(duration.sec) +
      static_cast<double>(duration.nanosec) * 1e-9;
}

double ExecutorExperimentLogger::quaternion_angle_error_roll_pitch_yaw(
    const geometry_msgs::msg::Quaternion& actual,
    const geometry_msgs::msg::Quaternion& ref,
    double& roll,
    double& pitch,
    double& yaw)
{
  tf2::Quaternion qa(actual.x, actual.y, actual.z, actual.w);
  tf2::Quaternion qr(ref.x, ref.y, ref.z, ref.w);
  if (qa.length2() <= 1e-18 || qr.length2() <= 1e-18)
  {
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    return 0.0;
  }
  qa.normalize();
  qr.normalize();
  const tf2::Quaternion q_error = qr.inverse() * qa;
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q_error.x();
  q_msg.y = q_error.y();
  q_msg.z = q_error.z();
  q_msg.w = q_error.w();
  quaternion_to_rpy(q_msg, roll, pitch, yaw);
  return std::sqrt(roll * roll + pitch * pitch + yaw * yaw);
}

void ExecutorExperimentLogger::quaternion_to_rpy(
    const geometry_msgs::msg::Quaternion& q,
    double& roll,
    double& pitch,
    double& yaw)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  if (quat.length2() <= 1e-18)
  {
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    return;
  }
  quat.normalize();
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

double ExecutorExperimentLogger::mean(const std::vector<double>& values)
{
  if (values.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum = 0.0;
  for (const double value : values)
  {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

double ExecutorExperimentLogger::max_value(const std::vector<double>& values)
{
  if (values.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return *std::max_element(values.begin(), values.end());
}

double ExecutorExperimentLogger::rmse(const std::vector<double>& values)
{
  if (values.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum_sq = 0.0;
  for (const double value : values)
  {
    sum_sq += value * value;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

bool ExecutorExperimentLogger::mkdirs(const std::string& path)
{
  try
  {
    std::filesystem::create_directories(path);
    return std::filesystem::exists(path);
  }
  catch (const std::exception&)
  {
    return false;
  }
}

}  // namespace robot_task_executor
