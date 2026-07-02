#pragma once

#include <filesystem>
#include <string>

namespace robot_task_manager
{

inline constexpr const char * kDefaultLogRootDir = "/home/minhquang/ros2_dev/Log_robot_data";

inline std::string absoluteLogPath(const std::string & path)
{
  std::filesystem::path p(path.empty() ? kDefaultLogRootDir : path);
  if (p.is_relative()) {
    p = std::filesystem::current_path() / p;
  }
  return std::filesystem::absolute(p).lexically_normal().string();
}

inline std::string actionMetricsLogDir(const std::string & log_root_dir, const std::string & action_name)
{
  return (std::filesystem::path(absoluteLogPath(log_root_dir)) / "action_metrics" / action_name).string();
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
  std::filesystem::path base = configured_executor_dir.empty() ?
    std::filesystem::path(executorLogBaseDir(log_root_dir)) :
    std::filesystem::path(absoluteLogPath(configured_executor_dir));
  if (base.filename() != action_name) {
    base /= action_name;
  }
  return base.lexically_normal().string();
}

}  // namespace robot_task_manager
