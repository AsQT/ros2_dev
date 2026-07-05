#pragma once

// codex.md §5/§7: best-effort "plot after finish" hook.
//
// runLogPlotsAsync() launches scripts/plot_action_log.py on a finished
// call directory in a DETACHED background thread. It never blocks the action
// server and never throws, so a plotting failure can never fail an action.
// Gated by the caller's enable_log_plots flag.

#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace robot_task_manager
{

// Resolve the installed plot script from AMENT_PREFIX_PATH (dependency-free).
inline std::string resolvePlotScript()
{
  const char * ap = std::getenv("AMENT_PREFIX_PATH");
  if (!ap) {
    return "";
  }
  std::string s(ap);
  size_t start = 0;
  while (start <= s.size()) {
    const size_t pos = s.find(':', start);
    const std::string prefix =
      s.substr(start, pos == std::string::npos ? std::string::npos : pos - start);
    if (!prefix.empty()) {
      const auto cand = std::filesystem::path(prefix) / "lib" / "robot_task_manager" /
        "plot_action_log.py";
      std::error_code ec;
      if (std::filesystem::exists(cand, ec)) {
        return cand.string();
      }
    }
    if (pos == std::string::npos) {
      break;
    }
    start = pos + 1;
  }
  return "";
}

// Best-effort, non-blocking, never-throws. Safe to call at any finish() path.
inline void runLogPlotsAsync(
  const rclcpp::Logger & logger, const std::string & call_dir, bool enable)
{
  if (!enable || call_dir.empty()) {
    return;
  }
  const std::string script = resolvePlotScript();
  if (script.empty()) {
    RCLCPP_WARN(logger, "enable_log_plots=true but plot_action_log.py not found on AMENT_PREFIX_PATH");
    return;
  }
  try {
    std::thread([script, call_dir]() {
      // Quotes guard against spaces; --quiet keeps stdout clean. The script
      // itself always exits 0, so this never signals failure.
      const std::string cmd =
        "python3 '" + script + "' '" + call_dir + "' --quiet >/dev/null 2>&1";
      std::system(cmd.c_str());
    }).detach();
  } catch (...) {
    // Even thread creation failure must not affect the action.
    RCLCPP_WARN(logger, "Failed to launch background log-plot thread (ignored)");
  }
}

// codex.md §5/§6: read the use_mock / hardware_plugin / enable_log_plots node
// parameters (declared by the server) and push them into any logger exposing
// setHardwareInfo()/setPlotsEnabled() (PerCallTcpLogger, ActionMetricsLogger).
// Safe with a null logger. Params must already be declared.
template<typename LoggerPtr>
inline void applyLogProvenanceFromParams(rclcpp::Node * node, const LoggerPtr & logger)
{
  if (!node || !logger) {
    return;
  }
  const std::string use_mock = node->has_parameter("use_mock") ?
    (node->get_parameter("use_mock").as_bool() ? "true" : "false") : "unknown";
  const std::string hw = node->has_parameter("hardware_plugin") ?
    node->get_parameter("hardware_plugin").as_string() : "unknown";
  const bool plots = node->has_parameter("enable_log_plots") ?
    node->get_parameter("enable_log_plots").as_bool() : true;
  logger->setHardwareInfo(use_mock, hw);
  logger->setPlotsEnabled(plots);
}

}  // namespace robot_task_manager
