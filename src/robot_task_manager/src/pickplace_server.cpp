#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robot_task_manager/action/pick_place.hpp"
#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/log_paths.hpp"
#include "robot_task_manager/log_plot_hook.hpp"
#include "robot_task_manager/standard_action_logger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "robot_task_executor/executor_experiment_logger.hpp"
#include <map>
#include <vector>

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// PickPlaceTcpLogger
//
// Dedicated, self-contained per-call CSV logger for /pickplace only. Separate
// from ExecutorExperimentLogger (which stays untouched/shared across nodes) —
// this one creates ONE CSV file per /pickplace call, containing only TCP
// set-vs-actual pose data for that single call, plus an index.csv listing all
// calls made during this node's run. Not intended for reuse by other action
// servers.
//
// Unlike PerCallTcpLogger (per_call_tcp_logger.hpp, used by /move_to_pose,
// /move_to_pose_cartesian, /repeatability_test, /move_checker_board), this
// logger keeps its own 33-column schema (fixed pick/place/velocity_scale
// metadata columns instead of a generic metadata_json column). It is still
// opt-in per goal via goal.enable_tcp_log so the GUI Log toggle can reliably
// control whether a CSV is created. It is exercised/tested independently —
// see Report/pickplace_tcp_per_call_csv_report.md and
// Report/pickplace_tcp_logger_cleanup_report.md.
// -----------------------------------------------------------------------------
class PickPlaceTcpLogger
{
public:
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

    // Goal-level metadata, constant for the whole call — repeated on every
    // row so a single pickplace_NNNN.csv is self-describing without having
    // to cross-reference index.csv.
    double velocity_scale = 0.0;
    double pick_x = 0.0, pick_y = 0.0, pick_z = 0.0;
    double place_x = 0.0, place_y = 0.0, place_z = 0.0;
  };

  PickPlaceTcpLogger(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    std::string log_dir,
    double sample_rate_hz,
    std::string base_frame,
    std::string tcp_frame)
  : node_(node),
    tf_buffer_(tf_buffer),
    log_dir_(std::filesystem::absolute(std::filesystem::path(std::move(log_dir))).lexically_normal().string()),
    sample_rate_hz_(sample_rate_hz > 0.0 ? sample_rate_hz : 50.0),
    base_frame_(std::move(base_frame)),
    tcp_frame_(std::move(tcp_frame))
  {
    prepareRunDir();
  }

  ~PickPlaceTcpLogger() = default;

  std::shared_ptr<Call> startCall(
    double velocity_scale,
    const geometry_msgs::msg::Point & pick,
    const geometry_msgs::msg::Point & place)
  {
    if (!ready_) {
      return nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto call = std::make_shared<Call>();
    call->call_index = ++next_call_index_;
    std::ostringstream call_id;
    call_id << "call_" << std::setw(4) << std::setfill('0') << call->call_index;
    call->action_call_id = call_id.str();
    call->start_time = node_->get_clock()->now();
    call->start_iso = isoNow();
    call->velocity_scale = velocity_scale;
    call->pick_x = pick.x; call->pick_y = pick.y; call->pick_z = pick.z;
    call->place_x = place.x; call->place_y = place.y; call->place_z = place.z;

    call->call_dir = (std::filesystem::path(run_dir_) / call->action_call_id).string();
    std::error_code ec;
    std::filesystem::create_directories(call->call_dir, ec);
    if (ec) {
      RCLCPP_WARN(node_->get_logger(), "PickPlaceTcpLogger: cannot create %s", call->call_dir.c_str());
      return nullptr;
    }
    call->csv_filename = "trajectory_tracking.csv";
    call->csv_path = (std::filesystem::path(call->call_dir) / call->csv_filename).string();

    call->csv.open(call->csv_path, std::ios::out | std::ios::trunc);
    if (!call->csv.is_open()) {
      RCLCPP_WARN(node_->get_logger(), "Failed to write log CSV: %s", call->csv_path.c_str());
      RCLCPP_WARN(node_->get_logger(), "PickPlaceTcpLogger: cannot open %s", call->csv_path.c_str());
      return nullptr;
    }
    writeHeader(call->csv);
    writeMetadata(*call);
    appendEvent(*call, "start", "action_start", "", "PickPlace log created");
    RCLCPP_INFO(node_->get_logger(), "Executor CSV saved: %s", call->csv_path.c_str());

    // Seed stage/set-pose immediately so no sample ever has an empty stage or
    // a blank set pose: default stage is "pickplace_start", and the initial
    // "set" TCP pose is whatever the robot is actually at right now (the
    // implicit target while only the gripper is being commanded, before the
    // first real motion goal is known).
    {
      std::lock_guard<std::mutex> state_lock(call->state_mutex);
      call->current_stage = "pickplace_start";
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
  // tracked (used for gripper-only stages that don't have a new TCP target).
  void setStage(const std::shared_ptr<Call> & call, const std::string & stage)
  {
    if (!call) {
      return;
    }
    std::lock_guard<std::mutex> lock(call->state_mutex);
    call->current_stage = stage;
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

  void startSampling(const std::shared_ptr<Call> & call)
  {
    if (!call || call->sampling.load()) {
      return;
    }
    call->sampling.store(true);
    call->sample_thread = std::thread(&PickPlaceTcpLogger::sampleLoop, this, call);
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
    writeRow(call, "summary", "pickplace_end", has_set ? &set_pose : nullptr, status, message, &success);
    call->csv.flush();
    call->csv.close();

    const std::string end_iso = isoNow();
    const double duration_sec = (node_->get_clock()->now() - call->start_time).seconds();
    writeSummary(*call, end_iso, duration_sec, status, success, message);
    appendEvent(*call, "pickplace_end", "action_result", success ? "true" : "false", message);
    appendIndexRow(*call, end_iso, duration_sec, status, success, message);
    // codex.md §2: task-level evaluation files derived from the tracking rows.
    writeTaskEvalFiles(*call, duration_sec, success, message);
    robot_task_manager::runLogPlotsAsync(node_->get_logger(), call->call_dir, true);
  }

private:
  // codex.md §2: derive phase_summary.csv, tcp_tracking.csv (RPY/phase),
  // object_tracking.csv and a task-level summary.csv by post-processing the
  // trajectory_tracking.csv rows this call already wrote. No robot interaction.
  struct PhaseAgg
  {
    std::string phase;
    double t_first = std::numeric_limits<double>::quiet_NaN();
    double t_last = std::numeric_limits<double>::quiet_NaN();
    double sx = 0, sy = 0, sz = 0, sroll = 0, spitch = 0, syaw = 0;   // start actual
    double fx = 0, fy = 0, fz = 0, froll = 0, fpitch = 0, fyaw = 0;   // final actual
    double tx = 0, ty = 0, tz = 0, troll = 0, tpitch = 0, tyaw = 0;   // last set/target
    double final_pos_err = std::numeric_limits<double>::quiet_NaN();
    double final_ori_err = std::numeric_limits<double>::quiet_NaN();
    double path_len = 0.0;
    bool has_prev = false;
    double px = 0, py = 0, pz = 0;
    bool seeded = false;
  };

  static std::vector<std::string> splitCsv(const std::string & line)
  {
    std::vector<std::string> out;
    std::string cur;
    bool q = false;
    for (size_t i = 0; i < line.size(); ++i) {
      const char c = line[i];
      if (q) {
        if (c == '"') {
          if (i + 1 < line.size() && line[i + 1] == '"') {cur += '"'; ++i;} else {q = false;}
        } else {cur += c;}
      } else if (c == '"') {q = true;}
      else if (c == ',') {out.push_back(cur); cur.clear();}
      else {cur += c;}
    }
    out.push_back(cur);
    return out;
  }

  static double toRpyComponent(double qx, double qy, double qz, double qw, int idx)
  {
    tf2::Quaternion tq(qx, qy, qz, qw);
    if (tq.length2() <= 1e-12) {return std::numeric_limits<double>::quiet_NaN();}
    tq.normalize();
    double r, p, y;
    tf2::Matrix3x3(tq).getRPY(r, p, y);
    return idx == 0 ? r : (idx == 1 ? p : y);
  }

  void writeTaskEvalFiles(const Call & call, double total_time_s, bool success,
    const std::string & message)
  {
    const std::filesystem::path dir(call.call_dir);
    std::ifstream in(call.csv_path);
    if (!in.is_open()) {
      return;
    }
    std::string line;
    if (!std::getline(in, line)) {return;}
    const auto header = splitCsv(line);
    auto idx = [&header](const std::string & n) -> int {
      for (size_t i = 0; i < header.size(); ++i) {if (header[i] == n) {return static_cast<int>(i);}}
      return -1;
    };
    const int i_t = idx("time_s"), i_stage = idx("stage");
    const int i_ax = idx("actual_x"), i_ay = idx("actual_y"), i_az = idx("actual_z");
    const int i_aqx = idx("actual_qx"), i_aqy = idx("actual_qy"), i_aqz = idx("actual_qz"), i_aqw = idx("actual_qw");
    const int i_sx = idx("set_x"), i_sy = idx("set_y"), i_sz = idx("set_z");
    const int i_sqx = idx("set_qx"), i_sqy = idx("set_qy"), i_sqz = idx("set_qz"), i_sqw = idx("set_qw");
    const int i_pe = idx("position_error_m"), i_oe = idx("orientation_error_rad");

    auto num = [](const std::vector<std::string> & f, int i) -> double {
      if (i < 0 || i >= static_cast<int>(f.size()) || f[i].empty()) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      try {return std::stod(f[i]);} catch (...) {return std::numeric_limits<double>::quiet_NaN();}
    };

    std::vector<PhaseAgg> phases;
    std::map<std::string, size_t> phase_idx;
    std::ofstream tcp(dir / "tcp_tracking.csv", std::ios::out | std::ios::trunc);
    if (tcp.is_open()) {
      tcp << "t_s,phase,tcp_x,tcp_y,tcp_z,tcp_roll,tcp_pitch,tcp_yaw,"
             "target_x,target_y,target_z,target_roll,target_pitch,target_yaw,"
             "position_error_m,orientation_error_rad,distance_to_phase_target,path_length_so_far_m\n";
    }
    double total_path = 0.0, max_err = 0.0, sum_pe2 = 0.0, sum_oe2 = 0.0;
    uint64_t nerr = 0;
    bool have_last = false;
    double lx = 0, ly = 0, lz = 0;
    auto fmt = [](double v) {
      if (!std::isfinite(v)) {return std::string();}
      std::ostringstream o; o << std::fixed << std::setprecision(6) << v; return o.str();
    };

    while (std::getline(in, line)) {
      const auto f = splitCsv(line);
      const std::string stage = (i_stage >= 0 && i_stage < static_cast<int>(f.size())) ? f[i_stage] : "";
      const double t = num(f, i_t);
      const double ax = num(f, i_ax), ay = num(f, i_ay), az = num(f, i_az);
      const double aroll = toRpyComponent(num(f, i_aqx), num(f, i_aqy), num(f, i_aqz), num(f, i_aqw), 0);
      const double apitch = toRpyComponent(num(f, i_aqx), num(f, i_aqy), num(f, i_aqz), num(f, i_aqw), 1);
      const double ayaw = toRpyComponent(num(f, i_aqx), num(f, i_aqy), num(f, i_aqz), num(f, i_aqw), 2);
      const double sx = num(f, i_sx), sy = num(f, i_sy), sz = num(f, i_sz);
      const double sroll = toRpyComponent(num(f, i_sqx), num(f, i_sqy), num(f, i_sqz), num(f, i_sqw), 0);
      const double spitch = toRpyComponent(num(f, i_sqx), num(f, i_sqy), num(f, i_sqz), num(f, i_sqw), 1);
      const double syaw = toRpyComponent(num(f, i_sqx), num(f, i_sqy), num(f, i_sqz), num(f, i_sqw), 2);
      const double pe = num(f, i_pe), oe = num(f, i_oe);

      double step = 0.0;
      if (have_last && std::isfinite(ax)) {
        const double dx = ax - lx, dy = ay - ly, dz = az - lz;
        step = std::sqrt(dx * dx + dy * dy + dz * dz);
        total_path += step;
      }
      if (std::isfinite(ax)) {lx = ax; ly = ay; lz = az; have_last = true;}
      if (std::isfinite(pe)) {sum_pe2 += pe * pe; max_err = std::max(max_err, pe); ++nerr;}
      if (std::isfinite(oe)) {sum_oe2 += oe * oe;}
      double dist_phase_target = std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(sx) && std::isfinite(ax)) {
        dist_phase_target = std::sqrt((sx - ax) * (sx - ax) + (sy - ay) * (sy - ay) + (sz - az) * (sz - az));
      }
      if (tcp.is_open()) {
        tcp << fmt(t) << "," << stage << "," << fmt(ax) << "," << fmt(ay) << "," << fmt(az) << ","
            << fmt(aroll) << "," << fmt(apitch) << "," << fmt(ayaw) << ","
            << fmt(sx) << "," << fmt(sy) << "," << fmt(sz) << ","
            << fmt(sroll) << "," << fmt(spitch) << "," << fmt(syaw) << ","
            << fmt(pe) << "," << fmt(oe) << "," << fmt(dist_phase_target) << "," << fmt(total_path) << "\n";
      }

      // per-phase aggregation
      auto pit = phase_idx.find(stage);
      if (pit == phase_idx.end()) {
        phase_idx[stage] = phases.size();
        phases.push_back(PhaseAgg{});
        phases.back().phase = stage;
        pit = phase_idx.find(stage);
      }
      PhaseAgg & pa = phases[pit->second];
      if (!pa.seeded && std::isfinite(ax)) {
        pa.t_first = t; pa.sx = ax; pa.sy = ay; pa.sz = az; pa.sroll = aroll; pa.spitch = apitch; pa.syaw = ayaw;
        pa.seeded = true;
      }
      pa.t_last = t;
      if (std::isfinite(ax)) {pa.fx = ax; pa.fy = ay; pa.fz = az; pa.froll = aroll; pa.fpitch = apitch; pa.fyaw = ayaw;}
      if (std::isfinite(sx)) {pa.tx = sx; pa.ty = sy; pa.tz = sz; pa.troll = sroll; pa.tpitch = spitch; pa.tyaw = syaw;}
      if (std::isfinite(pe)) {pa.final_pos_err = pe;}
      if (std::isfinite(oe)) {pa.final_ori_err = oe;}
      if (pa.has_prev && std::isfinite(ax)) {
        const double dx = ax - pa.px, dy = ay - pa.py, dz = az - pa.pz;
        pa.path_len += std::sqrt(dx * dx + dy * dy + dz * dz);
      }
      if (std::isfinite(ax)) {pa.px = ax; pa.py = ay; pa.pz = az; pa.has_prev = true;}
    }
    if (tcp.is_open()) {tcp.flush();}

    // phase_summary.csv
    std::ofstream ps(dir / "phase_summary.csv", std::ios::out | std::ios::trunc);
    if (ps.is_open()) {
      ps << "phase,start_time_s,end_time_s,duration_s,success,failed_reason,"
            "start_tcp_x,start_tcp_y,start_tcp_z,start_tcp_roll,start_tcp_pitch,start_tcp_yaw,"
            "target_tcp_x,target_tcp_y,target_tcp_z,target_tcp_roll,target_tcp_pitch,target_tcp_yaw,"
            "final_tcp_x,final_tcp_y,final_tcp_z,final_tcp_roll,final_tcp_pitch,final_tcp_yaw,"
            "final_position_error_m,final_orientation_error_rad,path_length_m,planning_time_s,execution_time_s,"
            "rl_num_points,moveit_num_points,min_obstacle_clearance_m\n";
      for (const auto & pa : phases) {
        const double dur = (std::isfinite(pa.t_first) && std::isfinite(pa.t_last)) ? (pa.t_last - pa.t_first) : std::numeric_limits<double>::quiet_NaN();
        ps << pa.phase << "," << fmt(pa.t_first) << "," << fmt(pa.t_last) << "," << fmt(dur) << ",,"  // success,failed_reason: per-phase not tracked
           << "," << fmt(pa.sx) << "," << fmt(pa.sy) << "," << fmt(pa.sz) << "," << fmt(pa.sroll) << "," << fmt(pa.spitch) << "," << fmt(pa.syaw)
           << "," << fmt(pa.tx) << "," << fmt(pa.ty) << "," << fmt(pa.tz) << "," << fmt(pa.troll) << "," << fmt(pa.tpitch) << "," << fmt(pa.tyaw)
           << "," << fmt(pa.fx) << "," << fmt(pa.fy) << "," << fmt(pa.fz) << "," << fmt(pa.froll) << "," << fmt(pa.fpitch) << "," << fmt(pa.fyaw)
           << "," << fmt(pa.final_pos_err) << "," << fmt(pa.final_ori_err) << "," << fmt(pa.path_len)
           << ",,,not_applicable,not_applicable,not_applicable\n";  // planning/execution time not separable; RL/obstacle N/A for baseline
      }
    }

    // object_tracking.csv — initial pick/place object poses from the goal.
    std::ofstream ot(dir / "object_tracking.csv", std::ios::out | std::ios::trunc);
    if (ot.is_open()) {
      ot << "data_available,empty_reason,t_s,phase,object_id,object_source,"
            "object_x,object_y,object_z,object_roll,object_pitch,object_yaw,"
            "object_confidence,object_attached,object_dropped\n";
      ot << "true,,0,pick,unknown,goal_pose," << fmt(call.pick_x) << "," << fmt(call.pick_y) << "," << fmt(call.pick_z) << ",,,,,,\n";
      ot << "true,," << fmt(total_time_s) << ",place,unknown,goal_pose," << fmt(call.place_x) << "," << fmt(call.place_y) << "," << fmt(call.place_z) << ",,,,,,\n";
    }

    // task-level summary.csv
    const double rmse_pos = nerr ? std::sqrt(sum_pe2 / static_cast<double>(nerr)) : std::numeric_limits<double>::quiet_NaN();
    const double rmse_ori = nerr ? std::sqrt(sum_oe2 / static_cast<double>(nerr)) : std::numeric_limits<double>::quiet_NaN();
    auto phaseTime = [&phases](const std::string & key) {
      double s = 0.0; bool any = false;
      for (const auto & pa : phases) {
        if (pa.phase.find(key) != std::string::npos && std::isfinite(pa.t_first) && std::isfinite(pa.t_last)) {
          s += (pa.t_last - pa.t_first); any = true;
        }
      }
      return any ? s : std::numeric_limits<double>::quiet_NaN();
    };
    std::ofstream ts(dir / "summary.csv", std::ios::out | std::ios::trunc);
    if (ts.is_open()) {
      ts << "action_name,hardware_mode,evaluation_group,run_id,call_id,goal_uuid,task_success,failed_phase,"
            "failed_stage,failure_reason,message,execute_requested,planning_only,"
            "total_time_s,total_planning_time_s,total_execution_time_s,pick_time_s,place_time_s,"
            "pick_success,place_success,pick_position_error_m,place_position_error_m,"
            "pick_orientation_error_rad,place_orientation_error_rad,"
            "total_tcp_path_length_m,max_tcp_error_m,rmse_tcp_position_m,rmse_tcp_orientation_rad,path_efficiency,"
            "object_source,object_id,object_dropped,total_rl_points,total_moveit_points,"
            "min_obstacle_clearance_m,collision_detected,workspace_violation\n";
      ts << "pick_place," << runtimeModeFromRunDir() << ",03_task_execution_eval,"
         << std::filesystem::path(run_dir_).filename().string() << "," << call.action_call_id << ",,"
         << (success ? "true" : "false") << ",,"
         << (success ? "" : "see_message") << ",," << csvEscape(message) << ",true,false,"
         << fmt(total_time_s) << ",,," << fmt(phaseTime("pick")) << "," << fmt(phaseTime("place")) << ","
         << (success ? "true" : "") << "," << (success ? "true" : "") << ",,,,,"
         << fmt(total_path) << "," << fmt(max_err) << "," << fmt(rmse_pos) << "," << fmt(rmse_ori) << ",,"
         << "goal_pose,unknown,,not_applicable,not_applicable,not_applicable,,\n";
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
        // TF/lookup issue inside writeRow is already handled there; anything
        // else here just means we skip this one sample and keep going.
      }

      const auto elapsed = std::chrono::steady_clock::now() - loop_start;
      const auto sleep_for = std::chrono::duration<double>(period_sec) - elapsed;
      if (sleep_for > std::chrono::duration<double>(0)) {
        std::this_thread::sleep_for(sleep_for);
      }
    }
  }

  // Best-effort TF lookup, shared by the sampling loop and the initial-pose
  // seed in startCall(). Never throws — returns false if TF isn't ready yet.
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
      // TF not ready yet (common for the first few samples right after
      // launch) — do not crash, just report why via the optional warning.
      if (warning) {
        *warning = std::string("TF unavailable: ") + e.what();
      }
      return false;
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
    (void)row_type;
    row << formatDouble(time_sec) << ","
        << csvEscape(stage) << ",";

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

    std::string full_message = message;
    if (!tf_warning.empty()) {
      full_message = full_message.empty() ? tf_warning : (full_message + " | " + tf_warning);
    }
    row << csvEscape(status) << ",";
    if (success) {
      row << (*success ? "true" : "false");
    }
    row << "," << csvEscape(full_message);

    std::lock_guard<std::mutex> lock(mutex_);
    call->csv << row.str() << "\n";
    call->csv.flush();
    ++call->row_count;
  }

  void writeHeader(std::ofstream & csv)
  {
    csv << "time_s,stage,"
           "set_x,set_y,set_z,set_qx,set_qy,set_qz,set_qw,"
           "actual_x,actual_y,actual_z,actual_qx,actual_qy,actual_qz,actual_qw,"
           "error_x,error_y,error_z,position_error_m,orientation_error_rad,"
           "status,success,message\n";
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
      idx << "call_index,action_name,call_dir,start_time,end_time,duration_sec,"
             "status,success,message,row_count\n";
      ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "PickPlaceTcpLogger run dir: %s", run_dir_.c_str());
      return;
    }
    RCLCPP_WARN(node_->get_logger(), "Failed to create log directory: %s", log_dir_.c_str());
    RCLCPP_WARN(node_->get_logger(), "PickPlaceTcpLogger: could not create a unique run dir under %s",
      log_dir_.c_str());
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
        << "/pickplace" << ","
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
    out << csvEscape(isoNow()) << ","
        << formatDouble((node_->get_clock()->now() - call.start_time).seconds()) << ","
        << csvEscape(stage) << "," << csvEscape(event_type) << ","
        << success << "," << csvEscape(message) << "\n";
  }

  void writeMetadata(const Call & call)
  {
    std::ofstream out(std::filesystem::path(call.call_dir) / "metadata.json", std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << "{\n"
        << "  \"runtime_mode\": \"" << runtimeModeFromRunDir() << "\",\n"
        << "  \"log_group\": \"baseline\",\n"
        << "  \"action_name\": \"pick_place\",\n"
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
    std::ofstream out(std::filesystem::path(call.call_dir) / "summary.csv", std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      return;
    }
    out << robot_task_manager::standardSummaryHeader()
        << ",pick_x,pick_y,pick_z,place_x,place_y,place_z,execution_time_s,final_position_error_m,final_orientation_error_rad,object_grasp_success,object_place_success\n";
    out << runtimeModeFromRunDir() << ",baseline,pick_place,"
        << std::filesystem::path(run_dir_).filename().string() << ","
        << call.action_call_id << ",,,"
        << call.start_iso << "," << end_iso << ",,"
        << (success ? "true" : "false") << ","
        << (success ? "" : csvEscape(status)) << ","
        << (success ? "" : csvEscape(status)) << ","
        << csvEscape(message) << ","
        << formatDouble(duration_sec) << ","
        << formatDouble(call.pick_x) << "," << formatDouble(call.pick_y) << "," << formatDouble(call.pick_z) << ","
        << formatDouble(call.place_x) << "," << formatDouble(call.place_y) << "," << formatDouble(call.place_z) << ","
        << formatDouble(duration_sec) << ",,,"
        << (success ? "true" : "false") << "," << (success ? "true" : "false") << "\n";
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

  std::mutex mutex_;
  std::string run_dir_;
  std::string index_path_;
  uint32_t next_call_index_ = 0;
  bool ready_ = false;
};

class PickPlaceActionServer : public rclcpp::Node
{
public:
  using PickPlace = robot_task_manager::action::PickPlace;
  using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;

  using MoveToPose = robot_task_manager::action::MoveToPose;
  using MoveToPoseGoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

  using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
  using MoveToPoseCartesianGoalHandle =
    rclcpp_action::ClientGoalHandle<MoveToPoseCartesian>;

  using MoveGripper = robot_task_manager::action::MoveGripper;
  using MoveGripperGoalHandle = rclcpp_action::ClientGoalHandle<MoveGripper>;

  PickPlaceActionServer()
  : Node("pickplace_action_server")
  {
    approach_height_ = declare_parameter<double>("approach_height", 0.050);
    pre_pick_z_offset_ = declare_parameter<double>("pre_pick_z_offset_m", 0.05);
    RCLCPP_INFO(get_logger(), "PickPlace pre_pick_z_offset_m=%.4f", pre_pick_z_offset_);
    open_gripper_position_ = declare_parameter<double>("open_gripper_position", 0.049);
    server_wait_timeout_s_ = declare_parameter<double>("server_wait_timeout_s", 5.0);
    action_result_timeout_s_ = declare_parameter<double>("action_result_timeout_s", 90.0);

    enable_executor_logging_ = declare_parameter<bool>("enable_executor_logging", false);
    log_root_dir_            = declare_parameter<std::string>("log_root_dir", robot_task_manager::kDefaultLogRootDir);
    runtime_mode_            = declare_parameter<std::string>("runtime_mode", "mock");
    executor_log_dir_        = declare_parameter<std::string>(
      "executor_log_dir", robot_task_manager::executorLogBaseDir(log_root_dir_));
    executor_sample_rate_hz_ = declare_parameter<double>("executor_sample_rate_hz", 50.0);
    executor_base_frame_     = declare_parameter<std::string>("executor_base_frame", "base_link");
    executor_tcp_frame_      = declare_parameter<std::string>("executor_tcp_frame", "tcp_link");

    move_to_pose_client_ =
      rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");

    move_to_pose_cartesian_client_ =
      rclcpp_action::create_client<MoveToPoseCartesian>(this, "move_to_pose_cartesian");

    move_gripper_client_ =
      rclcpp_action::create_client<MoveGripper>(this, "move_gripper");

    action_server_ = rclcpp_action::create_server<PickPlace>(
      this,
      "pickplace",
      std::bind(&PickPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PickPlaceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PickPlaceActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PickPlace action server ready: /pickplace");
  }

  void initialize_logging()
  {
    if (enable_executor_logging_) {
      try {
        log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*log_tf_buffer_);
        logger_ = std::make_shared<robot_task_executor::ExecutorExperimentLogger>(
          shared_from_this(), log_tf_buffer_,
          robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "PickPlace"),
          executor_sample_rate_hz_,
          executor_base_frame_, executor_tcp_frame_);
      } catch (const std::exception & e) {
        logger_.reset();
        RCLCPP_WARN(get_logger(), "PickPlace CSV logger unavailable: %s", e.what());
      }
    }

    // Dedicated per-call TCP set/actual logger (one CSV file per /pickplace
    // call). Reuses the same tf_buffer/params as the shared logger above but
    // is otherwise fully independent of it.
    try {
      if (!log_tf_buffer_) {
        log_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        log_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*log_tf_buffer_);
      }
      tcp_logger_ = std::make_shared<PickPlaceTcpLogger>(
        shared_from_this(), log_tf_buffer_,
        robot_task_manager::executorActionLogDir(log_root_dir_, executor_log_dir_, runtime_mode_, "PickPlace"),
        executor_sample_rate_hz_,
        executor_base_frame_, executor_tcp_frame_);
    } catch (const std::exception & e) {
      tcp_logger_.reset();
      RCLCPP_WARN(get_logger(), "PickPlace TCP per-call CSV logger unavailable: %s", e.what());
    }
  }

private:
  double approach_height_ = 0.05;
  double pre_pick_z_offset_ = 0.05;
  double open_gripper_position_ = 0.049;
  double server_wait_timeout_s_ = 5.0;
  double action_result_timeout_s_ = 90.0;

  bool enable_executor_logging_{false};
  std::string log_root_dir_;
  std::string runtime_mode_;
  std::string executor_log_dir_;
  double executor_sample_rate_hz_{50.0};
  std::string executor_base_frame_;
  std::string executor_tcp_frame_;
  std::shared_ptr<tf2_ros::Buffer> log_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> log_tf_listener_;
  std::shared_ptr<robot_task_executor::ExecutorExperimentLogger> logger_;
  std::shared_ptr<PickPlaceTcpLogger> tcp_logger_;
  uint64_t action_call_id_{0};

  rclcpp_action::Server<PickPlace>::SharedPtr action_server_;

  rclcpp_action::Client<MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp_action::Client<MoveToPoseCartesian>::SharedPtr move_to_pose_cartesian_client_;
  rclcpp_action::Client<MoveGripper>::SharedPtr move_gripper_client_;

  std::mutex active_goal_mutex_;

  MoveToPoseGoalHandle::SharedPtr active_move_to_pose_goal_;
  MoveToPoseCartesianGoalHandle::SharedPtr active_move_to_pose_cartesian_goal_;
  MoveGripperGoalHandle::SharedPtr active_move_gripper_goal_;

  // codex.md (goal-rejected diagnosis): whether any downstream sub-goal is still
  // held. /pickplace uses ACCEPT_AND_EXECUTE and never rejects on "busy", so this
  // is reported for diagnostics only — it is NOT a reject condition.
  bool downstream_busy()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    return active_move_to_pose_goal_ || active_move_to_pose_cartesian_goal_ ||
           active_move_gripper_goal_;
  }

  // codex.md (goal-rejected diagnosis): always log (RCLCPP_WARN, not gated on
  // enable_tcp_log) the full reject reason + goal contents + current flags, so a
  // rejected /pickplace goal is never silent again.
  rclcpp_action::GoalResponse reject_goal(
    const std::string & reason,
    const std::shared_ptr<const PickPlace::Goal> & goal,
    bool log_enabled)
  {
    RCLCPP_WARN(
      get_logger(),
      "[PickPlaceServer] reject goal reason=%s running=%s execute=%s "
      "velocity_scale=%.4f gripper=%.4f pose_pick=(%.4f, %.4f, %.4f) "
      "pose_place=(%.4f, %.4f, %.4f)",
      reason.c_str(),
      downstream_busy() ? "true" : "false",
      goal->execute ? "true" : "false",
      goal->velocity_scale, goal->gripper,
      goal->pose_pick.position.x, goal->pose_pick.position.y, goal->pose_pick.position.z,
      goal->pose_place.position.x, goal->pose_place.position.y, goal->pose_place.position.z);
    if (logger_ && log_enabled) {
      logger_->log_lifecycle_event(
        "/pickplace", "action_goal_rejected", "handle_goal", "rejected",
        reason, "", action_call_id_);
    }
    return rclcpp_action::GoalResponse::REJECT;
  }

  static bool pose_finite(const geometry_msgs::msg::Pose & p)
  {
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) &&
           std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
           std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickPlace::Goal> goal)
  {
    const bool log_enabled = goal->enable_tcp_log;
    RCLCPP_INFO(
      get_logger(), "[pickplace server] enable_tcp_log=%s", log_enabled ? "true" : "false");

    if (logger_ && log_enabled) {
      action_call_id_ = logger_->log_lifecycle_event(
        "/pickplace", "action_goal_received", "handle_goal", "received", "");
    } else {
      action_call_id_ = 0;
    }

    // Note: /pickplace is ACCEPT_AND_EXECUTE and holds NO server-side busy/running
    // flag that could reject a goal — there is no PICKPLACE_REJECT_BUSY path here.
    if (!std::isfinite(goal->velocity_scale) ||
        goal->velocity_scale <= 0.0 ||
        goal->velocity_scale > 0.2)
    {
      return reject_goal("PICKPLACE_REJECT_INVALID_VELOCITY", goal, log_enabled);
    }

    if (!std::isfinite(goal->gripper) || goal->gripper < 0.0) {
      return reject_goal("PICKPLACE_REJECT_INVALID_GRIPPER", goal, log_enabled);
    }

    if (!pose_finite(goal->pose_pick)) {
      return reject_goal("PICKPLACE_REJECT_INVALID_PICK_POSE", goal, log_enabled);
    }

    if (!pose_finite(goal->pose_place)) {
      return reject_goal("PICKPLACE_REJECT_INVALID_PLACE_POSE", goal, log_enabled);
    }

    RCLCPP_INFO(
      get_logger(),
      "[PickPlaceServer] accept goal execute=%s velocity_scale=%.4f gripper=%.4f",
      goal->execute ? "true" : "false", goal->velocity_scale, goal->gripper);
    if (logger_ && log_enabled) {
      logger_->log_lifecycle_event(
        "/pickplace", "action_goal_accepted", "handle_goal", "accepted", "",
        "", action_call_id_);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PickPlaceGoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "PickPlace cancel requested");

    std::lock_guard<std::mutex> lock(active_goal_mutex_);

    if (active_move_to_pose_goal_) {
      move_to_pose_client_->async_cancel_goal(active_move_to_pose_goal_);
    }

    if (active_move_to_pose_cartesian_goal_) {
      move_to_pose_cartesian_client_->async_cancel_goal(active_move_to_pose_cartesian_goal_);
    }

    if (active_move_gripper_goal_) {
      move_gripper_client_->async_cancel_goal(active_move_gripper_goal_);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<PickPlaceGoalHandle> goal_handle)
  {
    std::thread(&PickPlaceActionServer::execute, this, goal_handle).detach();
  }

  void publish_feedback(
    const std::shared_ptr<PickPlaceGoalHandle> & goal_handle,
    const std::string & stage,
    float progress)
  {
    auto feedback = std::make_shared<PickPlace::Feedback>();
    feedback->stage = stage;
    feedback->progress = progress;

    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      get_logger(),
      "[PickPlace] %s | %.1f%%",
      stage.c_str(),
      progress);
  }

  bool check_cancel(
    const std::shared_ptr<PickPlaceGoalHandle> & goal_handle,
    const std::shared_ptr<PickPlace::Result> & result,
    const std::shared_ptr<PickPlaceTcpLogger::Call> & tcp_call = nullptr)
  {
    if (!goal_handle->is_canceling()) {
      return false;
    }

    result->success = false;
    result->message = "PickPlace canceled";
    if (logger_ && action_call_id_ != 0) {
      logger_->log_lifecycle_event(
        "/pickplace", "action_canceled", "check_cancel", "canceled", result->message,
        "", action_call_id_);
      logger_->log_lifecycle_event(
        "/pickplace", "action_result", "check_cancel", "canceled", result->message,
        "", action_call_id_);
    }
    if (tcp_logger_ && tcp_call) {
      tcp_logger_->finishCall(tcp_call, "canceled", false, result->message);
    }
    goal_handle->canceled(result);

    RCLCPP_WARN(get_logger(), "PickPlace canceled");
    return true;
  }

  void clear_active_goals()
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_move_to_pose_goal_.reset();
    active_move_to_pose_cartesian_goal_.reset();
    active_move_gripper_goal_.reset();
  }

  void abort_goal(
    const std::shared_ptr<PickPlaceGoalHandle> & goal_handle,
    const std::shared_ptr<PickPlace::Result> & result,
    const std::shared_ptr<PickPlaceTcpLogger::Call> & tcp_call,
    const std::string & message)
  {
    clear_active_goals();

    result->success = false;
    result->message = message;

    RCLCPP_ERROR(get_logger(), "PickPlace failed: %s", message.c_str());

    if (logger_ && action_call_id_ != 0) {
      logger_->log_lifecycle_event(
        "/pickplace", "action_stage_failed", "execute", "failed", message, "", action_call_id_);
      logger_->log_lifecycle_event(
        "/pickplace", "action_result", "execute", "aborted", message, "", action_call_id_);
    }
    if (tcp_logger_ && tcp_call) {
      tcp_logger_->finishCall(tcp_call, "aborted", false, message);
    }

    goal_handle->abort(result);
  }

  bool wait_for_sub_action_servers(std::string & error_msg)
  {
    auto timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(server_wait_timeout_s_));

    if (!move_gripper_client_->wait_for_action_server(timeout)) {
      error_msg = "MoveGripper server not available: /move_gripper";
      return false;
    }

    if (!move_to_pose_client_->wait_for_action_server(timeout)) {
      error_msg = "MoveToPose server not available: /move_to_pose";
      return false;
    }

    if (!move_to_pose_cartesian_client_->wait_for_action_server(timeout)) {
      error_msg = "MoveToPoseCartesian server not available: /move_to_pose_cartesian";
      return false;
    }

    return true;
  }

  bool call_move_gripper(
    double position,
    bool execute,
    std::string & error_msg)
  {
    MoveGripper::Goal goal;
    goal.position = position;
    goal.execute = execute;

    auto result_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_result_timeout_s_));

    auto goal_handle_future = move_gripper_client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while sending MoveGripper goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "MoveGripper goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_gripper_goal_ = goal_handle;
    }

    auto result_future = move_gripper_client_->async_get_result(goal_handle);

    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while waiting MoveGripper result";
      return false;
    }

    auto wrapped_result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_gripper_goal_.reset();
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg =
        "MoveGripper failed, result code = " +
        std::to_string(static_cast<int>(wrapped_result.code));
      return false;
    }

    if (!wrapped_result.result) {
      error_msg = "MoveGripper result is null";
      return false;
    }

    if (!wrapped_result.result->success) {
      error_msg = wrapped_result.result->message;
      return false;
    }

    return true;
  }

  bool call_move_to_pose(
    const geometry_msgs::msg::Pose & target_pose,
    double velocity_scale,
    bool execute,
    std::string & error_msg)
  {
    MoveToPose::Goal goal;
    goal.target_pose = target_pose;
    goal.velocity_scale = velocity_scale;
    goal.execute = execute;

    auto result_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_result_timeout_s_));

    auto goal_handle_future = move_to_pose_client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while sending MoveToPose goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "MoveToPose goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_goal_ = goal_handle;
    }

    auto result_future = move_to_pose_client_->async_get_result(goal_handle);

    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while waiting MoveToPose result";
      return false;
    }

    auto wrapped_result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_goal_.reset();
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg =
        "MoveToPose failed, result code = " +
        std::to_string(static_cast<int>(wrapped_result.code));
      return false;
    }

    if (!wrapped_result.result) {
      error_msg = "MoveToPose result is null";
      return false;
    }

    if (!wrapped_result.result->success) {
      error_msg = wrapped_result.result->message;
      return false;
    }

    return true;
  }

  bool call_move_to_pose_cartesian(
    const geometry_msgs::msg::Pose & target_pose,
    double velocity_scale,
    bool execute,
    std::string & error_msg)
  {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = target_pose;
    goal.velocity_scale = velocity_scale;
    goal.execute = execute;

    auto result_timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_result_timeout_s_));

    auto goal_handle_future =
      move_to_pose_cartesian_client_->async_send_goal(goal);

    if (goal_handle_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while sending MoveToPoseCartesian goal";
      return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
      error_msg = "MoveToPoseCartesian goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_cartesian_goal_ = goal_handle;
    }

    auto result_future =
      move_to_pose_cartesian_client_->async_get_result(goal_handle);

    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      error_msg = "Timeout while waiting MoveToPoseCartesian result";
      return false;
    }

    auto wrapped_result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      active_move_to_pose_cartesian_goal_.reset();
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      error_msg =
        "MoveToPoseCartesian failed, result code = " +
        std::to_string(static_cast<int>(wrapped_result.code));
      return false;
    }

    if (!wrapped_result.result) {
      error_msg = "MoveToPoseCartesian result is null";
      return false;
    }

    if (!wrapped_result.result->success) {
      error_msg = wrapped_result.result->message;
      return false;
    }

    return true;
  }

void execute(
  const std::shared_ptr<PickPlaceGoalHandle> goal_handle)
{
  auto result = std::make_shared<PickPlace::Result>();

  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "goal_handle is null");
    return;
  }

  auto goal = goal_handle->get_goal();

  if (!goal) {
    result->success = false;
    result->message = "Goal is null";
    goal_handle->abort(result);
    return;
  }

  const bool log_enabled = goal->enable_tcp_log;
  RCLCPP_INFO(
    get_logger(), "[pickplace server] execute enable_tcp_log=%s",
    log_enabled ? "true" : "false");

  if (logger_ && log_enabled) {
    logger_->log_lifecycle_event(
      "/pickplace", "action_start", "execute", "started", "", "", action_call_id_);
  }

  std::shared_ptr<PickPlaceTcpLogger::Call> call;
  if (tcp_logger_ && log_enabled) {
    call = tcp_logger_->startCall(goal->velocity_scale, goal->pose_pick.position, goal->pose_place.position);
    if (call) {
      tcp_logger_->logEvent(call, "pickplace_start", "pickplace_start", "PickPlace goal accepted");
      tcp_logger_->startSampling(call);
    }
  }

  std::string error_msg;
  const bool execute_motion = goal->execute;

  publish_feedback(
    goal_handle,
    execute_motion ? "Waiting for sub action servers" : "Waiting for sub action servers (plan-only)",
    2.0f);

  if (!wait_for_sub_action_servers(error_msg)) {
    abort_goal(goal_handle, result, call,error_msg);
    return;
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  // codex.md Phase 6: pre_pick uses its own configurable Z offset (default
  // 0.05 m) so the descent above the pick object is 50 mm. The place approach
  // keeps the original approach_height_ (unchanged by codex.md Phase 6).
  geometry_msgs::msg::Pose pick_approach = goal->pose_pick;
  pick_approach.position.z += pre_pick_z_offset_;

  geometry_msgs::msg::Pose place_approach = goal->pose_place;
  place_approach.position.z += approach_height_;

  RCLCPP_INFO(
    get_logger(),
    "Fast PickPlace start | mode=%s | pick=(%.3f %.3f %.3f) | place=(%.3f %.3f %.3f) | gripper=%.4f | vel=%.2f",
    execute_motion ? "execute" : "plan-only",
    goal->pose_pick.position.x,
    goal->pose_pick.position.y,
    goal->pose_pick.position.z,
    goal->pose_place.position.x,
    goal->pose_place.position.y,
    goal->pose_place.position.z,
    goal->gripper,
    goal->velocity_scale);

  // 1. Open gripper
  publish_feedback(goal_handle, execute_motion ? "Open gripper" : "Plan open gripper (execution skipped)", 5.0f);
  if (tcp_logger_ && call) {
    // Stage-only update: keep the set-pose seeded in startCall() (current
    // actual TCP pose at call start) since opening the gripper has no TCP
    // motion target of its own — this keeps error_pos_norm continuous
    // instead of leaving it blank for this stage.
    tcp_logger_->setStage(call, "open_gripper");
    tcp_logger_->logEvent(call, "open_gripper", "stage_start", "opening gripper before pick");
  }

  if (!call_move_gripper(open_gripper_position_, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "open_gripper", "stage_failed", error_msg);
    }
    abort_goal(goal_handle, result, call,"open_gripper: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "open_gripper", "gripper_open", "gripper opened");
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  // 2. Move to pick approach
  publish_feedback(goal_handle, execute_motion ? "Move to pick approach" : "Plan move to pick approach (execution skipped)", 20.0f);
  if (tcp_logger_ && call) {
    tcp_logger_->updateStage(call, "move_to_pre_pick", pick_approach);
    tcp_logger_->logEvent(call, "move_to_pre_pick", "stage_start", "moving to pick approach pose", &pick_approach);
  }

  if (!call_move_to_pose(pick_approach, goal->velocity_scale, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "move_to_pre_pick", "stage_failed", error_msg, &pick_approach);
    }
    abort_goal(goal_handle, result, call,"move_to_pre_pick: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "move_to_pre_pick", "stage_end", "reached pick approach pose", &pick_approach);
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  if (!execute_motion) {
    abort_goal(
      goal_handle,
      result,
      call,
      "cartesian_to_pick: PickPlace plan-only staged composite planning is not fully supported "
      "without executing intermediate segments; refusing to report fake success.");
    return;
  }

  // 3. Cartesian down to pick
  publish_feedback(goal_handle, execute_motion ? "Cartesian down to pick" : "Plan Cartesian down to pick (execution skipped)", 35.0f);
  if (tcp_logger_ && call) {
    tcp_logger_->updateStage(call, "approach_pick", goal->pose_pick);
    tcp_logger_->logEvent(call, "approach_pick", "stage_start", "cartesian descent to pick pose", &goal->pose_pick);
  }

  if (!call_move_to_pose_cartesian(goal->pose_pick, goal->velocity_scale, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "approach_pick", "stage_failed", error_msg, &goal->pose_pick);
    }
    abort_goal(goal_handle, result, call,"cartesian_to_pick: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "approach_pick", "stage_end", "reached pick pose", &goal->pose_pick);
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  // 4. Close gripper
  publish_feedback(goal_handle, execute_motion ? "Close gripper" : "Plan close gripper (execution skipped)", 50.0f);
  if (tcp_logger_ && call) {
    tcp_logger_->updateStage(call, "close_gripper", goal->pose_pick);
    tcp_logger_->logEvent(call, "close_gripper", "stage_start", "closing gripper on object", &goal->pose_pick);
  }

  if (!call_move_gripper(goal->gripper, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "close_gripper", "stage_failed", error_msg, &goal->pose_pick);
    }
    abort_goal(goal_handle, result, call,"close_gripper: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "close_gripper", "gripper_close", "gripper closed", &goal->pose_pick);
  }

  // Đợi gripper đóng xong / vật ổn định rồi mới nâng
  publish_feedback(goal_handle, execute_motion ? "Wait gripper close settle" : "Skip gripper settle wait (plan-only)", 52.0f);

  if (execute_motion) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  // 5. Move directly to place approach
  //
  // Bước này thay cho:
  // - Cartesian lift from pick (retreat_pick)
  // - MoveToPose to place approach
  //
  // MoveIt sẽ tự plan từ pose_pick lên place_approach.
  publish_feedback(goal_handle, execute_motion ? "Move directly to place approach" : "Plan move directly to place approach (execution skipped)", 70.0f);
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "close_gripper", "stage_end", "gripper settle wait done", &goal->pose_pick);
    tcp_logger_->updateStage(call, "move_to_pre_place", place_approach);
    tcp_logger_->logEvent(call, "move_to_pre_place", "stage_start",
      "retreat from pick and move to place approach pose", &place_approach);
  }

  if (!call_move_to_pose(place_approach, goal->velocity_scale, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "move_to_pre_place", "stage_failed", error_msg, &place_approach);
    }
    abort_goal(goal_handle, result, call,"move_to_pre_place: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "move_to_pre_place", "stage_end", "reached place approach pose", &place_approach);
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  // 6. Cartesian down to place
  publish_feedback(goal_handle, execute_motion ? "Cartesian down to place" : "Plan Cartesian down to place (execution skipped)", 85.0f);
  if (tcp_logger_ && call) {
    tcp_logger_->updateStage(call, "approach_place", goal->pose_place);
    tcp_logger_->logEvent(call, "approach_place", "stage_start", "cartesian descent to place pose", &goal->pose_place);
  }

  if (!call_move_to_pose_cartesian(goal->pose_place, goal->velocity_scale, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "approach_place", "stage_failed", error_msg, &goal->pose_place);
    }
    abort_goal(goal_handle, result, call,"cartesian_to_place: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "approach_place", "stage_end", "reached place pose", &goal->pose_place);
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  // 7. Open gripper to release
  publish_feedback(goal_handle, execute_motion ? "Open gripper to release" : "Plan open gripper to release (execution skipped)", 95.0f);
  if (tcp_logger_ && call) {
    tcp_logger_->updateStage(call, "open_gripper_release", goal->pose_place);
    tcp_logger_->logEvent(call, "open_gripper_release", "stage_start", "releasing object", &goal->pose_place);
  }

  if (!call_move_gripper(open_gripper_position_, execute_motion, error_msg)) {
    if (tcp_logger_ && call) {
      tcp_logger_->logEvent(call, "open_gripper_release", "stage_failed", error_msg, &goal->pose_place);
    }
    abort_goal(goal_handle, result, call,"release_gripper: " + error_msg);
    return;
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "open_gripper_release", "gripper_open", "gripper opened, object released", &goal->pose_place);
  }

  if (check_cancel(goal_handle, result, call)) {
    return;
  }

  publish_feedback(
    goal_handle,
    execute_motion ? "Fast PickPlace completed" : "Fast PickPlace planning completed (execution skipped)",
    100.0f);

  result->success = true;
  result->message = execute_motion ?
    "Fast PickPlace completed successfully" :
    "Fast PickPlace planning success; execution skipped";

  clear_active_goals();

  if (logger_ && action_call_id_ != 0) {
    logger_->log_lifecycle_event(
      "/pickplace", "action_succeeded", "execute", "succeeded", result->message,
      "", action_call_id_);
    logger_->log_lifecycle_event(
      "/pickplace", "action_result", "execute", "succeeded", result->message,
      "", action_call_id_);
  }
  if (tcp_logger_ && call) {
    tcp_logger_->logEvent(call, "pickplace_end", "pickplace_end", result->message);
    tcp_logger_->finishCall(call, "completed", true, result->message);
  }

  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(), "Fast PickPlace completed successfully");
}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceActionServer>();
  node->initialize_logging();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
