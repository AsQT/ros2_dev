// Plugin
#include "robot_hardware_interface/tcp_system_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

/* Các hàm 
parse_csv_ints
parse_csv_doubles
on_ionnit
on_configure
on_activate
on_cleanup
export_state_interfaces
export_command_interfaces
read
write
*/


namespace robot_hardware_interface
{

namespace
{
constexpr const char * ROBOT_CONNECTED_TOPIC = "/robot_hw/connected";
constexpr const char * LEGACY_CONNECTED_TOPIC = "/hardware/connected";
constexpr const char * ROBOT_STATUS_TEXT_TOPIC = "/robot_hw/status_text";
constexpr const char * ROBOT_FLAGS_TOPIC = "/robot_hw/flags";
constexpr const char * LEGACY_FLAGS_TOPIC = "/hardware/flags";

constexpr uint32_t STATUS_ERROR_ALL = 0x00000001;
constexpr uint32_t STATUS_SOF_LIMIT_P = 0x00000008;
constexpr uint32_t STATUS_SOF_LIMIT_M = 0x00000010;
constexpr uint32_t STATUS_EMG = 0x00010000;
constexpr uint32_t STATUS_S_STOP = 0x00020000;
constexpr uint32_t STATUS_ORGINRETURNING = 0x00040000;
constexpr uint32_t STATUS_SERVO_ON = 0x00100000;
constexpr uint32_t STATUS_ALARM_RST = 0x00200000;
constexpr uint32_t STATUS_ORG_SENSOR = 0x00800000;
constexpr uint32_t STATUS_ORG_SET_OK = 0x02000000;
constexpr uint32_t STATUS_MOTIONING = 0x08000000;
}  // namespace

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad2deg(double rad) { return rad * 180.0 /M_PI ; }
static inline double deg2met(double deg) { return deg /2; }
static inline double met2deg(double rad) { return rad *2; }
/*__________________________________________________________________________________*/
std::vector<int> RobotSystemHardware::parse_csv_ints(const std::string & s)
{
  std::vector<int> out;
  std::stringstream ss(s);
  std::string tok;

  while (std::getline(ss, tok, ',')) {
    tok.erase(tok.begin(), std::find_if(tok.begin(), tok.end(),
      [](unsigned char c){ return !std::isspace(c); }));
    tok.erase(std::find_if(tok.rbegin(), tok.rend(),
      [](unsigned char c){ return !std::isspace(c); }).base(), tok.end());
    if (tok.empty()) continue;
    try { out.push_back(std::stoi(tok)); } catch (...) {}
  }
  return out;
}
/*__________________________________________________________________________________*/
std::vector<double> RobotSystemHardware::parse_csv_doubles(const std::string & s)
{
  std::vector<double> out;
  std::stringstream ss(s);
  std::string tok;

  while (std::getline(ss, tok, ',')) {
    tok.erase(tok.begin(), std::find_if(tok.begin(), tok.end(),
      [](unsigned char c){ return !std::isspace(c); }));
    tok.erase(std::find_if(tok.rbegin(), tok.rend(),
      [](unsigned char c){ return !std::isspace(c); }).base(), tok.end());
    if (tok.empty()) continue;
    try { out.push_back(std::stod(tok)); } catch (...) {}
  }
  return out;
}

void RobotSystemHardware::setup_ros_api()
{
  if (node_) {
    return;
  }

  node_ = get_node();
  if (!node_) {
    RCLCPP_WARN(get_logger(), "Hardware component node is not available; /robot_hw services disabled");
    return;
  }

  pub_connected_ = node_->create_publisher<std_msgs::msg::Bool>(ROBOT_CONNECTED_TOPIC, 10);
  pub_connected_legacy_ = node_->create_publisher<std_msgs::msg::Bool>(LEGACY_CONNECTED_TOPIC, 10);
  pub_status_text_ = node_->create_publisher<std_msgs::msg::String>(ROBOT_STATUS_TEXT_TOPIC, 10);
  pub_status_flags_ =
    node_->create_publisher<robot_hardware_interface::msg::FlagStatus>(ROBOT_FLAGS_TOPIC, 10);
  pub_status_flags_legacy_ =
    node_->create_publisher<robot_hardware_interface::msg::FlagStatus>(LEGACY_FLAGS_TOPIC, 10);

  sub_servo_axis_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/robot_hw/cmd_servo_axis", 10,
    [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
      on_servo_axis_topic(msg);
    });

  sub_jog_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/robot_hw/cmd_jog", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      on_jog_topic(msg);
    });

  sub_run_axis_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/robot_hw/cmd_run_axis", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      on_run_axis_topic(msg);
    });

  sub_run_all_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/robot_hw/cmd_run_all", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      on_run_all_topic(msg);
    });

  sub_joint_trajectory_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/robot_hw/joint_trajectory", 10,
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
      on_joint_trajectory_topic(msg);
    });

  srv_poll_now_ = node_->create_service<std_srvs::srv::Trigger>(
    "/robot_hw/poll_now",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      on_poll_now(res);
    });

  srv_servo_all_ = node_->create_service<std_srvs::srv::SetBool>(
    "/robot_hw/servo_all",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
      on_servo_all(req, res);
    });

  srv_servo_on_all_ = node_->create_service<robot_hardware_interface::srv::ServoOnAll>(
    "/robot_hw/servo_on_all",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Response> res)
    {
      on_servo_on_all(req, res);
    });

  srv_servo_on_axis_ = node_->create_service<robot_hardware_interface::srv::ServoOnAxis>(
    "/robot_hw/servo_on_axis",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Response> res)
    {
      on_servo_on_axis(req, res);
    });

  srv_jog_ = node_->create_service<robot_hardware_interface::srv::Jog>(
    "/robot_hw/jog",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::Jog::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::Jog::Response> res)
    {
      on_jog(req, res);
    });

  srv_home_ = node_->create_service<robot_hardware_interface::srv::Home>(
    "/robot_hw/home",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::Home::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::Home::Response> res)
    {
      on_home(req, res);
    });

  srv_run_axis_ = node_->create_service<robot_hardware_interface::srv::RunAxis>(
    "/robot_hw/run_axis",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::RunAxis::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::RunAxis::Response> res)
    {
      on_run_axis(req, res);
    });

  srv_stop_axis_ = node_->create_service<robot_hardware_interface::srv::StopAxis>(
    "/robot_hw/stop_axis",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::StopAxis::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::StopAxis::Response> res)
    {
      on_stop_axis(req, res);
    });

  srv_stop_all_ = node_->create_service<robot_hardware_interface::srv::StopAll>(
    "/robot_hw/stop_all",
    [this](
      const std::shared_ptr<robot_hardware_interface::srv::StopAll::Request> req,
      std::shared_ptr<robot_hardware_interface::srv::StopAll::Response> res)
    {
      on_stop_all(req, res);
    });

  timer_health_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      publish_connected(connected_ && client_.is_connected());
      if (!status_text_.empty()) {
        publish_status_text(status_text_);
      }
    });

  publish_connected(false);
  publish_status_text("Robot TCP hardware plugin ready");
  publish_status_flags(std::vector<uint32_t>(6, 0));
  RCLCPP_INFO(get_logger(), "Robot TCP plugin ROS API ready on /robot_hw/* services");
}

void RobotSystemHardware::publish_connected(bool connected)
{
  if (!pub_connected_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = connected;
  pub_connected_->publish(msg);
  if (pub_connected_legacy_) {
    pub_connected_legacy_->publish(msg);
  }
}

void RobotSystemHardware::publish_status_text(const std::string & text)
{
  status_text_ = text;
  if (!pub_status_text_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  pub_status_text_->publish(msg);
}

void RobotSystemHardware::publish_status_flags(const std::vector<uint32_t> & flags)
{
  if (!pub_status_flags_) {
    return;
  }

  robot_hardware_interface::msg::FlagStatus msg;
  const size_t expected_axes = msg.axes.size();
  if (flags.size() < expected_axes) {
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clk, 5000,
        "/robot_hw/flags got %zu flags, expected %zu; missing axes publish status_f=0",
        flags.size(), expected_axes);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "/robot_hw/flags got %zu flags, expected %zu; missing axes publish status_f=0",
        flags.size(), expected_axes);
    }
  }

  for (size_t axis = 0; axis < expected_axes; ++axis) {
    const uint32_t st = axis < flags.size() ? flags[axis] : 0u;
    auto & a = msg.axes[axis];
    a.servo_on      = (st & STATUS_SERVO_ON) != 0;
    a.error_all     = (st & STATUS_ERROR_ALL) != 0;
    a.org_ok        = (st & STATUS_ORG_SET_OK) != 0;
    a.motionning    = (st & STATUS_MOTIONING) != 0;
    a.org_retunning = (st & STATUS_ORGINRETURNING) != 0;
    a.limit_pos     = (st & STATUS_SOF_LIMIT_P) != 0;
    a.limit_neg     = (st & STATUS_SOF_LIMIT_M) != 0;
    a.org_sensor    = (st & STATUS_ORG_SENSOR) != 0;
    a.alarm_rst     = (st & STATUS_ALARM_RST) != 0;
    a.emg           = (st & STATUS_EMG) != 0;
    a.stop          = (st & STATUS_S_STOP) != 0;
    a.communi_err   = false;
    a.status_f      = st;
  }

  pub_status_flags_->publish(msg);
  if (pub_status_flags_legacy_) {
    pub_status_flags_legacy_->publish(msg);
  }
}

void RobotSystemHardware::update_state_from_raw(
  const std::vector<double> & pos_raw,
  const std::vector<double> & vel_raw,
  const std::vector<uint32_t> & flags)
{
  const size_t n = hw_pos_.size();
  const size_t avail = std::min(pos_raw.size(), vel_raw.size());

  for (size_t i = 0; i < n; ++i) {
    const int axis_id = axis_ids_[i];
    if (axis_id < 0 ||
      static_cast<size_t>(axis_id) >= pos_raw.size() ||
      static_cast<size_t>(axis_id) >= vel_raw.size())
    {
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *clk, 2000,
          "read: axis_id=%d for joint[%zu] outside state arrays pos=%zu vel=%zu",
          axis_id, i, pos_raw.size(), vel_raw.size());
      }
      continue;
    }

    const size_t axis_index = static_cast<size_t>(axis_id);
    const double pos_deg   = pos_raw[axis_index];
    const double vel_deg_s = vel_raw[axis_index];
    if (i < 6) {
      const double rad_driver   = deg2rad(pos_deg);
      const double rad_s_driver = deg2rad(vel_deg_s);
      const double rad_ros = rad_driver * direction_sign_[i] + rad_offset_[i];

      if (std::fabs(rad_ros) > 10.0) {
        if (auto clk = get_clock()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *clk, 2000,
            "read: unreasonable joint[%zu]=%f rad, keeping last value", i, rad_ros);
        }
        continue;
      }
      hw_pos_[i] = rad_ros;
      hw_vel_[i] = rad_s_driver * direction_sign_[i];
    } else {
      hw_pos_[i] = deg2met(pos_deg);
      hw_vel_[i] = deg2met(vel_deg_s);
    }
  }

  (void)avail;
  publish_connected(true);
  publish_status_flags(flags);
}

void RobotSystemHardware::on_poll_now(
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const auto [pos_raw, vel_raw, flags] = client_.get_all_state(read_timeout_ms_);
    update_state_from_raw(pos_raw, vel_raw, flags);
    consec_read_fail_ = 0;
    res->success = true;
    res->message = "OK";
    publish_status_text("Poll now OK");
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
    publish_status_text(std::string("Poll now failed: ") + e.what());
  }
}

void RobotSystemHardware::on_servo_all(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    client_.servo_all(req->data);
    res->success = true;
    res->message = req->data ? "Servo ALL ON" : "Servo ALL OFF";
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
    publish_status_text(std::string("Servo ALL failed: ") + e.what());
  }
}

void RobotSystemHardware::on_servo_on_all(
  const std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Request> req,
  std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const bool on = req->state != 0;
    client_.servo_all(on);
    res->ok = true;
    res->error_code = 0;
    res->message = on ? "Servo ALL ON" : "Servo ALL OFF";
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Servo ALL failed: ") + e.what());
  }
}

void RobotSystemHardware::on_servo_on_axis(
  const std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Request> req,
  std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const uint8_t id = req->id;
    const bool on = req->state != 0;
    if (on) {
      client_.servo_on_axis(id);
    } else {
      client_.servo_off_axis(id);
    }
    res->ok = true;
    res->error_code = 0;
    res->message = on ? "Servo ON" : "Servo OFF";
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Servo axis failed: ") + e.what());
  }
}

void RobotSystemHardware::on_jog(
  const std::shared_ptr<robot_hardware_interface::srv::Jog::Request> req,
  std::shared_ptr<robot_hardware_interface::srv::Jog::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const auto axis_it = std::find(axis_ids_.begin(), axis_ids_.end(), static_cast<int>(req->id));
    if (axis_it == axis_ids_.end()) {
      throw std::runtime_error("Jog axis id is not configured");
    }

    const bool dir_plus = req->dir != 0;
    const double vel_deg_s = std::fabs(rad2deg(req->vel));
    client_.jog(req->id, dir_plus, vel_deg_s);

    res->ok = true;
    res->error_code = 0;
    if (vel_deg_s <= 0.0) {
      res->message = "Jog STOP axis " + std::to_string(req->id);
    } else {
      res->message = std::string("Jog axis ") + std::to_string(req->id) + (dir_plus ? " +" : " -");
    }
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Jog failed: ") + e.what());
  }
}

void RobotSystemHardware::on_home(
  const std::shared_ptr<robot_hardware_interface::srv::Home::Request> req,
  std::shared_ptr<robot_hardware_interface::srv::Home::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const auto axis_it = std::find(axis_ids_.begin(), axis_ids_.end(), static_cast<int>(req->id));
    if (axis_it == axis_ids_.end()) {
      throw std::runtime_error("axis id not found in axis_ids param");
    }

    client_.servo_home_axis(req->id);
    res->ok = true;
    res->error_code = 0;
    res->message = "HOME (SERVO_HOME_AX)";
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Home failed: ") + e.what());
  }
}

void RobotSystemHardware::on_run_axis(
  const std::shared_ptr<robot_hardware_interface::srv::RunAxis::Request> req,
  std::shared_ptr<robot_hardware_interface::srv::RunAxis::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const auto axis_it = std::find(axis_ids_.begin(), axis_ids_.end(), static_cast<int>(req->id));
    if (axis_it == axis_ids_.end()) {
      throw std::runtime_error("axis id not found in axis_ids param");
    }

    const double pos_deg = rad2deg(req->pos);
    const double vel_deg_s = std::fabs(rad2deg(req->vel));
    client_.run_axis(req->id, pos_deg, vel_deg_s);
    res->ok = true;
    res->error_code = 0;
    res->message = "RUN axis";
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Run axis failed: ") + e.what());
  }
}

void RobotSystemHardware::on_stop_axis(
  const std::shared_ptr<robot_hardware_interface::srv::StopAxis::Request> req,
  std::shared_ptr<robot_hardware_interface::srv::StopAxis::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }
    const auto axis_it = std::find(axis_ids_.begin(), axis_ids_.end(), static_cast<int>(req->id));
    if (axis_it == axis_ids_.end()) {
      throw std::runtime_error("StopAxis id is not configured");
    }

    client_.jog(req->id, true, 0.0);
    res->ok = true;
    res->error_code = 0;
    res->message = "STOP axis " + std::to_string(req->id);
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Stop axis failed: ") + e.what());
  }
}

void RobotSystemHardware::on_stop_all(
  const std::shared_ptr<robot_hardware_interface::srv::StopAll::Request>,
  std::shared_ptr<robot_hardware_interface::srv::StopAll::Response> res)
{
  try {
    if (!connected_ || !client_.is_connected()) {
      throw std::runtime_error("Not connected");
    }

    for (const int axis_id : axis_ids_) {
      if (axis_id < 0 || axis_id > 255) {
        continue;
      }
      client_.jog(static_cast<uint8_t>(axis_id), true, 0.0);
    }
    res->ok = true;
    res->error_code = 0;
    res->message = "STOP all configured axes";
    publish_status_text(res->message);
  } catch (const std::exception & e) {
    res->ok = false;
    res->error_code = connected_ ? 3 : 1;
    res->message = e.what();
    publish_status_text(std::string("Stop failed: ") + e.what());
  }
}

void RobotSystemHardware::on_servo_axis_topic(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (!connected_ || !client_.is_connected() || msg->data.size() < 2) {
    return;
  }
  try {
    const uint8_t id = msg->data[0];
    const bool on = msg->data[1] != 0;
    if (on) {
      client_.servo_on_axis(id);
    } else {
      client_.servo_off_axis(id);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "servo_axis topic: %s", e.what());
  }
}

void RobotSystemHardware::on_jog_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!connected_ || !client_.is_connected() || msg->data.size() < 3) {
    return;
  }
  try {
    const uint8_t id = static_cast<uint8_t>(std::llround(msg->data[0]));
    const bool dir = std::llround(msg->data[1]) != 0;
    const double vel_deg_s = msg->data[2];
    client_.jog(id, dir, vel_deg_s);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "jog topic: %s", e.what());
  }
}

void RobotSystemHardware::on_run_axis_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!connected_ || !client_.is_connected() || msg->data.size() < 3) {
    return;
  }
  try {
    const uint8_t id = static_cast<uint8_t>(std::llround(msg->data[0]));
    const double pos_deg = msg->data[1];
    const double vel_deg_s = msg->data[2];
    client_.run_axis(id, pos_deg, vel_deg_s);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "run_axis topic: %s", e.what());
  }
}

void RobotSystemHardware::on_run_all_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!connected_ || !client_.is_connected()) {
    return;
  }

  const size_t n = axis_ids_.size();
  if (n == 0 || msg->data.size() < 2 * n) {
    return;
  }

  try {
    if (n == 8) {
      std::vector<double> pos(8), vel(8);
      for (size_t i = 0; i < 8; ++i) pos[i] = msg->data[i];
      for (size_t i = 0; i < 8; ++i) vel[i] = msg->data[8 + i];
      client_.run_all(pos, vel);
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      const uint8_t axis = static_cast<uint8_t>(axis_ids_[i]);
      const double pos_deg = msg->data[i];
      const double vel_deg_s = msg->data[n + i];
      client_.run_axis(axis, pos_deg, vel_deg_s);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "run_all topic: %s", e.what());
  }
}

void RobotSystemHardware::on_joint_trajectory_topic(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!connected_ || !client_.is_connected() || msg->points.empty()) {
    return;
  }

  const auto & point = msg->points.back();
  const size_t n = axis_ids_.size();
  if (n == 0 || point.positions.size() < n) {
    return;
  }

  try {
    if (n == 8) {
      std::vector<double> pos_deg(8), vel_deg_s(8, default_vel_deg_s_);
      for (size_t i = 0; i < 8; ++i) pos_deg[i] = rad2deg(point.positions[i]);
      if (point.velocities.size() >= 8) {
        for (size_t i = 0; i < 8; ++i) vel_deg_s[i] = std::fabs(rad2deg(point.velocities[i]));
      }
      client_.run_all(pos_deg, vel_deg_s);
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      const uint8_t axis = static_cast<uint8_t>(axis_ids_[i]);
      const double pos_deg = rad2deg(point.positions[i]);
      double vel_deg_s = default_vel_deg_s_;
      if (point.velocities.size() >= n) {
        vel_deg_s = std::fabs(rad2deg(point.velocities[i]));
      }
      client_.run_axis(axis, pos_deg, vel_deg_s);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "joint_trajectory topic: %s", e.what());
  }
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;

  auto get_s = [&](const std::string & key, const std::string & def)->std::string {
    auto it = hp.find(key);
    return (it == hp.end()) ? def : it->second;
  };
  auto get_i = [&](const std::string & key, int def)->int {
    auto it = hp.find(key);
    if (it == hp.end()) return def;
    try { return std::stoi(it->second); } catch (...) { return def; }
  };
  auto get_d = [&](const std::string & key, double def)->double {
    auto it = hp.find(key);
    if (it == hp.end()) return def;
    try { return std::stod(it->second); } catch (...) { return def; }
  };

  robot_ip_           = get_s("robot_ip", robot_ip_);
  robot_port_         = get_i("robot_port", robot_port_);
  connect_timeout_ms_ = get_i("connect_timeout_ms", connect_timeout_ms_);
  read_timeout_ms_    = get_i("read_timeout_ms", read_timeout_ms_);
  if (connect_timeout_ms_ < 0) {
    RCLCPP_WARN(get_logger(), "connect_timeout_ms < 0, using default 2000 ms");
    connect_timeout_ms_ = 2000;
  }
  if (read_timeout_ms_ < 0) {
    RCLCPP_WARN(get_logger(), "read_timeout_ms < 0, using default 50 ms");
    read_timeout_ms_ = 50;
  }
  all_token_          = get_i("all_token", all_token_);
  default_vel_deg_s_  = get_d("default_vel_deg_s", default_vel_deg_s_);

  RCLCPP_INFO(
    get_logger(),
    "Robot TCP target: %s:%d connect_timeout=%d ms read_timeout=%d ms, magic=0x%04X, header=%zu bytes",
    robot_ip_.c_str(), robot_port_, connect_timeout_ms_, read_timeout_ms_,
    ROBOT_TCP_MAGIC, ROBOT_TCP_HEADER_SIZE);

  setup_ros_api();

  const size_t n = info_.joints.size();
  if (n == 0) {
    RCLCPP_ERROR(get_logger(), "No joints configured in URDF ros2_control tag.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  axis_ids_ = parse_csv_ints(get_s("axis_ids", ""));
  if (axis_ids_.size() != n) {
    axis_ids_.clear();
    axis_ids_.reserve(n);
    for (size_t i = 0; i < n; ++i) axis_ids_.push_back(static_cast<int>(i));
  }

  direction_sign_ = parse_csv_doubles(get_s("direction_sign", ""));
  if (direction_sign_.size() != n) direction_sign_.assign(n, 1.0);

  rad_offset_ = parse_csv_doubles(get_s("rad_offset", ""));
  if (rad_offset_.size() != n) rad_offset_.assign(n, 0.0);

  hw_pos_.assign(n, 0.0);
  hw_vel_.assign(n, 0.0);

  cmd_pos_.assign(n, 0.0);
  cmd_vel_.assign(n, 0.0);

  last_sent_pos_.assign(n, 0.0);

  for (const auto & j : info_.joints) {
    bool has_pos_cmd = false;
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
    }
    if (!has_pos_cmd) {
      RCLCPP_ERROR(get_logger(),
        "Joint '%s' missing command_interface 'position'. This driver expects position control.",
        j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  connected_ = false;
  publish_connected(false);
  RCLCPP_INFO(
    get_logger(),
    "Robot TCP configured for %s:%d. Port will open on activate.",
    robot_ip_.c_str(), robot_port_);
  publish_status_text("Robot TCP configured. Port will open on activate.");
  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!connected_ || !client_.is_connected()) {
    try {
      RCLCPP_INFO(
        get_logger(),
        "Robot TCP connecting: %s:%d (connect_timeout=%d ms)",
        robot_ip_.c_str(), robot_port_, connect_timeout_ms_);
      connected_ = client_.connect(robot_ip_, robot_port_, connect_timeout_ms_);
      if (!connected_) {
        RCLCPP_WARN(
          get_logger(),
          "Robot TCP connect failed: %s:%d: %s. Activating offline with zero joint state.",
          robot_ip_.c_str(), robot_port_, client_.last_error().c_str());
        publish_connected(false);
        publish_status_text(std::string("Robot TCP connect failed; running offline: ") + client_.last_error());
        publish_status_flags(std::vector<uint32_t>(6, 0));
      }
      if (connected_) {
        RCLCPP_INFO(get_logger(), "Robot TCP connect ok: %s:%d", robot_ip_.c_str(), robot_port_);
        publish_connected(true);
        publish_status_text("Robot TCP connected");
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Robot TCP connect failed: %s:%d: %s. Activating offline with zero joint state.",
        robot_ip_.c_str(), robot_port_, e.what());
      connected_ = false;
      publish_connected(false);
      publish_status_text(std::string("Robot TCP connect failed; running offline: ") + e.what());
      publish_status_flags(std::vector<uint32_t>(6, 0));
    }
  }


  if (connected_ && client_.is_connected()) {
    auto ret = read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    if (ret == hardware_interface::return_type::OK) {
      cmd_pos_ = hw_pos_;           // tránh giật về 0
      last_sent_pos_ = cmd_pos_;    // coi như đã "gửi" trạng thái hiện tại
      state_synced_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "Initial read() failed. Holding write until a successful read.");
      state_synced_ = false;        // sẽ chờ read() thành công sau
    }
  } else {
    hw_pos_.assign(hw_pos_.size(), 0.0);
    hw_vel_.assign(hw_vel_.size(), 0.0);
    cmd_pos_ = hw_pos_;
    last_sent_pos_ = cmd_pos_;
    state_synced_ = true;
    consec_read_fail_ = 0;
    RCLCPP_WARN(get_logger(), "Robot TCP offline mode active. Publishing zero joint state for RViz/model.");
  }

  warmup_cycles_ = warmup_cycles_cfg_;
  last_write_time_ = get_clock()->now();

  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    if (connected_ && client_.is_connected()) {
      client_.servo_all(false);
      RCLCPP_INFO(get_logger(), "Robot TCP servo all off before deactivate");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "TCP servo_all(off) failed: %s", e.what());
  }

  try {
    client_.disconnect();
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "TCP disconnect failed during deactivate: %s", e.what());
  } catch (...) {
    RCLCPP_WARN(get_logger(), "TCP disconnect failed during deactivate");
  }
  connected_ = false;
  publish_connected(false);
  publish_status_text("Robot TCP disconnected on deactivate");
  RCLCPP_INFO(get_logger(), "Robot TCP disconnected on deactivate");
  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  try { client_.disconnect(); } catch (...) {}
  connected_ = false;
  publish_connected(false);
  publish_status_text("Robot TCP disconnected");
  RCLCPP_INFO(get_logger(), "Robot TCP disconnected");
  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
std::vector<hardware_interface::StateInterface> RobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & j = info_.joints[i];
    for (const auto & si : j.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) 
      {
        out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
      }
      else if 
      (si.name == hardware_interface::HW_IF_VELOCITY) 
      {
        out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
      } 
    }
  }
  return out;
}
/*__________________________________________________________________________________*/
std::vector<hardware_interface::CommandInterface> RobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & j = info_.joints[i];
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) 
      {
        out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
      } 
      else if (ci.name == hardware_interface::HW_IF_VELOCITY) 
      {
        out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
      } 
    }
  }
  return out;
}
/*__________________________________________________________________________________*/
hardware_interface::return_type RobotSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!connected_ || !client_.is_connected()) {
    connected_ = false;
    publish_connected(false);
    publish_status_flags(std::vector<uint32_t>(6, 0));
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clk, 2000,
        "Robot TCP unavailable: %s:%d (publishing default/last state)",
        robot_ip_.c_str(), robot_port_);
    }
    consec_read_fail_ = 0;
    return hardware_interface::return_type::OK;
  }

  const size_t n = hw_pos_.size();

  try {
    const auto [pos_raw, vel_raw, flag] = client_.get_all_state(read_timeout_ms_);
    publish_connected(true);
    publish_status_flags(flag);

    if (!first_state_frame_logged_) {
      first_state_frame_logged_ = true;
      std::ostringstream raw_state;
      for (size_t i = 0; i < std::min<size_t>(ROBOT_STATUS_FRAME_AXIS_COUNT, flag.size()); ++i) {
        if (i > 0) {
          raw_state << "; ";
        }
        raw_state << "axis[" << i << "]: pos=" << static_cast<int>(std::llround(pos_raw[i] * 1000.0))
                  << " vel=" << static_cast<unsigned>(std::llround(std::fabs(vel_raw[i]) * 1000.0))
                  << " flag=0x" << std::hex << std::uppercase << std::setw(8)
                  << std::setfill('0') << flag[i] << std::dec << std::setfill(' ');
      }
      RCLCPP_INFO(
        get_logger(),
        "CMD_GET_ALL OK: payload_len=%zu status=0x%02X axis_offset=%zu axis_count=%zu axis_bytes=%zu %s",
        client_.last_state_payload_length(),
        static_cast<unsigned>(ROBOT_CMD_OK),
        client_.last_state_payload_offset(),
        flag.size(),
        client_.last_state_axis_bytes(),
        raw_state.str().c_str());
    }

    if (pos_raw.empty() || vel_raw.empty()) {
      publish_status_flags(std::vector<uint32_t>(6, 0));
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *clk, 2000,
          "read get_all_state returned empty state: pos=%zu vel=%zu (keeping last state)",
          pos_raw.size(), vel_raw.size());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "read get_all_state returned empty state: pos=%zu vel=%zu (keeping last state)",
          pos_raw.size(), vel_raw.size());
      }

      consec_read_fail_++;
      return hardware_interface::return_type::OK;
    }
    // OK -> reset fail counter
    consec_read_fail_ = 0;

    for (size_t i = 0; i < n; ++i) {
      const int axis_id = axis_ids_[i];
      if (axis_id < 0 ||
        static_cast<size_t>(axis_id) >= pos_raw.size() ||
        static_cast<size_t>(axis_id) >= vel_raw.size())
      {
        if (auto clk = get_clock()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *clk, 2000,
            "read: axis_id=%d for joint[%zu] outside state arrays pos=%zu vel=%zu",
            axis_id, i, pos_raw.size(), vel_raw.size());
        }
        continue;
      }

      const size_t axis_index = static_cast<size_t>(axis_id);
      const double pos_deg   = pos_raw[axis_index];
      const double vel_deg_s = vel_raw[axis_index];
      if (i<6)
      {
        const double rad_driver   = deg2rad(pos_deg);
        const double rad_s_driver = deg2rad(vel_deg_s);
        const double rad_ros = rad_driver * direction_sign_[i] + rad_offset_[i];

        if (std::fabs(rad_ros) > 10.0) {
          if (auto clk = get_clock()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *clk, 2000,
              "read: unreasonable joint[%zu]=%f rad, keeping last value", i, rad_ros);
          }
          continue; 
        }
        hw_pos_[i] = rad_ros;
        hw_vel_[i] = rad_s_driver * direction_sign_[i];
      } else
      {
        hw_pos_[i] = deg2met(pos_deg);
        hw_vel_[i] = deg2met(vel_deg_s);
      }
    }

    return hardware_interface::return_type::OK;

  } catch (const std::exception & e) {
    if (!client_.is_connected()) {
      connected_ = false;
      publish_connected(false);
    }
    publish_status_flags(std::vector<uint32_t>(6, 0));
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clk, 2000,
        "get_all_state failed: %s:%d: %s (keeping last state, publishing default flags)",
        robot_ip_.c_str(), robot_port_, e.what());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "get_all_state failed: %s:%d: %s (keeping last state, publishing default flags)",
        robot_ip_.c_str(), robot_port_, e.what());
    }

    consec_read_fail_++;
    return hardware_interface::return_type::OK;
  }
}
 
hardware_interface::return_type RobotSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!connected_ || !client_.is_connected()) {
    connected_ = false;
    publish_connected(false);
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clk, 2000,
        "Robot TCP write skipped: not connected to %s:%d",
        robot_ip_.c_str(), robot_port_);
    }
    return hardware_interface::return_type::OK;
  }

  constexpr size_t AXES = 8;
  if (!state_synced_) {
    return hardware_interface::return_type::OK;
  }

  if (warmup_cycles_ > 0) {
    --warmup_cycles_;
    return hardware_interface::return_type::OK;
  }

  if (auto clk = get_clock()) {
    const auto now = clk->now();
    if ((now - last_write_time_).seconds() < write_period_s_) {
      return hardware_interface::return_type::OK;
    }
    last_write_time_ = now;
  }

  if (cmd_pos_.size() != AXES ||
      rad_offset_.size() != AXES ||
      direction_sign_.size() != AXES) {
    RCLCPP_ERROR(
      get_logger(),
      "write: size mismatch. cmd_pos=%zu rad_offset=%zu dir_sign=%zu (need %zu)",
      cmd_pos_.size(), rad_offset_.size(), direction_sign_.size(), AXES);
    return hardware_interface::return_type::ERROR;
  }

  if (last_sent_pos_.size() == AXES) {
    bool changed = false;
    for (size_t i = 0; i < AXES; ++i) {
      if (std::fabs(cmd_pos_[i] - last_sent_pos_[i]) > cmd_eps_rad_) {
        changed = true;
        break;
      }
    }
    if (!changed) return hardware_interface::return_type::OK;
  }

  const bool vel_ok = (cmd_vel_.size() == AXES);
  auto vel_deg_s_for = [&](size_t i) -> double {
    double v = default_vel_deg_s_;
    if (vel_ok) v = std::fabs(rad2deg(cmd_vel_[i]));  // rad/s -> deg/s
    return v;
  };

  std::vector<double> pos_deg(AXES, 0.0);
  std::vector<double> vel_deg_s(AXES, default_vel_deg_s_);

  for (size_t i = 0; i < AXES; ++i) {
    const double rad_driver = (cmd_pos_[i] - rad_offset_[i]) * direction_sign_[i];
    if(i <6)
    {
      pos_deg[i] = rad2deg(rad_driver);
      vel_deg_s[i] = vel_deg_s_for(i);
    } else
    {
      pos_deg[i] = met2deg(rad_driver);
      vel_deg_s[i] = met2deg(i);
    }

  }

  try {
    client_.run_all(pos_deg, vel_deg_s);
    last_sent_pos_ = cmd_pos_;
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    if (!client_.is_connected()) {
      connected_ = false;
      publish_connected(false);
      RCLCPP_ERROR(
        get_logger(),
        "Robot TCP lost connection during write: %s:%d",
        robot_ip_.c_str(), robot_port_);
    }
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clk, 2000,
        "Robot TCP send failed: %s:%d: %s",
        robot_ip_.c_str(), robot_port_, e.what());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Robot TCP send failed: %s:%d: %s",
        robot_ip_.c_str(), robot_port_, e.what());
    }
    return hardware_interface::return_type::OK;
  }
}

}  // namespace robot_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RobotSystemHardware,
  hardware_interface::SystemInterface
)
