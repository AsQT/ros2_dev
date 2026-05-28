// Plugin
#include "robot_hardware_interface/rs485_system_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cctype>
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

  port_               = get_s("port", port_);
  baudrate_           = get_i("baudrate", baudrate_);
  serial_timeout_s_   = get_d("serial_timeout_s", serial_timeout_s_);
  pos_timeout_s_      = get_d("pos_timeout_s", pos_timeout_s_);
  status_timeout_s_   = get_d("status_timeout_s", status_timeout_s_);
  all_token_          = get_i("all_token", all_token_);
  default_vel_deg_s_  = get_d("default_vel_deg_s", default_vel_deg_s_);

  {
    ProtocolBytes p = client_.protocol();
    p.header1 = static_cast<uint8_t>(get_i("proto_header1", p.header1) & 0xFF);
    p.tail1   = static_cast<uint8_t>(get_i("proto_tail1",   p.tail1) & 0xFF);
    p.stuff   = static_cast<uint8_t>(get_i("proto_stuff",   p.stuff) & 0xFF);
    client_.set_protocol(p);
    RCLCPP_INFO(get_logger(), "RS485 protocol: header=[0x%02X 0x%02X] tail=[0x%02X 0x%02X] stuff=0x%02X",
                p.header0, p.header1, p.tail0, p.tail1, p.stuff);
  }

  const size_t n = info_.joints.size();
  if (n == 0) {
    RCLCPP_ERROR(get_logger(), "No joints configured in URDF ros2_control tag.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  axis_ids_ = parse_csv_ints(get_s("axis_ids", ""));
  if (axis_ids_.size() != n) {
    axis_ids_.clear();
    axis_ids_.reserve(n);
    for (size_t i = 0; i < n; ++i) axis_ids_.push_back(static_cast<int>(i) + 1);
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
  try {
    client_.connect(port_, baudrate_, serial_timeout_s_);
    connected_ = true;
    RCLCPP_INFO(get_logger(), "Connected %s @ %d", port_.c_str(), baudrate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "RS485 connect failed: %s", e.what());
    connected_ = false;
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!connected_) {
    try {
      client_.connect(port_, baudrate_, serial_timeout_s_);
      connected_ = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "RS485 connect failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }


  auto ret = read(rclcpp::Time(0), rclcpp::Duration(0, 0));
  if (ret == hardware_interface::return_type::OK) {
    cmd_pos_ = hw_pos_;           // tránh giật về 0
    last_sent_pos_ = cmd_pos_;    // coi như đã "gửi" trạng thái hiện tại
    state_synced_ = true;
  } else {
    RCLCPP_WARN(get_logger(), "Initial read() failed. Holding write until a successful read.");
    state_synced_ = false;        // sẽ chờ read() thành công sau
  }

  warmup_cycles_ = warmup_cycles_cfg_;
  last_write_time_ = get_clock()->now();

  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    client_.servo_all(false);
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "servo_all(off) failed: %s", e.what());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
/*__________________________________________________________________________________*/
hardware_interface::CallbackReturn RobotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  try { client_.disconnect(); } catch (...) {}
  connected_ = false;
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
  if (!connected_) return hardware_interface::return_type::ERROR;

  const size_t n = hw_pos_.size();

  try {
    const auto [pos_raw, vel_raw, flag] = client_.get_all_state(pos_timeout_s_);

    if (pos_raw.size() != n || vel_raw.size() != n) {
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *clk, 2000,
          "read get_pos_all size mismatch: pos=%zu vel=%zu expected=%zu (keeping last state)",
          pos_raw.size(), vel_raw.size(), n);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "read get_pos_all size mismatch: pos=%zu vel=%zu expected=%zu (keeping last state)",
          pos_raw.size(), vel_raw.size(), n);
      }

      consec_read_fail_++;
      return (consec_read_fail_ >= max_consec_read_fail_)
        ? hardware_interface::return_type::ERROR
        : hardware_interface::return_type::OK;
    }
    // OK -> reset fail counter
    consec_read_fail_ = 0;

    for (size_t i = 0; i < n; ++i) {
      // pos_raw/vel_raw đang là milli-degree (deg*1000)
      const double pos_deg   = pos_raw[i];
      const double vel_deg_s = vel_raw[i];
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
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clk, 2000, "read get_pos_all failed: %s (keeping last state)", e.what());
    } else {
      RCLCPP_WARN(get_logger(), "read get_pos_all failed: %s (keeping last state)", e.what());
    }

    consec_read_fail_++;
    return (consec_read_fail_ >= max_consec_read_fail_)
      ? hardware_interface::return_type::ERROR
      : hardware_interface::return_type::OK;
  }
}
 
hardware_interface::return_type RobotSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!connected_) return hardware_interface::return_type::ERROR;

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
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clk, 2000, "write run_all failed: %s", e.what());
    } else {
      RCLCPP_WARN(get_logger(), "write run_all failed: %s", e.what());
    }
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace robot_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RobotSystemHardware,
  hardware_interface::SystemInterface
)
