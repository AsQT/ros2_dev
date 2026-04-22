#include "robot_hardware_interface/rs485_protocol.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "robot_hardware_interface/srv/servo_on_axis.hpp"
#include "robot_hardware_interface/srv/servo_on_all.hpp"
#include "robot_hardware_interface/srv/jog.hpp"
#include "robot_hardware_interface/srv/home.hpp"
#include "robot_hardware_interface/srv/run_axis.hpp"
#include "robot_hardware_interface/srv/stop_axis.hpp"
#include "robot_hardware_interface/srv/stop_all.hpp"

#include "robot_hardware_interface/msg/flag_status.hpp"
#include "robot_hardware_interface/msg/axis_flag.hpp"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <chrono>

using robot_hardware_interface::Rs485Client;

namespace {
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
}  // namespace

class Rs485HwNode : public rclcpp::Node {
public:
  Rs485HwNode() : Node("rs485_hw") {
    // ======================
    // Parameters
    // ======================
    declare_parameter<std::string>("port", "");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<double>("serial_timeout_s", 0.2);
    declare_parameter<double>("poll_status_hz", 5.0);
    declare_parameter<double>("poll_pos_hz", 20.0);
    declare_parameter<double>("pos_timeout_s", 0.35);
    declare_parameter<double>("status_timeout_s", 0.5);
    declare_parameter<int>("all_token", 153);

    // Protocol bytes (frame = [0xAA, header1] ... [0xAA, tail1])
    declare_parameter<int>("proto_header1", 0xBB);
    declare_parameter<int>("proto_tail1", 0xFF);
    declare_parameter<int>("proto_stuff", 0xAA);

    // (Giữ lại cho tương thích, hiện HOME đã là SERVO_HOME_AX)
    declare_parameter<std::vector<double>>("home_positions_rad", std::vector<double>{});
    declare_parameter<double>("home_vel_rad_s", 0.5);

    declare_parameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_gl","joint_gr"}
    );
    declare_parameter<std::vector<int64_t>>("axis_ids", std::vector<int64_t>{0,1,2,3,4,5,6,7});

    // Cache joint_names / axis_ids
    joint_names_ = get_parameter("joint_names").as_string_array();
    {
      auto ids64 = get_parameter("axis_ids").as_integer_array();
      axis_ids_.clear();
      axis_ids_.reserve(ids64.size());
      for (auto v : ids64) axis_ids_.push_back(static_cast<int>(v));
    }

    if (axis_ids_.empty()) {
      axis_ids_.resize(joint_names_.size());
      for (size_t i = 0; i < axis_ids_.size(); ++i) axis_ids_[i] = static_cast<int>(i);
    }

    // Keep sizes consistent
    {
      const size_t n = std::min(joint_names_.size(), axis_ids_.size());
      joint_names_.resize(n);
      axis_ids_.resize(n);
    }

    // ======================
    // Publishers
    // ======================
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_connected_    = create_publisher<std_msgs::msg::Bool>("/hardware/connected", 10);
    //pub_flags_ =        create_publisher<std_msgs::msg::UInt16MultiArray>("/hardware/status_flags", 10);
    pub_status_flag_ = create_publisher<robot_hardware_interface::msg::FlagStatus>( "/hardware/flags", 10);
    // ======================
    // Subscribers
    // ======================
    sub_servo_axis_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/rs485_hw/cmd_servo_axis", 10,
      [this](std_msgs::msg::UInt8MultiArray::SharedPtr m){ on_servo_axis(m); });

    sub_run_axis_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rs485_hw/cmd_run_axis", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ on_run_axis(m); });

    sub_jog_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rs485_hw/cmd_jog", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ on_jog(m); });

    sub_run_all_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rs485_hw/cmd_run_all", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ on_run_all(m); });

    sub_traj_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/rs485_hw/joint_trajectory", 10,
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr m){ on_joint_traj(m); });

    // ======================
    // Services
    // ======================
    srv_connect_ = create_service<std_srvs::srv::Trigger>(
      "/rs485_hw/connect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){ on_connect(res); });

    srv_disconnect_ = create_service<std_srvs::srv::Trigger>(
      "/rs485_hw/disconnect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){ on_disconnect(res); });

    srv_poll_now_ = create_service<std_srvs::srv::Trigger>(
      "/rs485_hw/poll_now",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){ on_poll_now(res); });

    srv_servo_all_ = create_service<std_srvs::srv::SetBool>(
      "/rs485_hw/servo_all",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res){ on_servo_all(req, res); });

    // Typed services
    srv_servo_on_axis_ = create_service<robot_hardware_interface::srv::ServoOnAxis>(
      "/rs485_hw/servo_on_axis",
      [this](const std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Response> res){ on_servo_on_axis(req, res); });

    srv_servo_on_all_ = create_service<robot_hardware_interface::srv::ServoOnAll>(
      "/rs485_hw/servo_on_all",
      [this](const std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Response> res){ on_servo_on_all(req, res); });

    srv_jog_ = create_service<robot_hardware_interface::srv::Jog>(
      "/rs485_hw/jog",
      [this](const std::shared_ptr<robot_hardware_interface::srv::Jog::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::Jog::Response> res){ on_jog_srv(req, res); });

    srv_home_ = create_service<robot_hardware_interface::srv::Home>(
      "/rs485_hw/home",
      [this](const std::shared_ptr<robot_hardware_interface::srv::Home::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::Home::Response> res){ on_home(req, res); });

    srv_run_axis_ = create_service<robot_hardware_interface::srv::RunAxis>(
      "/rs485_hw/run_axis",
      [this](const std::shared_ptr<robot_hardware_interface::srv::RunAxis::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::RunAxis::Response> res){ on_run_axis_srv(req, res); });

    srv_stop_axis_ = create_service<robot_hardware_interface::srv::StopAxis>(
      "/rs485_hw/stop_axis",
      [this](const std::shared_ptr<robot_hardware_interface::srv::StopAxis::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::StopAxis::Response> res){ on_stop_axis(req, res); });

    srv_stop_all_ = create_service<robot_hardware_interface::srv::StopAll>(
      "/rs485_hw/stop_all",
      [this](const std::shared_ptr<robot_hardware_interface::srv::StopAll::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::StopAll::Response> res){ on_stop_all(req, res); });

    // ======================
    // Timers
    // ======================
    double poll_status_hz = get_parameter("poll_status_hz").as_double();

    if (poll_status_hz > 0.0) poll_status_hz = std::max(0.1, poll_status_hz);

    if (poll_status_hz > 0.0) {
      timer_status_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / poll_status_hz)),
        [this](){ 
          //poll_all(); 
          //RCLCPP_INFO(get_logger(), "poll_all");
        } 
      );
    } else {
      RCLCPP_WARN(get_logger(), "poll_status_hz <= 0 -> status polling disabled");
    }
    publish_connected(false);
    RCLCPP_INFO(get_logger(), "rs485_hw ready. Set params then call /rs485_hw/connect");
  }

private:
  static constexpr size_t kStatusAxes = 6;

  // ======================
  // Helpers
  // ======================
  void publish_connected(bool ok) {
    std_msgs::msg::Bool m; m.data = ok;
    pub_connected_->publish(m);
  }

  bool connected() const { return client_.is_connected(); }

  static inline bool has_flag(uint32_t v, uint32_t mask) {
    return (v & mask) != 0u;
  }
  // ======================
  // Core services
  // ======================
  void on_connect(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    try {
      auto port = get_parameter("port").as_string();
      auto baud = static_cast<int>(get_parameter("baudrate").as_int());
      auto to_s = get_parameter("serial_timeout_s").as_double();
      if (port.empty()) throw std::runtime_error("port param empty");

      // Apply protocol bytes BEFORE connecting
      robot_hardware_interface::ProtocolBytes p = client_.protocol();
      p.header1 = static_cast<uint8_t>(get_parameter("proto_header1").as_int() & 0xFF);
      p.tail1   = static_cast<uint8_t>(get_parameter("proto_tail1").as_int() & 0xFF);
      p.stuff   = static_cast<uint8_t>(get_parameter("proto_stuff").as_int() & 0xFF);
      client_.set_protocol(p);

      client_.connect(port, baud, to_s);
      publish_connected(true);

      res->success = true;
      res->message = "Connected";

      RCLCPP_INFO(get_logger(), "Connected %s @ %d", port.c_str(), baud);
      RCLCPP_INFO(get_logger(),
                  "Protocol bytes: header=[0x%02X 0x%02X] tail=[0x%02X 0x%02X] stuff=0x%02X",
                  p.header0, p.header1, p.tail0, p.tail1, p.stuff);
    } catch (const std::exception& e) {
      publish_connected(false);
      res->success = false;
      res->message = e.what();
      RCLCPP_ERROR(get_logger(), "Connect failed: %s", e.what());
    }
  }

  void on_disconnect(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    try {
      client_.disconnect();
      res->success = true;
      res->message = "Disconnected";
      RCLCPP_INFO(get_logger(), "Disconnected");
    } catch (const std::exception &e) {
      res->success = false;
      res->message = e.what();
      RCLCPP_WARN(get_logger(), "disconnect: %s", e.what());
    }
    publish_connected(false);
  }

  void on_poll_now(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    //poll_status();
    poll_all();
    //poll_positions();
    RCLCPP_INFO(get_logger(), "on_poll_now");
    res->success = true;
    res->message = "OK";
  }

  void on_servo_all(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      const int token = get_parameter("all_token").as_int();
      (void)token;
      client_.servo_all(req->data);
      res->success = true;
      res->message = req->data ? "Servo ALL ON" : "Servo ALL OFF";
    } catch (const std::exception& e) {
      res->success = false;
      res->message = e.what();
    }
  }
void publish_status(const std::vector<uint16_t> & flag_s)
{
  robot_hardware_interface::msg::FlagStatus msg;

  const size_t n = std::min(flag_s.size(), msg.axes.size());

  for (size_t axis = 0; axis < n; ++axis) {
    const uint16_t st = flag_s[axis];
    auto & a = msg.axes[axis];

    a.servo_on      = (st & 0x0001) != 0; 
    a.error_all     = (st & 0x0002) != 0;
    a.org_ok        = (st & 0x0004) != 0;
    a.motionning    = (st & 0x0008) != 0;
    a.org_retunning = (st & 0x0010) != 0;
    a.limit_pos     = (st & 0x0020) != 0;
    a.limit_neg     = (st & 0x0040) != 0;
    a.org_sensor    = (st & 0x0080) != 0;
    a.alarm_rst     = (st & 0x0100) != 0;
    a.emg           = (st & 0x0200) != 0;
    a.stop          = (st & 0x0400) != 0;
    a.communi_err   = (st & 0x8000) != 0;

    a.status_f      = st;
  }

  pub_status_flag_->publish(msg);
}
  // ======================
  // Typed service handlers
  // ======================
  void on_servo_on_axis(const std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Request> req,
                        std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      const uint8_t id = req->id;
      const bool on = (req->state != 0);
      if (on) client_.servo_on_axis(id);
      else    client_.servo_off_axis(id);
      res->ok = true;
      res->error_code = 0;
      res->message = on ? "Servo ON" : "Servo OFF";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void on_servo_on_all(const std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Request> req,
                       std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      const int token = get_parameter("all_token").as_int();
      (void)token;
      const bool on = (req->state != 0);
      client_.servo_all(on);
      res->ok = true;
      res->error_code = 0;
      res->message = on ? "Servo ALL ON" : "Servo ALL OFF";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void on_jog_srv(const std::shared_ptr<robot_hardware_interface::srv::Jog::Request> req,
                  std::shared_ptr<robot_hardware_interface::srv::Jog::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      const uint8_t id = req->id;
      const bool dir_plus = (req->dir != 0);
      const double vel_deg_s = std::fabs(rad2deg(req->vel));
      client_.jog(id, dir_plus, vel_deg_s);
      res->ok = true;
      res->error_code = 0;
      if (vel_deg_s <= 0.0) res->message = "Jog STOP";
      else res->message = dir_plus ? "Jog +" : "Jog -";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void on_run_axis_srv(const std::shared_ptr<robot_hardware_interface::srv::RunAxis::Request> req,
                       std::shared_ptr<robot_hardware_interface::srv::RunAxis::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      const uint8_t id = req->id;
      const double pos_deg = rad2deg(req->pos);
      const double vel_deg_s = std::fabs(rad2deg(req->vel));
      client_.run_axis(id, pos_deg, vel_deg_s);
      res->ok = true;
      res->error_code = 0;
      res->message = "RUN axis";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void on_home(const std::shared_ptr<robot_hardware_interface::srv::Home::Request> req,
               std::shared_ptr<robot_hardware_interface::srv::Home::Response> res)
  {
    try {
      if (!connected()) throw std::runtime_error("Not connected");

      const uint8_t axis = req->id;

      // kiểm tra axis có nằm trong axis_ids_
      auto it = std::find(axis_ids_.begin(), axis_ids_.end(), static_cast<int>(axis));
      if (it == axis_ids_.end()) {
        throw std::runtime_error("axis id not found in axis_ids param");
      }

      client_.servo_home_axis(axis);

      res->ok = true;
      res->error_code = 0;
      res->message = "HOME (SERVO_HOME_AX)";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void on_stop_axis(const std::shared_ptr<robot_hardware_interface::srv::StopAxis::Request> req,
                    std::shared_ptr<robot_hardware_interface::srv::StopAxis::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      // Protocol defines: JOG vel=0 => stop.
      client_.jog(req->id, true, 0.0);
      res->ok = true;
      res->error_code = 0;
      res->message = "STOP axis";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void on_stop_all(const std::shared_ptr<robot_hardware_interface::srv::StopAll::Request>,
                   std::shared_ptr<robot_hardware_interface::srv::StopAll::Response> res) {
    try {
      if (!connected()) throw std::runtime_error("Not connected");
      for (const int axis_i : axis_ids_) {
        client_.jog(static_cast<uint8_t>(axis_i & 0xFF), true, 0.0);
      }
      res->ok = true;
      res->error_code = 0;
      res->message = "STOP ALL";
    } catch (const std::exception& e) {
      res->ok = false;
      res->error_code = connected() ? 3 : 1;
      res->message = e.what();
    }
  }

  void poll_all()
  {
    if (!connected()) 
    {
      //RCLCPP_INFO(get_logger(), "Polling fail, because not connect port");
      return;
    }

    const size_t n_cfg = std::min(joint_names_.size(), axis_ids_.size());
    if (n_cfg == 0)
    {
      RCLCPP_INFO(get_logger(), "Polling fail, because erro lenght");
      return;
    }

    const double to = get_parameter("pos_timeout_s").as_double();

    std::vector<double> pos_deg;
    std::vector<double> vel_deg_s;
    std::vector<uint16_t> flag_s;
    try {
      std::tie(pos_deg, vel_deg_s, flag_s) = client_.get_all_state(to);
      /*
      if (!pos_deg.empty() && !vel_deg_s.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "J1 raw pos = %.3f deg | vel = %.3f deg/s",
          pos_deg[0],
          vel_deg_s[0]
        );
      }
      */ 
    } catch (const std::exception &e) {
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *clk, 2000, "poll_positions(get_pos_all): %s", e.what());
      } else {
        RCLCPP_WARN(get_logger(), "poll_positions(get_pos_all): %s", e.what());
      }
      return;
    }
    const size_t avail = std::min(pos_deg.size(), vel_deg_s.size());
    if (avail == 0) return;

    if (n_cfg > avail) {
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *clk, 5000,
          "get_pos_all returned %zu axes, but node configured for %zu joints -> publishing first %zu",
          avail, n_cfg, avail);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "get_pos_all returned %zu axes, but node configured for %zu joints -> publishing first %zu",
          avail, n_cfg, avail);
      }
    }

    const size_t m = std::min(n_cfg, avail);

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name.assign(joint_names_.begin(), joint_names_.begin() + m);

    js.position.resize(m);
    js.velocity.resize(m);

    
    /*
    RCLCPP_INFO(
      get_logger(),
      "J1 pos = %.3f rad | vel = %.3f rad/s | flag = 0x%04X",
      js.position[5],
      js.velocity[5],
      static_cast<unsigned int>(flag_s[5])
    );
    */
    for (size_t i = 0; i < m; ++i) {
      js.position[i] = deg2rad(pos_deg[i]);
      js.velocity[i] = deg2rad(vel_deg_s[i]);
    }
    //std_msgs::msg::UInt16MultiArray msg;

    //msg.data = flag_s;

    //pub_flags_->publish(msg);
    pub_joint_states_->publish(js);
    publish_status(flag_s);
  }

  // ======================
  // Subscribers handlers
  // ======================
  void on_servo_axis(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (!connected() || msg->data.size() < 2) return;
    const uint8_t id = msg->data[0];
    const uint8_t on = msg->data[1];
    try {
      if (on) client_.servo_on_axis(id);
      else    client_.servo_off_axis(id);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "servo_axis: %s", e.what());
    }
  }

  void on_run_axis(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!connected() || msg->data.size() < 3) return;
    const uint8_t id = static_cast<uint8_t>(llround(msg->data[0]));
    const double pos_deg = msg->data[1];
    const double vel_deg_s = msg->data[2];
    try {
      client_.run_axis(id, pos_deg, vel_deg_s);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "run_axis: %s", e.what());
    }
  }

  void on_jog(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!connected() || msg->data.size() < 3) return;
    const uint8_t id = static_cast<uint8_t>(llround(msg->data[0]));
    const bool dir = (llround(msg->data[1]) != 0);
    const double vel_deg_s = msg->data[2];
    try {
      client_.jog(id, dir, vel_deg_s);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "jog: %s", e.what());
    }
  }

  void on_run_all(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!connected()) return;

    const size_t n = axis_ids_.size();
    if (n == 0) return;
    if (msg->data.size() < 2 * n) return;

    // Firmware optimized RUN_ALL for exactly 6 axes
    if (n == 6) {
      std::vector<double> pos(6), vel(6);
      for (size_t i = 0; i < 6; ++i) pos[i] = msg->data[i];
      for (size_t i = 0; i < 6; ++i) vel[i] = msg->data[6 + i];
      try {
        client_.run_all(pos, vel);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "run_all: %s", e.what());
      }
      return;
    }

    // Otherwise: per-axis
    try {
      for (size_t i = 0; i < n; ++i) {
        const uint8_t axis = static_cast<uint8_t>(axis_ids_[i]);
        const double pos_deg = msg->data[i];
        const double vel_deg_s = msg->data[n + i];
        client_.run_axis(axis, pos_deg, vel_deg_s);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "run_all(per-axis): %s", e.what());
    }
  }

  void on_joint_traj(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (!connected() || msg->points.empty()) return;

    const auto& p = msg->points.back();
    const size_t n = axis_ids_.size();
    if (n == 0) return;
    if (p.positions.size() < n) return;

    // Firmware optimized RUN_ALL for exactly 6 axes
    if (n == 6) {
      std::vector<double> pos_deg(6), vel_deg_s(6, 10.0);
      for (size_t i = 0; i < 6; ++i) pos_deg[i] = p.positions[i] * 180.0 / M_PI;

      if (p.velocities.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) vel_deg_s[i] = std::fabs(p.velocities[i] * 180.0 / M_PI);
      }

      try {
        client_.run_all(pos_deg, vel_deg_s);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "joint_trajectory: %s", e.what());
      }
      return;
    }

    // Otherwise: per-axis
    try {
      for (size_t i = 0; i < n; ++i) {
        const uint8_t axis = static_cast<uint8_t>(axis_ids_[i]);
        const double pos_deg = p.positions[i] * 180.0 / M_PI;
        double vel_deg_s = 10.0;
        if (p.velocities.size() >= n) vel_deg_s = std::fabs(p.velocities[i] * 180.0 / M_PI);
        client_.run_axis(axis, pos_deg, vel_deg_s);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "joint_trajectory(per-axis): %s", e.what());
    }
  }

private:
  Rs485Client client_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_;
  //rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_flags_;
  rclcpp::Publisher<robot_hardware_interface::msg::FlagStatus>::SharedPtr pub_status_flag_;


  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_servo_axis_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_run_axis_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_jog_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_run_all_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_traj_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_connect_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disconnect_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_poll_now_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_servo_all_;

  rclcpp::Service<robot_hardware_interface::srv::ServoOnAxis>::SharedPtr srv_servo_on_axis_;
  rclcpp::Service<robot_hardware_interface::srv::ServoOnAll>::SharedPtr srv_servo_on_all_;
  rclcpp::Service<robot_hardware_interface::srv::Jog>::SharedPtr srv_jog_;
  rclcpp::Service<robot_hardware_interface::srv::Home>::SharedPtr srv_home_;
  rclcpp::Service<robot_hardware_interface::srv::RunAxis>::SharedPtr srv_run_axis_;
  rclcpp::Service<robot_hardware_interface::srv::StopAxis>::SharedPtr srv_stop_axis_;
  rclcpp::Service<robot_hardware_interface::srv::StopAll>::SharedPtr srv_stop_all_;

  rclcpp::TimerBase::SharedPtr timer_status_;
  rclcpp::TimerBase::SharedPtr timer_pos_;

  std::vector<std::string> joint_names_;
  std::vector<int> axis_ids_;
  uint32_t counter_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rs485HwNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
