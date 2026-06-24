#include "robot_hardware_interface/tcp_client.hpp"

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
#include <iomanip>
#include <tuple>

using robot_hardware_interface::RobotTcpClient;

namespace {
constexpr const char * ROBOT_CONNECTED_TOPIC = "/robot_hw/connected";
constexpr const char * ROBOT_STATUS_TEXT_TOPIC = "/robot_hw/status_text";
constexpr const char * LEGACY_CONNECTED_TOPIC = "/hardware/connected";
constexpr const char * ROBOT_FLAGS_TOPIC = "/robot_hw/flags";
constexpr const char * LEGACY_FLAGS_TOPIC = "/hardware/flags";
constexpr const char * ROBOT_IP_DEFAULT = "192.168.2.50";
constexpr int ROBOT_PORT_DEFAULT = 5000;
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

double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
}  // namespace

class RobotHwNode : public rclcpp::Node {
public:
  RobotHwNode() : Node("robot_hw") {
    /*___________ Parameters ______________*/
    declare_parameter<std::string>("robot_ip", ROBOT_IP_DEFAULT);
    declare_parameter<int>("robot_port", ROBOT_PORT_DEFAULT);
    declare_parameter<int>("connect_timeout_ms", 2000);
    declare_parameter<int>("read_timeout_ms", 50);
    declare_parameter<double>("poll_pos_hz", 20.0);
    declare_parameter<int>("all_token", 153);
    declare_parameter<bool>("auto_connect", true);

    declare_parameter<std::vector<double>>("home_positions_rad", std::vector<double>{});
    declare_parameter<double>("home_vel_rad_s", 0.5);

    declare_parameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"});
    declare_parameter<std::vector<int64_t>>("axis_ids", std::vector<int64_t>{0,1,2,3,4,5});

    /*_____ Cache joint_names / axis_ids ___________*/
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

    /*_____________ Publishers _______________*/ 
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_connected_ = create_publisher<std_msgs::msg::Bool>(ROBOT_CONNECTED_TOPIC, 10);
    pub_connected_legacy_ = create_publisher<std_msgs::msg::Bool>(LEGACY_CONNECTED_TOPIC, 10);
    pub_status_text_ = create_publisher<std_msgs::msg::String>(ROBOT_STATUS_TEXT_TOPIC, 10);
    pub_status_flag_ = create_publisher<robot_hardware_interface::msg::FlagStatus>(ROBOT_FLAGS_TOPIC, 10);
    pub_status_flag_legacy_ =
      create_publisher<robot_hardware_interface::msg::FlagStatus>(LEGACY_FLAGS_TOPIC, 10);
    /*_____________ Subscribers _______________*/ 
    sub_servo_axis_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/robot_hw/cmd_servo_axis", 10,
      [this](std_msgs::msg::UInt8MultiArray::SharedPtr m){ on_servo_axis(m); });

    sub_run_axis_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/robot_hw/cmd_run_axis", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ on_run_axis(m); });

    sub_jog_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/robot_hw/cmd_jog", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ on_jog(m); });

    sub_run_all_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/robot_hw/cmd_run_all", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ on_run_all(m); });

    sub_traj_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/robot_hw/joint_trajectory", 10,
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr m){ on_joint_traj(m); });

    /*__________ Services ____________*/ 
    srv_connect_ = create_service<std_srvs::srv::Trigger>(
      "/robot_hw/connect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){ on_connect(res); });

    srv_disconnect_ = create_service<std_srvs::srv::Trigger>(
      "/robot_hw/disconnect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){ on_disconnect(res); });

    srv_poll_now_ = create_service<std_srvs::srv::Trigger>(
      "/robot_hw/poll_now",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){ on_poll_now(res); });

    srv_servo_all_ = create_service<std_srvs::srv::SetBool>(
      "/robot_hw/servo_all",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res){ on_servo_all(req, res); });

    // Typed services
    srv_servo_on_axis_ = create_service<robot_hardware_interface::srv::ServoOnAxis>(
      "/robot_hw/servo_on_axis",
      [this](const std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Response> res){ on_servo_on_axis(req, res); });

    srv_servo_on_all_ = create_service<robot_hardware_interface::srv::ServoOnAll>(
      "/robot_hw/servo_on_all",
      [this](const std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Response> res){ on_servo_on_all(req, res); });

    srv_jog_ = create_service<robot_hardware_interface::srv::Jog>(
      "/robot_hw/jog",
      [this](const std::shared_ptr<robot_hardware_interface::srv::Jog::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::Jog::Response> res){ on_jog_srv(req, res); });

    srv_home_ = create_service<robot_hardware_interface::srv::Home>(
      "/robot_hw/home",
      [this](const std::shared_ptr<robot_hardware_interface::srv::Home::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::Home::Response> res){ on_home(req, res); });

    srv_run_axis_ = create_service<robot_hardware_interface::srv::RunAxis>(
      "/robot_hw/run_axis",
      [this](const std::shared_ptr<robot_hardware_interface::srv::RunAxis::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::RunAxis::Response> res){ on_run_axis_srv(req, res); });

    srv_stop_axis_ = create_service<robot_hardware_interface::srv::StopAxis>(
      "/robot_hw/stop_axis",
      [this](const std::shared_ptr<robot_hardware_interface::srv::StopAxis::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::StopAxis::Response> res){ on_stop_axis(req, res); });

    srv_stop_all_ = create_service<robot_hardware_interface::srv::StopAll>(
      "/robot_hw/stop_all",
      [this](const std::shared_ptr<robot_hardware_interface::srv::StopAll::Request> req,
             std::shared_ptr<robot_hardware_interface::srv::StopAll::Response> res){ on_stop_all(req, res); });

    /*__________Timers ____________*/ 
    double poll_pos_hz = get_parameter("poll_pos_hz").as_double();

    if (poll_pos_hz > 0.0) poll_pos_hz = std::max(0.1, poll_pos_hz);

    if (poll_pos_hz > 0.0) {
      timer_pos_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / poll_pos_hz)),
        [this](){ poll_all(); }
      );
    } else {
      RCLCPP_WARN(get_logger(), "poll_pos_hz <= 0 -> state polling disabled");
    }
    timer_health_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        publish_connected(connected_state_);
        publish_status_text(status_text_state_);
      });
    publish_connected(false);
    publish_status_text("Robot TCP hardware node ready");
    publish_status(std::vector<uint32_t>(6, 0));
    if (get_parameter("auto_connect").as_bool()) {
      timer_auto_connect_ = create_wall_timer(
        std::chrono::milliseconds(300),
        [this]() {
          timer_auto_connect_->cancel();
          std::string message;
          if (!connect_robot(message)) {
            RCLCPP_ERROR(get_logger(), "Auto connect failed: %s", message.c_str());
          }
        });
      RCLCPP_INFO(get_logger(), "Robot TCP hardware node ready. Auto connect enabled.");
    } else {
      RCLCPP_INFO(get_logger(), "Robot TCP hardware node ready. Set params then call /robot_hw/connect");
    }
  }

  ~RobotHwNode() override {
    try {
      if (connected()) {
        client_.servo_all(false);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "servo_all(off) during shutdown failed: %s", e.what());
    }
    try {
      client_.disconnect();
    } catch (...) {
    }
  }

private:
  /*___________ Helpers _______________*/ 
  void publish_connected(bool ok) {
    connected_state_ = ok;
    std_msgs::msg::Bool m; m.data = ok;
    pub_connected_->publish(m);
    pub_connected_legacy_->publish(m);
  }

  void publish_status_text(const std::string & text) {
    status_text_state_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    pub_status_text_->publish(msg);
  }

  bool connected() const { return client_.is_connected(); }

  bool connect_robot(std::string & message) {
    try {
      if (connected()) {
        publish_connected(true);
        publish_status_text("Robot TCP connected");
        message = "Connected";
        return true;
      }

      auto ip = get_parameter("robot_ip").as_string();
      auto port = static_cast<int>(get_parameter("robot_port").as_int());
      const auto connect_timeout_ms =
        static_cast<int>(get_parameter("connect_timeout_ms").as_int());
      const auto read_timeout_ms =
        static_cast<int>(get_parameter("read_timeout_ms").as_int());
      if (ip.empty()) throw std::runtime_error("robot_ip param empty");

      first_state_logged_ = false;
      first_state_request_logged_ = false;
      first_joint_states_logged_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Robot TCP connecting: %s:%d (connect_timeout=%d ms, read_timeout=%d ms)",
        ip.c_str(), port, connect_timeout_ms, read_timeout_ms);
      if (!client_.connect(ip, port, connect_timeout_ms)) {
        throw std::runtime_error(client_.last_error());
      }
      publish_connected(true);
      publish_status_text("Robot TCP connected");

      RCLCPP_INFO(get_logger(), "Robot TCP connect ok: %s:%d", ip.c_str(), port);
      const auto poll_pos_hz = get_parameter("poll_pos_hz").as_double();
      RCLCPP_INFO(
        get_logger(),
        "Robot TCP state polling started: %.3f Hz, read_timeout=%d ms",
        poll_pos_hz,
        read_timeout_ms);
      RCLCPP_INFO(
        get_logger(),
        "Robot TCP frame: magic=0x%04X wire=AA 55 header=%zu bytes",
        robot_hardware_interface::ROBOT_TCP_MAGIC,
        robot_hardware_interface::ROBOT_TCP_HEADER_SIZE);
      message = "Connected";
      return true;
    } catch (const std::exception & e) {
      publish_connected(false);
      publish_status_text(std::string("Robot TCP connect failed: ") + e.what());
      message = e.what();
      return false;
    }
  }

  /* __________________ Core services ______________*/ 
  void on_connect(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    std::string message;
    res->success = connect_robot(message);
    res->message = message;
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "TCP connect failed: %s", message.c_str());
    }
  }

  void on_disconnect(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    try {
      if (connected()) {
        client_.servo_all(false);
      }
      client_.disconnect();
      res->success = true;
      res->message = "Disconnected";
      RCLCPP_INFO(get_logger(), "TCP disconnected");
    } catch (const std::exception &e) {
      res->success = false;
      res->message = e.what();
      RCLCPP_WARN(get_logger(), "disconnect: %s", e.what());
    }
    publish_connected(false);
    publish_status_text("Robot TCP disconnected");
  }

  void on_poll_now(std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    poll_all();
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
void publish_status(const std::vector<uint32_t> & flag_s)
{
  robot_hardware_interface::msg::FlagStatus msg;
  const size_t expected_axes = msg.axes.size();
  if (flag_s.size() < expected_axes) {
    if (auto clk = get_clock()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clk, 5000,
        "/robot_hw/flags got %zu flags, expected %zu; missing axes publish status_f=0",
        flag_s.size(), expected_axes);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "/robot_hw/flags got %zu flags, expected %zu; missing axes publish status_f=0",
        flag_s.size(), expected_axes);
    }
  }

  for (size_t axis = 0; axis < expected_axes; ++axis) {
    const uint32_t st = axis < flag_s.size() ? flag_s[axis] : 0u;
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

  pub_status_flag_->publish(msg);
  pub_status_flag_legacy_->publish(msg);
}
  /*___________ Typed service handlers _______________*/ 
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
      //RCLCPP_INFO(get_logger(), "Polling fail, because TCP is not connected");
      publish_status(std::vector<uint32_t>(6, 0));
      return;
    }

    const size_t n_cfg = std::min(joint_names_.size(), axis_ids_.size());
    if (n_cfg == 0)
    {
      RCLCPP_INFO(get_logger(), "Polling fail, because erro lenght");
      publish_status(std::vector<uint32_t>(6, 0));
      return;
    }

    const int timeout_ms = static_cast<int>(get_parameter("read_timeout_ms").as_int());

    std::vector<double> pos_deg;
    std::vector<double> vel_deg_s;
    std::vector<uint32_t> flag_s;
    try {
      if (!first_state_request_logged_) {
        first_state_request_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Robot TCP sending first state request: cmd=0x%02X read_timeout=%d ms",
          robot_hardware_interface::cmd::STATUS_ALL,
          timeout_ms);
      }
      std::tie(pos_deg, vel_deg_s, flag_s) = client_.get_all_state(timeout_ms);
    } catch (const std::exception &e) {
      if (!client_.is_connected()) {
        publish_connected(false);
        publish_status_text("Robot TCP disconnected while polling state");
      }
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *clk, 2000,
          "get_all_state failed while polling: %s (publishing default flags)", e.what());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "get_all_state failed while polling: %s (publishing default flags)", e.what());
      }
      publish_status(std::vector<uint32_t>(6, 0));
      return;
    }
    const size_t avail = std::min(pos_deg.size(), vel_deg_s.size());
    if (avail == 0) {
      if (auto clk = get_clock()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *clk, 2000,
          "get_all_state returned empty state: pos=%zu vel=%zu (publishing default flags)",
          pos_deg.size(), vel_deg_s.size());
      }
      publish_status(std::vector<uint32_t>(6, 0));
      return;
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;

    js.position.resize(n_cfg, 0.0);
    js.velocity.resize(n_cfg, 0.0);

    for (size_t i = 0; i < n_cfg; ++i) {
      const int axis_id = axis_ids_[i];
      if (axis_id < 0 || static_cast<size_t>(axis_id) >= avail) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping joint '%s': axis_id=%d outside state array length=%zu",
          joint_names_[i].c_str(), axis_id, avail);
        continue;
      }
      const size_t axis_index = static_cast<size_t>(axis_id);
      js.position[i] = deg2rad(pos_deg[axis_index]);
      js.velocity[i] = deg2rad(vel_deg_s[axis_index]);
      const uint32_t flags = axis_index < flag_s.size() ? flag_s[axis_index] : 0;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "state axis_id=%d joint=%s position_mdeg=%d velocity_mdeg_s=%u flags=0x%08X",
        axis_id,
        joint_names_[i].c_str(),
        static_cast<int>(std::llround(pos_deg[axis_index] * 1000.0)),
        static_cast<unsigned>(std::llround(std::fabs(vel_deg_s[axis_index]) * 1000.0)),
        flags);
    }
    if (!first_state_logged_) {
      first_state_logged_ = true;
      publish_status_text("Robot TCP received first valid state frame");
      std::ostringstream raw_state;
      for (size_t i = 0; i < std::min<size_t>(
          robot_hardware_interface::ROBOT_STATUS_FRAME_AXIS_COUNT, pos_deg.size()); ++i)
      {
        if (i > 0) {
          raw_state << "; ";
        }
        const uint32_t flag = i < flag_s.size() ? flag_s[i] : 0u;
        const double vel = i < vel_deg_s.size() ? vel_deg_s[i] : 0.0;
        raw_state << "axis[" << i << "]: pos=" << static_cast<int>(std::llround(pos_deg[i] * 1000.0))
                  << " vel=" << static_cast<unsigned>(std::llround(std::fabs(vel) * 1000.0))
                  << " flag=0x" << std::hex << std::uppercase << std::setw(8)
                  << std::setfill('0') << flag << std::dec << std::setfill(' ');
      }
      RCLCPP_INFO(
        get_logger(),
        "CMD_GET_ALL OK: payload_len=%zu status=0x%02X axis_offset=%zu axis_count=%zu axis_bytes=%zu configured_joints=%zu %s",
        client_.last_state_payload_length(),
        static_cast<unsigned>(robot_hardware_interface::ROBOT_CMD_OK),
        client_.last_state_payload_offset(),
        flag_s.size(),
        client_.last_state_axis_bytes(),
        n_cfg,
        raw_state.str().c_str());
    }
    pub_joint_states_->publish(js);
    if (!first_joint_states_logged_) {
      first_joint_states_logged_ = true;
      RCLCPP_INFO(get_logger(), "Published first /joint_states message with %zu joints", js.name.size());
    }
    publish_status(flag_s);
  }
  /*__________ Subscribers handlers __________________*/
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
    // Firmware optimized RUN_ALL for exactly 8 axes
    if (n == 8) {
      std::vector<double> pos(8), vel(8);
      for (size_t i = 0; i < 8; ++i) pos[i] = msg->data[i];
      for (size_t i = 0; i < 8; ++i) vel[i] = msg->data[8 + i];
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
    // Firmware optimized RUN_ALL for exactly 8 axes
    if (n == 8) {
      std::vector<double> pos_deg(8), vel_deg_s(8, 10.0);
      for (size_t i = 0; i < 8; ++i) pos_deg[i] = p.positions[i] * 180.0 / M_PI;

      if (p.velocities.size() >= 8) {
        for (size_t i = 0; i < 8; ++i) vel_deg_s[i] = std::fabs(p.velocities[i] * 180.0 / M_PI);
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
  RobotTcpClient client_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_legacy_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_text_;
  rclcpp::Publisher<robot_hardware_interface::msg::FlagStatus>::SharedPtr pub_status_flag_;
  rclcpp::Publisher<robot_hardware_interface::msg::FlagStatus>::SharedPtr pub_status_flag_legacy_;


  rclcpp::Subscription<std_msgs::msg::        UInt8MultiArray>::SharedPtr   sub_servo_axis_;
  rclcpp::Subscription<std_msgs::msg::        Float64MultiArray>::SharedPtr sub_run_axis_;
  rclcpp::Subscription<std_msgs::msg::        Float64MultiArray>::SharedPtr sub_jog_;
  rclcpp::Subscription<std_msgs::msg::        Float64MultiArray>::SharedPtr sub_run_all_;
  rclcpp::Subscription<trajectory_msgs::msg:: JointTrajectory>::SharedPtr   sub_traj_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_connect_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disconnect_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_poll_now_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_servo_all_;

  rclcpp::Service<robot_hardware_interface::srv::ServoOnAxis>:: SharedPtr srv_servo_on_axis_;
  rclcpp::Service<robot_hardware_interface::srv::ServoOnAll>::  SharedPtr srv_servo_on_all_;
  rclcpp::Service<robot_hardware_interface::srv::Jog>::         SharedPtr srv_jog_;
  rclcpp::Service<robot_hardware_interface::srv::Home>::        SharedPtr srv_home_;
  rclcpp::Service<robot_hardware_interface::srv::RunAxis>::     SharedPtr srv_run_axis_;
  rclcpp::Service<robot_hardware_interface::srv::StopAxis>::    SharedPtr srv_stop_axis_;
  rclcpp::Service<robot_hardware_interface::srv::StopAll>::     SharedPtr srv_stop_all_;

  rclcpp::TimerBase::SharedPtr timer_pos_;
  rclcpp::TimerBase::SharedPtr timer_health_;
  rclcpp::TimerBase::SharedPtr timer_auto_connect_;

  std::vector<std::string> joint_names_;
  std::vector<int> axis_ids_;
  bool connected_state_{false};
  std::string status_text_state_;
  bool first_state_logged_{false};
  bool first_state_request_logged_{false};
  bool first_joint_states_logged_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotHwNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
