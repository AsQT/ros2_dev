#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <vector>

#include "robot_hardware_interface/msg/flag_status.hpp"
#include "robot_hardware_interface/srv/home.hpp"
#include "robot_hardware_interface/srv/jog.hpp"
#include "robot_hardware_interface/srv/run_axis.hpp"
#include "robot_hardware_interface/srv/servo_on_all.hpp"
#include "robot_hardware_interface/srv/servo_on_axis.hpp"
#include "robot_hardware_interface/srv/stop_all.hpp"
#include "robot_hardware_interface/srv/stop_axis.hpp"
#include "robot_hardware_interface/tcp_client.hpp"

namespace robot_hardware_interface
{

class RobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  // Tránh hide overload base on_init()
  using hardware_interface::SystemInterface::on_init;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool state_synced_{false};
  int warmup_cycles_{0};
  int warmup_cycles_cfg_{30}; 

  double write_period_s_{0.1}; 
  double cmd_eps_rad_{1e-4}; 

  int consec_read_fail_{0};
  int max_consec_read_fail_{50};

  rclcpp::Time last_write_time_{0,0,RCL_ROS_TIME};

  std::vector<double> last_sent_pos_;    

private:
  static std::vector<int> parse_csv_ints(const std::string & s);
  static std::vector<double> parse_csv_doubles(const std::string & s);

  void setup_ros_api();
  void publish_connected(bool connected);
  void publish_status_text(const std::string & text);
  void publish_status_flags(const std::vector<uint32_t> & flags);
  void update_state_from_raw(
    const std::vector<double> & pos_raw,
    const std::vector<double> & vel_raw,
    const std::vector<uint32_t> & flags);
  void on_poll_now(
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void on_servo_all(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void on_servo_on_all(
    const std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::ServoOnAll::Response> res);
  void on_servo_on_axis(
    const std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::ServoOnAxis::Response> res);
  void on_jog(
    const std::shared_ptr<robot_hardware_interface::srv::Jog::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::Jog::Response> res);
  void on_home(
    const std::shared_ptr<robot_hardware_interface::srv::Home::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::Home::Response> res);
  void on_run_axis(
    const std::shared_ptr<robot_hardware_interface::srv::RunAxis::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::RunAxis::Response> res);
  void on_stop_axis(
    const std::shared_ptr<robot_hardware_interface::srv::StopAxis::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::StopAxis::Response> res);
  void on_stop_all(
    const std::shared_ptr<robot_hardware_interface::srv::StopAll::Request> req,
    std::shared_ptr<robot_hardware_interface::srv::StopAll::Response> res);
  void on_servo_axis_topic(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void on_jog_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_run_axis_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_run_all_topic(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void on_joint_trajectory_topic(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

private:
  RobotTcpClient client_;
  bool connected_{false};

  std::string robot_ip_{"192.168.2.50"};
  int robot_port_{5000};
  int connect_timeout_ms_{2000};
  int read_timeout_ms_{50};
  int all_token_{255};
  double default_vel_deg_s_{30.0};

  std::vector<int> axis_ids_;
  std::vector<double> direction_sign_;
  std::vector<double> rad_offset_;

  std::vector<double> hw_pos_, hw_vel_;
  std::vector<double> cmd_pos_, cmd_vel_;
  //std::vector<double> hw_pos_, hw_vel_, hw_eff_;
  //std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_legacy_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_text_;
  rclcpp::Publisher<robot_hardware_interface::msg::FlagStatus>::SharedPtr pub_status_flags_;
  rclcpp::Publisher<robot_hardware_interface::msg::FlagStatus>::SharedPtr pub_status_flags_legacy_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_poll_now_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_servo_all_;
  rclcpp::Service<robot_hardware_interface::srv::ServoOnAll>::SharedPtr srv_servo_on_all_;
  rclcpp::Service<robot_hardware_interface::srv::ServoOnAxis>::SharedPtr srv_servo_on_axis_;
  rclcpp::Service<robot_hardware_interface::srv::Jog>::SharedPtr srv_jog_;
  rclcpp::Service<robot_hardware_interface::srv::Home>::SharedPtr srv_home_;
  rclcpp::Service<robot_hardware_interface::srv::RunAxis>::SharedPtr srv_run_axis_;
  rclcpp::Service<robot_hardware_interface::srv::StopAxis>::SharedPtr srv_stop_axis_;
  rclcpp::Service<robot_hardware_interface::srv::StopAll>::SharedPtr srv_stop_all_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_servo_axis_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_jog_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_run_axis_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_run_all_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_joint_trajectory_;
  rclcpp::TimerBase::SharedPtr timer_health_;
  std::string status_text_;
  bool first_state_frame_logged_{false};
};

}  // namespace robot_hardware_interface
