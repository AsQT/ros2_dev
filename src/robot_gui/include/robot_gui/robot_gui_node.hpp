#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "robot_hardware_interface/msg/flag_status.hpp"
#include "robot_hardware_interface/srv/home.hpp"
#include "robot_hardware_interface/srv/jog.hpp"
#include "robot_hardware_interface/srv/run_axis.hpp"
#include "robot_hardware_interface/srv/stop_all.hpp"
#include "robot_hardware_interface/srv/stop_axis.hpp"

namespace robot_gui
{

class RobotGuiNode : public rclcpp::Node
{
public:
  using FlagCallback = std::function<void(const std::vector<uint32_t> &)>;
  using JointStateCallback = std::function<void(
    const std::vector<std::string> &,
    const std::vector<double> &,
    const std::vector<double> &)>;
  using LogCallback = std::function<void(const std::string &)>;
  using ImageCallback = std::function<void(const sensor_msgs::msg::Image::SharedPtr &)>;

  explicit RobotGuiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void set_flag_callback(FlagCallback callback);
  void set_joint_state_callback(JointStateCallback callback);
  void set_log_callback(LogCallback callback);
  void set_raw_image_callback(ImageCallback callback);
  void set_detection_image_callback(ImageCallback callback);
  void set_yolo_image_callback(ImageCallback callback);

  bool embed_rviz() const;
  int initial_page() const;
  std::string rviz_config_package() const;
  std::string rviz_config_relative_path() const;
  const std::vector<std::string> & joint_names() const;
  const std::vector<int64_t> & axis_ids() const;
  const std::string & raw_image_topic() const;
  const std::string & detection_image_topic() const;
  const std::string & yolo_image_topic() const;
  bool has_robot_description_provider() const;
  bool has_node_name(const std::string & node_name) const;
  bool has_topic_name(const std::string & topic_name) const;

  void set_servo_all(bool enabled);
  void home_axis(int axis_ui);
  void stop_axis(int axis_ui);
  void stop_all();
  void jog_axis(int axis_ui, int direction, double velocity_deg_s);
  void run_axis(int axis_ui, double position_deg, double velocity_deg_s);
  void publish_gripper(double width_m);
  void publish_joint_trajectory(const std::array<double, 6> & commanded_deg);

private:
  template<typename ServiceT>
  void call_service(
    const std::string & service_name,
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request);

  int hardware_id_from_axis(int axis_ui) const;
  std::string get_string_parameter(const std::string & name, const std::string & default_value);
  void create_image_subscriptions();
  void log(const std::string & message) const;

  FlagCallback flag_callback_;
  JointStateCallback joint_state_callback_;
  LogCallback log_callback_;
  ImageCallback raw_image_callback_;
  ImageCallback detection_image_callback_;
  ImageCallback yolo_image_callback_;
  bool embed_rviz_{true};
  int initial_page_{-1};
  std::string rviz_config_package_{"robot_moveit"};
  std::string rviz_config_relative_path_{"config/moveit.rviz"};
  std::string raw_image_topic_;
  std::string detection_image_topic_;
  std::string yolo_image_topic_;
  std::vector<std::string> joint_names_;
  std::vector<int64_t> axis_ids_;

  rclcpp::Subscription<robot_hardware_interface::msg::FlagStatus>::SharedPtr flags_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr detection_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr yolo_image_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr servo_all_client_;
  rclcpp::Client<robot_hardware_interface::srv::Home>::SharedPtr home_client_;
  rclcpp::Client<robot_hardware_interface::srv::Jog>::SharedPtr jog_client_;
  rclcpp::Client<robot_hardware_interface::srv::RunAxis>::SharedPtr run_axis_client_;
  rclcpp::Client<robot_hardware_interface::srv::StopAxis>::SharedPtr stop_axis_client_;
  rclcpp::Client<robot_hardware_interface::srv::StopAll>::SharedPtr stop_all_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
};

}  // namespace robot_gui
