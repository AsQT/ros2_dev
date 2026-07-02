#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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
#include "robot_vision_pipeline_msgs/msg/box_array.hpp"
#include "robot_vision_pipeline_msgs/msg/wood_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

  // TCP pose of tcp_link expressed in base_link, ready for display.
  // Position is in millimetres, orientation is roll/pitch/yaw in degrees.
  struct TcpPose
  {
    double x_mm{0.0};
    double y_mm{0.0};
    double z_mm{0.0};
    double roll_deg{0.0};
    double pitch_deg{0.0};
    double yaw_deg{0.0};
  };

  // Looks up the base_link -> tcp_link transform and fills `out`.
  // Returns false (with `error` describing why) when the TF is unavailable;
  // callers must not use `out` in that case.
  bool tcp_pose(TcpPose & out, std::string & error) const;

  // Wood target for the Move Pose RL "Wood Target" mode: picks the
  // highest-confidence fresh /vision/wood_objects detection, transforms it into
  // base_link, and applies the configured x/y/z offsets. Returns false (with
  // `error`) when there is no fresh wood or the TF transform fails — callers
  // must NOT send a goal in that case (no silent fallback to manual target).
  struct WoodTarget
  {
    double x_m{0.0};        // base_link, offsets applied
    double y_m{0.0};
    double z_m{0.0};
    float confidence{0.0f};
    int wood_id{-1};
    std::string frame_in;   // source frame of the detection
    double x_offset_m{0.0};
    double y_offset_m{0.0};
    double z_offset_m{0.0};
  };
  bool move_pose_rl_wood_target(WoodTarget & out, std::string & error) const;

  void set_servo_all(bool enabled);
  void home_axis(int axis_ui);
  void stop_axis(int axis_ui);
  void stop_all();
  void jog_axis(int axis_ui, int direction, double velocity_deg_s);
  void run_axis(int axis_ui, double position_deg, double velocity_deg_s);
  void publish_gripper(double width_m);
  void publish_joint_trajectory(const std::array<double, 6> & commanded_deg);

  // codex2.md 5/6: PickPlace Vision / PickPlace RL pick target selection.
  // Picks the highest-confidence Wood/Box detection from the latest fresh
  // message and transforms it into target_frame via TF (no manual offset).
  // Returns false (with `error` explaining why) if there is no fresh
  // detection or the TF lookup fails — callers must not send an action
  // goal in that case.
  bool best_wood_pose(
    const std::string & target_frame,
    geometry_msgs::msg::Pose & pose,
    float & confidence,
    std::string & error) const;
  bool best_box_pose(
    const std::string & target_frame,
    geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::Vector3 & size,
    float & confidence,
    std::string & error) const;

private:
  template<typename ServiceT>
  void call_service(
    const std::string & service_name,
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request);

  int hardware_id_from_axis(int axis_ui) const;
  std::string get_string_parameter(const std::string & name, const std::string & default_value);
  void create_image_subscriptions();
  void create_vision_object_subscriptions();
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
  std::string wood_objects_topic_{"/vision/wood_objects"};
  std::string box_objects_topic_{"/vision/box_objects"};
  std::string tcp_base_frame_{"base_link"};
  std::string tcp_link_frame_{"tcp_link"};
  double move_pose_rl_wood_target_x_offset_m_{0.0};
  double move_pose_rl_wood_target_y_offset_m_{0.0};
  double move_pose_rl_wood_target_z_offset_m_{0.0};
  double vision_timeout_sec_{2.0};
  std::vector<std::string> joint_names_;
  std::vector<int64_t> axis_ids_;

  rclcpp::Subscription<robot_hardware_interface::msg::FlagStatus>::SharedPtr flags_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr detection_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr yolo_image_sub_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::WoodArray>::SharedPtr wood_sub_;
  rclcpp::Subscription<robot_vision_pipeline_msgs::msg::BoxArray>::SharedPtr box_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr servo_all_client_;
  rclcpp::Client<robot_hardware_interface::srv::Home>::SharedPtr home_client_;
  rclcpp::Client<robot_hardware_interface::srv::Jog>::SharedPtr jog_client_;
  rclcpp::Client<robot_hardware_interface::srv::RunAxis>::SharedPtr run_axis_client_;
  rclcpp::Client<robot_hardware_interface::srv::StopAxis>::SharedPtr stop_axis_client_;
  rclcpp::Client<robot_hardware_interface::srv::StopAll>::SharedPtr stop_all_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex vision_mutex_;
  robot_vision_pipeline_msgs::msg::WoodArray latest_wood_;
  rclcpp::Time latest_wood_stamp_;
  bool have_wood_{false};
  robot_vision_pipeline_msgs::msg::BoxArray latest_box_;
  rclcpp::Time latest_box_stamp_;
  bool have_box_{false};
};

}  // namespace robot_gui
