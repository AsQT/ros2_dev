#include "robot_gui/robot_gui_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace robot_gui
{
namespace
{
constexpr int kJointCount = 6;
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

RobotGuiNode::RobotGuiNode(const rclcpp::NodeOptions & options)
: Node("robot_gui", options)
{
  if (has_parameter("joint_names")) {
    joint_names_ = get_parameter("joint_names").as_string_array();
  } else {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
  }
  if (has_parameter("axis_ids")) {
    axis_ids_ = get_parameter("axis_ids").as_integer_array();
  } else {
    axis_ids_ = declare_parameter<std::vector<int64_t>>("axis_ids", {0, 1, 2, 3, 4, 5});
  }
  embed_rviz_ =
    has_parameter("embed_rviz") ? get_parameter("embed_rviz").as_bool() :
    declare_parameter<bool>("embed_rviz", true);
  initial_page_ =
    has_parameter("initial_page") ? static_cast<int>(get_parameter("initial_page").as_int()) :
    declare_parameter<int>("initial_page", -1);
  rviz_config_package_ =
    has_parameter("rviz_config_package") ? get_parameter("rviz_config_package").as_string() :
    declare_parameter<std::string>("rviz_config_package", "robot_moveit");
  rviz_config_relative_path_ =
    has_parameter("rviz_config_relative_path") ?
    get_parameter("rviz_config_relative_path").as_string() :
    declare_parameter<std::string>("rviz_config_relative_path", "config/moveit.rviz");
  raw_image_topic_ = get_string_parameter("image_topics.raw", "");
  detection_image_topic_ = get_string_parameter("image_topics.detection", "");
  yolo_image_topic_ = get_string_parameter("image_topics.yolo", "");
  wood_objects_topic_ = get_string_parameter("vision_topics.wood_objects", wood_objects_topic_);
  box_objects_topic_ = get_string_parameter("vision_topics.box_objects", box_objects_topic_);
  vision_timeout_sec_ =
    has_parameter("vision_timeout_sec") ? get_parameter("vision_timeout_sec").as_double() :
    declare_parameter<double>("vision_timeout_sec", vision_timeout_sec_);
  tcp_base_frame_ =
    has_parameter("tcp_base_frame") ? get_parameter("tcp_base_frame").as_string() :
    declare_parameter<std::string>("tcp_base_frame", tcp_base_frame_);
  tcp_link_frame_ =
    has_parameter("tcp_link_frame") ? get_parameter("tcp_link_frame").as_string() :
    declare_parameter<std::string>("tcp_link_frame", tcp_link_frame_);
  move_pose_rl_wood_target_x_offset_m_ =
    has_parameter("move_pose_rl_wood_target_x_offset_m") ?
    get_parameter("move_pose_rl_wood_target_x_offset_m").as_double() :
    declare_parameter<double>("move_pose_rl_wood_target_x_offset_m", 0.0);
  move_pose_rl_wood_target_y_offset_m_ =
    has_parameter("move_pose_rl_wood_target_y_offset_m") ?
    get_parameter("move_pose_rl_wood_target_y_offset_m").as_double() :
    declare_parameter<double>("move_pose_rl_wood_target_y_offset_m", 0.0);
  move_pose_rl_wood_target_z_offset_m_ =
    has_parameter("move_pose_rl_wood_target_z_offset_m") ?
    get_parameter("move_pose_rl_wood_target_z_offset_m").as_double() :
    declare_parameter<double>("move_pose_rl_wood_target_z_offset_m", 0.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  flags_sub_ = create_subscription<robot_hardware_interface::msg::FlagStatus>(
    "/robot_hw/flags", rclcpp::SensorDataQoS(),
    [this](const robot_hardware_interface::msg::FlagStatus::SharedPtr msg) {
      std::vector<uint32_t> flags;
      flags.reserve(msg->axes.size());
      for (const auto & axis : msg->axes) {
        flags.push_back(axis.status_f);
      }
      if (flag_callback_) {
        flag_callback_(flags);
      }
    });

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (joint_state_callback_) {
        joint_state_callback_(msg->name, msg->position, msg->velocity);
      }
    });
  create_image_subscriptions();
  create_vision_object_subscriptions();

  servo_all_client_ = create_client<std_srvs::srv::SetBool>("/robot_hw/servo_all");
  home_client_ = create_client<robot_hardware_interface::srv::Home>("/robot_hw/home");
  jog_client_ = create_client<robot_hardware_interface::srv::Jog>("/robot_hw/jog");
  run_axis_client_ = create_client<robot_hardware_interface::srv::RunAxis>("/robot_hw/run_axis");
  stop_axis_client_ = create_client<robot_hardware_interface::srv::StopAxis>("/robot_hw/stop_axis");
  stop_all_client_ = create_client<robot_hardware_interface::srv::StopAll>("/robot_hw/stop_all");
  arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", 10);
  gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/gripper_controller/joint_trajectory", 10);
}

void RobotGuiNode::set_flag_callback(FlagCallback callback)
{
  flag_callback_ = std::move(callback);
}

void RobotGuiNode::set_joint_state_callback(JointStateCallback callback)
{
  joint_state_callback_ = std::move(callback);
}

void RobotGuiNode::set_log_callback(LogCallback callback)
{
  log_callback_ = std::move(callback);
}

void RobotGuiNode::set_raw_image_callback(ImageCallback callback)
{
  raw_image_callback_ = std::move(callback);
}

void RobotGuiNode::set_detection_image_callback(ImageCallback callback)
{
  detection_image_callback_ = std::move(callback);
}

void RobotGuiNode::set_yolo_image_callback(ImageCallback callback)
{
  yolo_image_callback_ = std::move(callback);
}

bool RobotGuiNode::embed_rviz() const {return embed_rviz_;}
int RobotGuiNode::initial_page() const {return initial_page_;}
std::string RobotGuiNode::rviz_config_package() const {return rviz_config_package_;}
std::string RobotGuiNode::rviz_config_relative_path() const {return rviz_config_relative_path_;}
const std::vector<std::string> & RobotGuiNode::joint_names() const {return joint_names_;}
const std::vector<int64_t> & RobotGuiNode::axis_ids() const {return axis_ids_;}
const std::string & RobotGuiNode::raw_image_topic() const {return raw_image_topic_;}
const std::string & RobotGuiNode::detection_image_topic() const {return detection_image_topic_;}
const std::string & RobotGuiNode::yolo_image_topic() const {return yolo_image_topic_;}

bool RobotGuiNode::has_robot_description_provider() const
{
  return has_node_name("robot_state_publisher") || has_node_name("move_group");
}

bool RobotGuiNode::has_node_name(const std::string & node_name) const
{
  const auto node_names = get_node_names();
  const auto absolute_node_name = "/" + node_name;
  return std::find(node_names.begin(), node_names.end(), node_name) != node_names.end() ||
         std::find(node_names.begin(), node_names.end(), absolute_node_name) != node_names.end();
}

bool RobotGuiNode::has_topic_name(const std::string & topic_name) const
{
  const auto names_and_types = get_topic_names_and_types();
  return names_and_types.find(topic_name) != names_and_types.end();
}

void RobotGuiNode::set_servo_all(bool enabled)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  const std::string service_name = "/robot_hw/servo_all";
  if (!servo_all_client_) {
    log(service_name + " client is not available");
    return;
  }
  if (!servo_all_client_->service_is_ready()) {
    log(service_name + " service is not available");
    return;
  }
  servo_all_client_->async_send_request(
    request,
    [this, service_name, enabled](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      try {
        const auto response = future.get();
        std::ostringstream stream;
        stream << service_name << " " << (response->success ? "OK" : "FAILED")
               << " data=" << (enabled ? "true" : "false")
               << " " << response->message;
        log(stream.str());
      } catch (const std::exception & exc) {
        log(service_name + " response failed: " + exc.what());
      }
    });
}

void RobotGuiNode::home_axis(int axis_ui)
{
  auto request = std::make_shared<robot_hardware_interface::srv::Home::Request>();
  request->id = static_cast<uint8_t>(hardware_id_from_axis(axis_ui));
  call_service<robot_hardware_interface::srv::Home>("/robot_hw/home", home_client_, request);
}

void RobotGuiNode::stop_axis(int axis_ui)
{
  auto request = std::make_shared<robot_hardware_interface::srv::StopAxis::Request>();
  request->id = static_cast<uint8_t>(hardware_id_from_axis(axis_ui));
  call_service<robot_hardware_interface::srv::StopAxis>(
    "/robot_hw/stop_axis", stop_axis_client_, request);
}

void RobotGuiNode::stop_all()
{
  auto request = std::make_shared<robot_hardware_interface::srv::StopAll::Request>();
  call_service<robot_hardware_interface::srv::StopAll>("/robot_hw/stop_all", stop_all_client_, request);
}

void RobotGuiNode::jog_axis(int axis_ui, int direction, double velocity_deg_s)
{
  auto request = std::make_shared<robot_hardware_interface::srv::Jog::Request>();
  request->id = static_cast<uint8_t>(hardware_id_from_axis(axis_ui));
  request->vel = std::abs(velocity_deg_s) * kDegToRad;
  request->dir = direction >= 0 ? 1 : 0;
  call_service<robot_hardware_interface::srv::Jog>("/robot_hw/jog", jog_client_, request);
}

void RobotGuiNode::run_axis(int axis_ui, double position_deg, double velocity_deg_s)
{
  auto request = std::make_shared<robot_hardware_interface::srv::RunAxis::Request>();
  request->id = static_cast<uint8_t>(hardware_id_from_axis(axis_ui));
  request->pos = position_deg * kDegToRad;
  request->vel = std::abs(velocity_deg_s) * kDegToRad;
  call_service<robot_hardware_interface::srv::RunAxis>("/robot_hw/run_axis", run_axis_client_, request);
}

void RobotGuiNode::publish_gripper(double width_m)
{
  trajectory_msgs::msg::JointTrajectory msg;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  msg.joint_names = {"joint_gl", "joint_gr"};
  point.positions = {std::max(width_m, 0.0) / 2.0, std::max(width_m, 0.0) / 2.0};
  point.time_from_start.sec = 1;
  msg.points.push_back(point);
  gripper_pub_->publish(msg);
}

void RobotGuiNode::publish_joint_trajectory(const std::array<double, 6> & commanded_deg)
{
  trajectory_msgs::msg::JointTrajectory msg;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  msg.joint_names = joint_names_;
  if (msg.joint_names.size() > kJointCount) {
    msg.joint_names.resize(kJointCount);
  }
  while (msg.joint_names.size() < kJointCount) {
    msg.joint_names.push_back("joint_" + std::to_string(msg.joint_names.size() + 1));
  }
  for (double deg : commanded_deg) {
    point.positions.push_back(deg * kDegToRad);
  }
  point.time_from_start.sec = 2;
  msg.points.push_back(point);
  arm_pub_->publish(msg);
}

template<typename ServiceT>
void RobotGuiNode::call_service(
  const std::string & service_name,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request)
{
  if (!client) {
    log(service_name + " client is not available");
    return;
  }
  if (!client->service_is_ready()) {
    log(service_name + " service is not available");
    return;
  }
  client->async_send_request(
    request,
    [this, service_name](typename rclcpp::Client<ServiceT>::SharedFuture future) {
      try {
        const auto response = future.get();
        std::ostringstream stream;
        stream << service_name << " " << (response->ok ? "OK" : "FAILED")
               << " code=" << static_cast<int>(response->error_code)
               << " " << response->message;
        log(stream.str());
      } catch (const std::exception & exc) {
        log(service_name + " response failed: " + exc.what());
      }
    });
}

int RobotGuiNode::hardware_id_from_axis(int axis_ui) const
{
  const int index = std::clamp(axis_ui - 1, 0, kJointCount - 1);
  if (index < static_cast<int>(axis_ids_.size())) {
    return static_cast<int>(axis_ids_[index]);
  }
  return index;
}

std::string RobotGuiNode::get_string_parameter(
  const std::string & name,
  const std::string & default_value)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string();
  }
  return declare_parameter<std::string>(name, default_value);
}

void RobotGuiNode::create_image_subscriptions()
{
  if (!raw_image_topic_.empty()) {
    raw_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (raw_image_callback_) {
          raw_image_callback_(msg);
        }
      });
    RCLCPP_INFO(get_logger(), "Raw image topic: %s", raw_image_topic_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Raw image topic is not configured");
  }

  if (!detection_image_topic_.empty()) {
    detection_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      detection_image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (detection_image_callback_) {
          detection_image_callback_(msg);
        }
      });
    RCLCPP_INFO(get_logger(), "Detection image topic: %s", detection_image_topic_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Detection image topic is not configured");
  }

  if (!yolo_image_topic_.empty()) {
    yolo_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      yolo_image_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (yolo_image_callback_) {
          yolo_image_callback_(msg);
        }
      });
    RCLCPP_INFO(get_logger(), "YOLO image topic: %s", yolo_image_topic_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "YOLO image topic is not configured");
  }
}

void RobotGuiNode::create_vision_object_subscriptions()
{
  // Vision detections are published BEST_EFFORT/VOLATILE by
  // pixel_to_base_mapper_node (robot_vision_pipeline), matching the QoS
  // used by robot_task_manager's own MoveTargetRl/MoveToPoseObstacle
  // subscriptions to the same topics.
  auto vision_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  wood_sub_ = create_subscription<robot_vision_pipeline_msgs::msg::WoodArray>(
    wood_objects_topic_, vision_qos,
    [this](const robot_vision_pipeline_msgs::msg::WoodArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      latest_wood_ = *msg;
      latest_wood_stamp_ = now();
      have_wood_ = true;
    });
  box_sub_ = create_subscription<robot_vision_pipeline_msgs::msg::BoxArray>(
    box_objects_topic_, vision_qos,
    [this](const robot_vision_pipeline_msgs::msg::BoxArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      latest_box_ = *msg;
      latest_box_stamp_ = now();
      have_box_ = true;
    });
}

bool RobotGuiNode::best_wood_pose(
  const std::string & target_frame,
  geometry_msgs::msg::Pose & pose,
  float & confidence,
  std::string & error) const
{
  robot_vision_pipeline_msgs::msg::WoodArray snapshot;
  {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    if (!have_wood_) {
      error = "No wood detection received yet on " + wood_objects_topic_;
      return false;
    }
    if ((now() - latest_wood_stamp_).seconds() > vision_timeout_sec_) {
      error = "Wood detection is stale (older than " +
        std::to_string(vision_timeout_sec_) + "s)";
      return false;
    }
    snapshot = latest_wood_;
  }

  const robot_vision_pipeline_msgs::msg::Wood * best = nullptr;
  for (const auto & wood : snapshot.woods) {
    if (best == nullptr || wood.confidence > best->confidence) {
      best = &wood;
    }
  }
  if (best == nullptr) {
    error = "No Wood detection in latest " + wood_objects_topic_ + " message";
    return false;
  }

  geometry_msgs::msg::PoseStamped in;
  in.header = snapshot.header;
  in.pose = best->pose;
  if (in.header.frame_id.empty() || in.header.frame_id == target_frame) {
    pose = in.pose;
  } else {
    try {
      const auto out = tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.2));
      pose = out.pose;
    } catch (const tf2::TransformException & e) {
      error = "TF transform " + in.header.frame_id + " -> " + target_frame +
        " failed: " + e.what();
      return false;
    }
  }
  confidence = best->confidence;
  return true;
}

bool RobotGuiNode::wood_candidates_base(
  const std::string & target_frame,
  std::vector<WoodCandidate> & candidates,
  std::string & error) const
{
  candidates.clear();
  robot_vision_pipeline_msgs::msg::WoodArray snapshot;
  {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    if (!have_wood_) {
      error = "No wood detection received yet on " + wood_objects_topic_;
      return false;
    }
    if ((now() - latest_wood_stamp_).seconds() > vision_timeout_sec_) {
      error = "Wood detection is stale (older than " +
        std::to_string(vision_timeout_sec_) + "s)";
      return false;
    }
    snapshot = latest_wood_;
  }

  for (const auto & wood : snapshot.woods) {
    WoodCandidate c;
    c.wood_id = wood.wood_id;
    c.confidence = wood.confidence;
    c.valid = true;

    geometry_msgs::msg::PoseStamped in;
    in.header = snapshot.header;
    in.pose = wood.pose;
    geometry_msgs::msg::Pose pose_base;
    if (in.header.frame_id.empty() || in.header.frame_id == target_frame) {
      pose_base = in.pose;
    } else {
      try {
        const auto out = tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.2));
        pose_base = out.pose;
      } catch (const tf2::TransformException & e) {
        // Keep the candidate but mark it invalid so it is counted in
        // number_of_wood_detected yet never selected (codex.md Phase 3 step 2).
        c.valid = false;
        c.invalid_reason = std::string("tf_transform_failed: ") + e.what();
        candidates.push_back(c);
        continue;
      }
    }
    c.x = pose_base.position.x;
    c.y = pose_base.position.y;
    c.z = pose_base.position.z;
    // codex.md Phase 5: carry the detection yaw when the (transformed)
    // orientation is a real, non-identity quaternion. A plain axis-aligned bbox
    // detection is published with identity orientation (see
    // yolo_wood_center_to_world.py) -> has_yaw stays false and callers keep the
    // default downward pick orientation (NO_TARGET_YAW_AVAILABLE).
    const auto & q = pose_base.orientation;
    if (std::abs(q.x) + std::abs(q.y) + std::abs(q.z) > 1e-6) {
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
      c.has_yaw = true;
      c.yaw_rad = yaw;
    }
    candidates.push_back(c);
  }
  return true;
}

bool RobotGuiNode::best_box_pose(
  const std::string & target_frame,
  geometry_msgs::msg::Pose & pose,
  geometry_msgs::msg::Vector3 & size,
  float & confidence,
  std::string & error) const
{
  robot_vision_pipeline_msgs::msg::BoxArray snapshot;
  {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    if (!have_box_) {
      error = "No box detection received yet on " + box_objects_topic_;
      return false;
    }
    if ((now() - latest_box_stamp_).seconds() > vision_timeout_sec_) {
      error = "Box detection is stale (older than " +
        std::to_string(vision_timeout_sec_) + "s)";
      return false;
    }
    snapshot = latest_box_;
  }

  const robot_vision_pipeline_msgs::msg::Box * best = nullptr;
  for (const auto & box : snapshot.boxes) {
    if (best == nullptr || box.confidence > best->confidence) {
      best = &box;
    }
  }
  if (best == nullptr) {
    error = "No Box detection in latest " + box_objects_topic_ + " message";
    return false;
  }

  geometry_msgs::msg::PoseStamped in;
  in.header = snapshot.header;
  in.pose = best->pose;
  if (in.header.frame_id.empty() || in.header.frame_id == target_frame) {
    pose = in.pose;
  } else {
    try {
      const auto out = tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.2));
      pose = out.pose;
    } catch (const tf2::TransformException & e) {
      error = "TF transform " + in.header.frame_id + " -> " + target_frame +
        " failed: " + e.what();
      return false;
    }
  }
  size = best->size;
  confidence = best->confidence;
  return true;
}

bool RobotGuiNode::tcp_pose(TcpPose & out, std::string & error) const
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(tcp_base_frame_, tcp_link_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    error = tcp_base_frame_ + " -> " + tcp_link_frame_ + ": " + e.what();
    return false;
  }

  const auto & t = tf.transform.translation;
  out.x_mm = t.x * 1000.0;
  out.y_mm = t.y * 1000.0;
  out.z_mm = t.z * 1000.0;

  const auto & q = tf.transform.rotation;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
  constexpr double kRadToDeg = 180.0 / M_PI;
  out.roll_deg = roll * kRadToDeg;
  out.pitch_deg = pitch * kRadToDeg;
  out.yaw_deg = yaw * kRadToDeg;
  return true;
}

bool RobotGuiNode::move_pose_rl_wood_target(WoodTarget & out, std::string & error) const
{
  robot_vision_pipeline_msgs::msg::WoodArray snapshot;
  {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    if (!have_wood_) {
      error = "No wood detection received yet on " + wood_objects_topic_;
      return false;
    }
    if ((now() - latest_wood_stamp_).seconds() > vision_timeout_sec_) {
      error = "Wood detection is stale (older than " +
        std::to_string(vision_timeout_sec_) + "s)";
      return false;
    }
    snapshot = latest_wood_;
  }

  // Selection rule: highest confidence (codex.md section 6, rule 1).
  const robot_vision_pipeline_msgs::msg::Wood * best = nullptr;
  for (const auto & wood : snapshot.woods) {
    if (best == nullptr || wood.confidence > best->confidence) {
      best = &wood;
    }
  }
  if (best == nullptr) {
    error = "No Wood detection in latest " + wood_objects_topic_ + " message";
    return false;
  }

  geometry_msgs::msg::PoseStamped in;
  in.header = snapshot.header;
  in.pose = best->pose;
  geometry_msgs::msg::Pose pose_base;
  if (in.header.frame_id.empty() || in.header.frame_id == tcp_base_frame_) {
    pose_base = in.pose;
  } else {
    try {
      const auto transformed = tf_buffer_->transform(
        in, tcp_base_frame_, tf2::durationFromSec(0.2));
      pose_base = transformed.pose;
    } catch (const tf2::TransformException & e) {
      error = "TF transform " + in.header.frame_id + " -> " + tcp_base_frame_ +
        " failed: " + e.what();
      return false;
    }
  }

  out.frame_in = in.header.frame_id;
  out.wood_id = best->wood_id;
  out.confidence = best->confidence;
  out.x_offset_m = move_pose_rl_wood_target_x_offset_m_;
  out.y_offset_m = move_pose_rl_wood_target_y_offset_m_;
  out.z_offset_m = move_pose_rl_wood_target_z_offset_m_;
  out.x_m = pose_base.position.x + move_pose_rl_wood_target_x_offset_m_;
  out.y_m = pose_base.position.y + move_pose_rl_wood_target_y_offset_m_;
  out.z_m = pose_base.position.z + move_pose_rl_wood_target_z_offset_m_;
  return true;
}

void RobotGuiNode::log(const std::string & message) const
{
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  if (log_callback_) {
    log_callback_(message);
  }
}

}  // namespace robot_gui
