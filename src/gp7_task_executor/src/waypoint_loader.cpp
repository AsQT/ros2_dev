#include "gp7_task_executor/waypoint_loader.h"

#include <cmath>
#include "yaml-cpp/yaml.h"

namespace gp7_task_executor
{

void WaypointLoader::rpy_to_quaternion(const double roll, const double pitch, const double yaw,
                                      double& qx, double& qy, double& qz, double& qw)
{
  const double cy = std::cos(yaw   * 0.5);
  const double sy = std::sin(yaw   * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll  * 0.5);
  const double sr = std::sin(roll  * 0.5);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;

  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;
}

WaypointLoader::WaypointLoader(
    rclcpp::Node* node,
    const std::string& waypoints_config_path,
    const std::string& cartesian_points_config_path,
    const std::string& pose_waypoints_config_path,
    const std::vector<std::string>& joint_names,
    const geometry_msgs::msg::Quaternion cartesian_quat)
  : node_(node)
  , waypoints_config_path_(waypoints_config_path)
  , cartesian_points_config_path_(cartesian_points_config_path)
  , joint_names_(joint_names)
  , cartesian_quat_(cartesian_quat)
  , pose_waypoints_config_path_(pose_waypoints_config_path)
{
}

bool WaypointLoader::load()
{
  load_joint_targets();
  load_cartesian_points();
  load_pose_targets();
  return true;
}

bool WaypointLoader::has_joint_target(const std::string& name) const
{
  return named_joint_targets_.count(name) > 0;
}

bool WaypointLoader::has_cartesian_point(const std::string& name) const
{
  return named_cartesian_points_.count(name) > 0;
}

bool WaypointLoader::has_pose_target(const std::string& name) const
{
  return named_pose_targets_.count(name) > 0;
}

const std::vector<double>* WaypointLoader::get_joint_target(const std::string& name) const
{
  auto it = named_joint_targets_.find(name);
  if (it == named_joint_targets_.end()) return nullptr;
  return &it->second;
}

const geometry_msgs::msg::PoseStamped*
WaypointLoader::get_cartesian_point(const std::string& name) const
{
  auto it = named_cartesian_points_.find(name);
  if (it == named_cartesian_points_.end()) return nullptr;
  return &it->second;
}

const geometry_msgs::msg::PoseStamped*
WaypointLoader::get_pose_target(const std::string& name) const
{
  auto it = named_pose_targets_.find(name);
  if (it == named_pose_targets_.end()) return nullptr;
  return &it->second;
}

std::string WaypointLoader::available_joint_target_names() const
{
  if (named_joint_targets_.empty()) return "(none loaded)";
  std::string names;
  for (auto it = named_joint_targets_.begin(); it != named_joint_targets_.end(); ++it)
  {
    if (!names.empty()) names += ", ";
    names += "'" + it->first + "'";
  }
  return names;
}

std::string WaypointLoader::available_cartesian_point_names() const
{
  if (named_cartesian_points_.empty()) return "(none loaded)";
  std::string names;
  for (auto it = named_cartesian_points_.begin(); it != named_cartesian_points_.end(); ++it)
  {
    if (!names.empty()) names += ", ";
    names += "'" + it->first + "'";
  }
  return names;
}

std::string WaypointLoader::available_pose_target_names() const
{
  if (named_pose_targets_.empty()) return "(none loaded)";
  std::string names;
  for (auto it = named_pose_targets_.begin(); it != named_pose_targets_.end(); ++it)
  {
    if (!names.empty()) names += ", ";
    names += "'" + it->first + "'";
  }
  return names;
}

void WaypointLoader::load_joint_targets()
{
  if (waypoints_config_path_.empty())
  {
    RCLCPP_WARN(logger(),
                "[WaypointLoader] waypoints_config_path is empty — no joint waypoints loaded.");
    return;
  }

  RCLCPP_INFO(logger(), "[WaypointLoader] Loading joint waypoints from: '%s'",
              waypoints_config_path_.c_str());

  try
  {
    YAML::Node root = YAML::LoadFile(waypoints_config_path_);
    if (!root["waypoints"] || !root["waypoints"].IsMap())
    {
      RCLCPP_ERROR(logger(),
                   "[WaypointLoader] YAML file '%s' is missing or has invalid 'waypoints:' map.",
                   waypoints_config_path_.c_str());
      return;
    }

    const YAML::Node& waypoints = root["waypoints"];
    named_joint_targets_.clear();

    for (const auto& wp : waypoints)
    {
      const std::string name = wp.first.as<std::string>();
      const YAML::Node& node = wp.second;

      if (!node["joints"] || !node["joints"].IsSequence())
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': 'joints' field is missing or not a sequence.",
                     name.c_str());
        continue;
      }

      const YAML::Node& joints_node = node["joints"];
      if (joints_node.size() != EXPECTED_JOINT_COUNT)
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': expected %zu joints but got %zu.",
                     name.c_str(), EXPECTED_JOINT_COUNT, joints_node.size());
        continue;
      }

      std::vector<double> rad_values;
      rad_values.reserve(EXPECTED_JOINT_COUNT);
      for (size_t i = 0; i < EXPECTED_JOINT_COUNT; ++i)
      {
        rad_values.push_back(joints_node[i].as<double>() * (M_PI / 180.0));
      }

      named_joint_targets_[name] = rad_values;
    }

    RCLCPP_INFO(logger(), "[WaypointLoader] Loaded %zu named joint waypoints.",
                named_joint_targets_.size());
    for (const auto& [name, values] : named_joint_targets_)
    {
      std::string msg = "  " + name + ": [";
      for (size_t i = 0; i < values.size(); ++i)
      {
        msg += std::to_string(values[i]) + " rad";
        if (i < values.size() - 1) msg += ", ";
      }
      msg += "]  (" + joint_names_[0];
      for (size_t i = 1; i < joint_names_.size(); ++i) msg += ", " + joint_names_[i];
      msg += ")";
      RCLCPP_INFO(logger(), "%s", msg.c_str());
    }
  }
  catch (const YAML::Exception& ex)
  {
    RCLCPP_ERROR(logger(),
                 "[WaypointLoader] Failed to parse '%s': %s",
                 waypoints_config_path_.c_str(), ex.what());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(logger(),
                 "[WaypointLoader] Unexpected error loading '%s': %s",
                 waypoints_config_path_.c_str(), ex.what());
  }
}

void WaypointLoader::load_cartesian_points()
{
  if (cartesian_points_config_path_.empty())
  {
    RCLCPP_WARN(logger(),
                "[WaypointLoader] cartesian_points_config_path is empty — no cartesian points loaded.");
    return;
  }

  RCLCPP_INFO(logger(),
              "[WaypointLoader] Loading cartesian points from: '%s'",
              cartesian_points_config_path_.c_str());

  try
  {
    YAML::Node root = YAML::LoadFile(cartesian_points_config_path_);
    if (!root["cartesian_points"] || !root["cartesian_points"].IsMap())
    {
      RCLCPP_ERROR(logger(),
                   "[WaypointLoader] YAML file '%s' is missing or has invalid 'cartesian_points:' map.",
                   cartesian_points_config_path_.c_str());
      return;
    }

    const YAML::Node& cartesian_points = root["cartesian_points"];
    named_cartesian_points_.clear();

    for (const auto& wp : cartesian_points)
    {
      const std::string name = wp.first.as<std::string>();
      const YAML::Node& node = wp.second;

      if (!node["frame_id"] || !node["frame_id"].IsScalar() ||
          node["frame_id"].as<std::string>().empty())
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': 'frame_id' is missing or empty.",
                     name.c_str());
        continue;
      }

      if (!node["xyz"] || !node["xyz"].IsSequence() || node["xyz"].size() != 3)
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': 'xyz' must be a sequence of exactly 3 values.",
                     name.c_str());
        continue;
      }

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = node_->get_clock()->now();
      pose_stamped.header.frame_id = node["frame_id"].as<std::string>();
      pose_stamped.pose.position.x = node["xyz"][0].as<double>();
      pose_stamped.pose.position.y = node["xyz"][1].as<double>();
      pose_stamped.pose.position.z = node["xyz"][2].as<double>();
      pose_stamped.pose.orientation = cartesian_quat_;

      named_cartesian_points_[name] = pose_stamped;

      RCLCPP_INFO(logger(), "[WaypointLoader] Loaded '%s' (cartesian):", name.c_str());
      RCLCPP_INFO(logger(), "  frame_id: '%s'", pose_stamped.header.frame_id.c_str());
      RCLCPP_INFO(logger(), "  xyz: (%.4f, %.4f, %.4f)",
                  pose_stamped.pose.position.x,
                  pose_stamped.pose.position.y,
                  pose_stamped.pose.position.z);
    }

    RCLCPP_INFO(logger(), "[WaypointLoader] Loaded %zu cartesian points.",
                named_cartesian_points_.size());
  }
  catch (const YAML::Exception& ex)
  {
    RCLCPP_ERROR(logger(),
                 "[WaypointLoader] Failed to parse cartesian points '%s': %s",
                 cartesian_points_config_path_.c_str(), ex.what());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(logger(),
                 "[WaypointLoader] Unexpected error loading cartesian points '%s': %s",
                 cartesian_points_config_path_.c_str(), ex.what());
  }
}

void WaypointLoader::load_pose_targets()
{
  if (pose_waypoints_config_path_.empty())
  {
    RCLCPP_WARN(logger(),
                "[WaypointLoader] pose_waypoints_config_path is empty — no pose targets loaded.");
    return;
  }

  RCLCPP_INFO(logger(),
              "[WaypointLoader] Loading pose targets from: '%s'",
              pose_waypoints_config_path_.c_str());

  try
  {
    YAML::Node root = YAML::LoadFile(pose_waypoints_config_path_);
    if (!root["pose_waypoints"] || !root["pose_waypoints"].IsMap())
    {
      RCLCPP_ERROR(logger(),
                   "[WaypointLoader] YAML '%s' is missing or has invalid 'pose_waypoints:' map.",
                   pose_waypoints_config_path_.c_str());
      return;
    }

    const YAML::Node& pose_waypoints = root["pose_waypoints"];
    named_pose_targets_.clear();

    for (const auto& wp : pose_waypoints)
    {
      const std::string name = wp.first.as<std::string>();
      const YAML::Node& node = wp.second;

      if (!node["frame_id"] || !node["frame_id"].IsScalar() ||
          node["frame_id"].as<std::string>().empty())
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': 'frame_id' is missing or empty.",
                     name.c_str());
        continue;
      }

      if (!node["position"] || !node["position"].IsSequence() || node["position"].size() != 3)
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': 'position' must be a sequence of 3 values.",
                     name.c_str());
        continue;
      }

      if (!node["rpy"] || !node["rpy"].IsSequence() || node["rpy"].size() != 3)
      {
        RCLCPP_ERROR(logger(),
                     "[WaypointLoader] Skipping '%s': 'rpy' must be a sequence of 3 values.",
                     name.c_str());
        continue;
      }

      const double roll  = node["rpy"][0].as<double>();
      const double pitch = node["rpy"][1].as<double>();
      const double yaw   = node["rpy"][2].as<double>();

      double qx, qy, qz, qw;
      rpy_to_quaternion(roll, pitch, yaw, qx, qy, qz, qw);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = node_->get_clock()->now();
      pose_stamped.header.frame_id = node["frame_id"].as<std::string>();
      pose_stamped.pose.position.x = node["position"][0].as<double>();
      pose_stamped.pose.position.y = node["position"][1].as<double>();
      pose_stamped.pose.position.z = node["position"][2].as<double>();
      pose_stamped.pose.orientation.x = qx;
      pose_stamped.pose.orientation.y = qy;
      pose_stamped.pose.orientation.z = qz;
      pose_stamped.pose.orientation.w = qw;

      named_pose_targets_[name] = pose_stamped;

      RCLCPP_INFO(logger(), "[WaypointLoader] Loaded '%s' (pose):", name.c_str());
      RCLCPP_INFO(logger(), "  frame_id: '%s'", pose_stamped.header.frame_id.c_str());
      RCLCPP_INFO(logger(), "  position: (%.4f, %.4f, %.4f)",
                  pose_stamped.pose.position.x,
                  pose_stamped.pose.position.y,
                  pose_stamped.pose.position.z);
      RCLCPP_INFO(logger(), "  rpy (deg): (%.2f, %.2f, %.2f)  quat: (%.6f, %.6f, %.6f, %.6f)",
                  roll * 180.0 / M_PI,
                  pitch * 180.0 / M_PI,
                  yaw * 180.0 / M_PI,
                  qx, qy, qz, qw);
    }

    RCLCPP_INFO(logger(), "[WaypointLoader] Loaded %zu pose targets.",
                named_pose_targets_.size());
  }
  catch (const YAML::Exception& ex)
  {
    RCLCPP_ERROR(logger(),
                 "[WaypointLoader] Failed to parse pose targets '%s': %s",
                 pose_waypoints_config_path_.c_str(), ex.what());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(logger(),
                 "[WaypointLoader] Unexpected error loading pose targets '%s': %s",
                 pose_waypoints_config_path_.c_str(), ex.what());
  }
}

}  // namespace gp7_task_executor
