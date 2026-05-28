#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace astra_camera
{

class ParametersBackend
{
public:
  // Dùng alias của rclcpp::Node thay vì NodeParametersInterface
  using OnSetParametersCallbackType   = rclcpp::Node::OnSetParametersCallbackType;
  using OnSetParametersCallbackHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;

  explicit ParametersBackend(rclcpp::Node * node);
  ~ParametersBackend();

  void addOnSetParametersCallback(OnSetParametersCallbackType callback);

private:
  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  std::shared_ptr<OnSetParametersCallbackHandle> ros_callback_;
};

}  // namespace astra_camera

