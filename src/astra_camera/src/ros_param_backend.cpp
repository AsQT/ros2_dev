#include "astra_camera/ros_param_backend.h"

namespace astra_camera
{

ParametersBackend::ParametersBackend(rclcpp::Node * node)
: node_(node),
  logger_(node_->get_logger())
{
}

ParametersBackend::~ParametersBackend()
{
  if (ros_callback_) {
    // API mới: dùng handle trực tiếp
    node_->remove_on_set_parameters_callback(ros_callback_.get());
    ros_callback_.reset();
  }
}

void ParametersBackend::addOnSetParametersCallback(OnSetParametersCallbackType callback)
{
  // API mới: callback type lấy từ rclcpp::Node::OnSetParametersCallbackType
  ros_callback_ = node_->add_on_set_parameters_callback(std::move(callback));
}

}  // namespace astra_camera

