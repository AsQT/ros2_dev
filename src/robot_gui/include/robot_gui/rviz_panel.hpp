#pragma once

#include <memory>
#include <string>

#include <QString>

class QApplication;
class QWidget;
namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace rviz_common
{
class VisualizationFrame;
namespace ros_integration
{
class RosNodeAbstraction;
}  // namespace ros_integration
}  // namespace rviz_common

namespace robot_gui
{

class RvizPanel
{
public:
  RvizPanel(QApplication * app, const std::shared_ptr<rclcpp::Node> & ros_node, QWidget * parent);
  ~RvizPanel();

  bool initialize(const std::string & package_name, const std::string & relative_path);
  QWidget * widget() const;
  QString last_error() const;
  QString config_path() const;
  QString fixed_frame() const;

private:
  QApplication * app_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  QWidget * parent_{nullptr};
  rviz_common::VisualizationFrame * frame_{nullptr};
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_;
  QString last_error_;
  QString config_path_;
  QString fixed_frame_;
};

}  // namespace robot_gui
