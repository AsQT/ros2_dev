#include "robot_gui/rviz_panel.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <QApplication>
#include <QDebug>
#include <QMenuBar>
#include <QStatusBar>
#include <QWidget>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visualization_frame.hpp"

namespace robot_gui
{
namespace
{
QString find_fixed_frame(const std::string & config_path)
{
  std::ifstream input(config_path);
  std::string line;
  while (std::getline(input, line)) {
    const auto pos = line.find("Fixed Frame:");
    if (pos == std::string::npos) {
      continue;
    }
    auto value = line.substr(pos + std::string("Fixed Frame:").size());
    const auto first = value.find_first_not_of(" \t");
    if (first == std::string::npos) {
      return {};
    }
    value.erase(0, first);
    return QString::fromStdString(value);
  }
  return {};
}
}  // namespace

RvizPanel::RvizPanel(QApplication * app, const std::shared_ptr<rclcpp::Node> & ros_node, QWidget * parent)
: app_(app), ros_node_(ros_node), parent_(parent)
{}

RvizPanel::~RvizPanel()
{
  if (frame_ != nullptr) {
    delete frame_;
    frame_ = nullptr;
  }
}

bool RvizPanel::initialize(const std::string & package_name, const std::string & relative_path)
{
  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
    config_path_ = QString::fromStdString(config_path);
    fixed_frame_ = find_fixed_frame(config_path);
    qInfo() << "RViz config path:" << config_path_;
    qInfo() << "RViz fixed frame from config:" << (fixed_frame_.isEmpty() ? "<unknown>" : fixed_frame_);
  } catch (const std::exception & exc) {
    last_error_ = QString("Cannot resolve RViz config package '%1': %2")
      .arg(QString::fromStdString(package_name), QString::fromUtf8(exc.what()));
    return false;
  }

  if (!std::filesystem::exists(config_path)) {
    last_error_ = QString("RViz config does not exist: %1").arg(QString::fromStdString(config_path));
    return false;
  }

  try {
    rviz_ros_node_ =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(ros_node_->get_name());
    frame_ = new rviz_common::VisualizationFrame(rviz_ros_node_, parent_);
    frame_->setParent(parent_);
    frame_->setWindowFlags(Qt::Widget);
    frame_->setApp(app_);
    frame_->setSplashPath("");
    frame_->initialize(rviz_ros_node_, QString::fromStdString(config_path));
    frame_->setHideButtonVisibility(false);
    if (frame_->menuBar() != nullptr) {
      frame_->menuBar()->hide();
    }
    if (frame_->statusBar() != nullptr) {
      frame_->statusBar()->hide();
    }

    frame_->show();
    return true;
  } catch (const std::exception & exc) {
    last_error_ = QString("RViz native initialization failed: %1").arg(QString::fromUtf8(exc.what()));
  } catch (...) {
    last_error_ = "RViz native initialization failed: unknown exception";
  }

  if (frame_ != nullptr) {
    delete frame_;
    frame_ = nullptr;
  }
  rviz_ros_node_.reset();
  return false;
}

QWidget * RvizPanel::widget() const
{
  return frame_;
}

QString RvizPanel::last_error() const
{
  return last_error_;
}

QString RvizPanel::config_path() const
{
  return config_path_;
}

QString RvizPanel::fixed_frame() const
{
  return fixed_frame_;
}

}  // namespace robot_gui
