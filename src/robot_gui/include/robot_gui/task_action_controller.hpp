#pragma once

#include <memory>
#include <functional>

#include <QObject>

#include "rclcpp/rclcpp.hpp"

class QWidget;

namespace robot_gui
{

class TaskActionController : public QObject
{
public:
  TaskActionController(rclcpp::Node::SharedPtr node, QWidget * root, QObject * parent = nullptr);

  void connectUiSignals();

private:
  void configureUi();
  void updateMovePoseCartesianStyle(bool checked);
  void addPlanButtonIfMissing(const QString & object_name, const QString & tab_name);
  void addRepeatAxisSelectorIfMissing();
  void connectButton(const QString & object_name, const std::function<void()> & callback);

  void sendMovePose(bool execute);
  void sendGripper(double position, bool execute, const QString & label);
  void sendPickPlace(bool execute);
  void sendPickPlaceVision(bool execute);
  void sendDrlPickPlace(bool execute);
  void sendMovePoseRl(bool execute);
  void sendCheckerBoard(bool execute);
  void sendRepeatabilityTest(bool execute);
  void setMovePoseRlBusy(bool busy);
  void logCancelUnavailable(const QString & label);
  void appendActionLog(const QString & msg);

  rclcpp::Node::SharedPtr node_;
  QWidget * root_{nullptr};
};

}  // namespace robot_gui
