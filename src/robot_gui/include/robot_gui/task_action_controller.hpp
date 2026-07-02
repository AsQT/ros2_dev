#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

#include <QObject>
#include <QString>

#include "rclcpp/rclcpp.hpp"

#include "robot_gui/robot_gui_node.hpp"
#include "robot_task_manager/action/move_pose_rl.hpp"

class QLineEdit;
class QTimer;
class QWidget;

namespace robot_gui
{

class TaskActionController : public QObject
{
public:
  TaskActionController(
    std::shared_ptr<RobotGuiNode> node, QWidget * root, QObject * parent = nullptr);

  void connectUiSignals();

  // Public so the free-function sendGoal<ActionT>() helper (translation-unit
  // local, not a member) can register/clear a cancel handler for the goal it
  // just sent, and so Stop buttons can invoke it. Thread-safe: goal
  // response/result callbacks run on the ROS executor thread, not the Qt
  // GUI thread.
  void registerCancelHandle(const QString & slot_key, std::function<void()> canceller);
  void clearCancelHandle(const QString & slot_key);
  void appendActionLog(const QString & msg);

private:
  void configureUi();
  void addPlanButtonIfMissing(const QString & object_name, const QString & tab_name);
  void setupLogToggle(const QString & object_name);
  void setupVisionObstacleToggle(const QString & object_name);
  void connectButton(const QString & object_name, const std::function<void()> & callback);
  void requestCancel(const QString & slot_key, const QString & label);
  bool isLogEnabled(const QString & toggle_object_name) const;
  bool isVisionObstacleEnabled() const;

  void sendMovePose(bool execute);
  void sendMoveToPoseObstacle(bool execute);
  void sendGoHome();
  void sendGripper(double position, bool execute, const QString & label);
  void sendPickPlace(bool execute);
  void sendPickPlaceVision(bool execute);
  void sendDrlPickPlace(bool execute);
  void sendMovePoseRl(bool execute);
  void dispatchMovePoseRlGoal(
    const robot_task_manager::action::MovePoseRl::Goal & goal, bool execute);
  void sendCheckerBoard(bool execute);
  void sendRepeatabilityTest(bool execute);
  void setMovePoseRlBusy(bool busy);

  // Auto Plan (codex.md "Auto Planning cho tab Move Pose RL"): periodically
  // fires plan-only MovePoseRL goals so RViz shows the RL trajectory adapting
  // to the live scene. Never executes the robot; skips a tick if the previous
  // plan is still running; auto-disables on Stop/Execute/tab-change.
  void setupAutoPlan();
  void setAutoPlanEnabled(bool on, const QString & reason);
  void onAutoPlanTick();
  bool readMovePoseRlTargetMeters(double & x, double & y, double & z) const;

  // Wood Target mode (codex.md "wood detection làm target"): when enabled, the
  // MovePoseRL target comes from the highest-confidence /vision/wood_objects
  // detection (transformed to base_link) instead of the manual X/Y/Z fields.
  void setupWoodTarget();
  // Fills roll/pitch/yaw (deg) for the MovePoseRL target orientation: user
  // values if the RPY fields have text, else default R=180/P=0/Y=90. Returns
  // false only on invalid numeric input.
  bool movePoseRlOrientationDeg(double & roll_deg, double & pitch_deg, double & yaw_deg);
  // Resolves the target position (metres, base_link) per the current Wood
  // Target mode. `source` is set to "vision_wood" or "manual_gui". Enforces the
  // trained workspace when using wood target or when `for_auto_plan`. Returns
  // false (goal must NOT be sent) after logging the reason.
  bool movePoseRlTargetPosition(
    bool for_auto_plan, double & x, double & y, double & z, QString & source);
  std::optional<double> readMmAsMeter(
    QLineEdit * edit,
    double default_mm,
    const QString & field_name);
  std::optional<double> readVelocityScale(
    QLineEdit * edit,
    double default_value,
    const QString & field_name);

  std::shared_ptr<RobotGuiNode> node_;
  QWidget * root_{nullptr};
  std::mutex cancel_mutex_;
  std::map<QString, std::function<void()>> cancel_handlers_;

  QTimer * auto_plan_timer_{nullptr};
  std::atomic<bool> move_pose_rl_busy_{false};
  int auto_plan_cycle_{0};
  bool wood_target_enabled_{false};
};

}  // namespace robot_gui
