#include "robot_gui/task_action_controller.hpp"

#include <cmath>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QTextEdit>
#include <QWidget>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_task_manager/action/checker_board.hpp"
#include "robot_task_manager/action/drl_pick_place.hpp"
#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action/pick_place.hpp"
#include "robot_task_manager/action/repeatability_test.hpp"

namespace robot_gui
{
namespace
{
using CheckerBoard = robot_task_manager::action::CheckerBoard;
using DrlPickPlace = robot_task_manager::action::DrlPickPlace;
using MoveGripper = robot_task_manager::action::MoveGripper;
using MoveToPose = robot_task_manager::action::MoveToPose;
using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
using PickPlace = robot_task_manager::action::PickPlace;
using RepeatabilityTest = robot_task_manager::action::RepeatabilityTest;
using LogFn = std::function<void(const QString &)>;

constexpr double kDefaultVelocityScale = 0.5;
constexpr double kDefaultRepeatVelocityScale = 0.25;
constexpr double kDefaultOriX = 1.0;
constexpr double kDefaultOriY = 1.0;
constexpr double kDefaultOriZ = 0.0;
constexpr double kDefaultOriW = 0.0;
constexpr double kDrlRepeatOriX = 0.7071068;
constexpr double kDrlRepeatOriY = 0.7071068;
constexpr double kDrlRepeatOriZ = 0.0;
constexpr double kDrlRepeatOriW = 0.0;
constexpr double kDefaultGripperOpen = 0.048;
constexpr double kDefaultGripperClose = 0.028;
constexpr double kDefaultPickGripper = 0.01;
constexpr double kPi = 3.14159265358979323846;

QString resultCodeToString(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
  }
}

geometry_msgs::msg::Quaternion defaultQuaternion()
{
  geometry_msgs::msg::Quaternion q;
  q.x = kDefaultOriX;
  q.y = kDefaultOriY;
  q.z = kDefaultOriZ;
  q.w = kDefaultOriW;
  return q;
}

geometry_msgs::msg::Quaternion drlRepeatQuaternion()
{
  geometry_msgs::msg::Quaternion q;
  q.x = kDrlRepeatOriX;
  q.y = kDrlRepeatOriY;
  q.z = kDrlRepeatOriZ;
  q.w = kDrlRepeatOriW;
  return q;
}

geometry_msgs::msg::Quaternion rpyDegToQuaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double roll = roll_deg * kPi / 180.0;
  const double pitch = pitch_deg * kPi / 180.0;
  const double yaw = yaw_deg * kPi / 180.0;

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

QLineEdit * lineEdit(QWidget * root, const QString & name)
{
  return root == nullptr ? nullptr : root->findChild<QLineEdit *>(name);
}

bool editHasText(QWidget * root, const QString & name)
{
  auto * edit = lineEdit(root, name);
  return edit != nullptr && !edit->text().trimmed().isEmpty();
}

bool readDouble(
  QWidget * root,
  const QString & object_name,
  double default_value,
  const QString & label,
  const LogFn & log,
  double * value)
{
  auto * edit = lineEdit(root, object_name);
  if (edit == nullptr || edit->text().trimmed().isEmpty()) {
    *value = default_value;
    return true;
  }

  bool ok = false;
  const double parsed = edit->text().trimmed().toDouble(&ok);
  if (!ok || !std::isfinite(parsed)) {
    log(QString("%1 không hợp lệ: '%2'.").arg(label, edit->text()));
    return false;
  }
  *value = parsed;
  return true;
}

bool readVelocity(
  QWidget * root,
  const QString & object_name,
  double default_value,
  const QString & label,
  const LogFn & log,
  double * value)
{
  if (!readDouble(root, object_name, default_value, label, log, value)) {
    return false;
  }
  if (*value <= 0.0 || *value > 1.0) {
    log(QString("%1 phải nằm trong (0, 1].").arg(label));
    return false;
  }
  return true;
}

bool readNonNegative(
  QWidget * root,
  const QString & object_name,
  double default_value,
  const QString & label,
  const LogFn & log,
  double * value)
{
  if (!readDouble(root, object_name, default_value, label, log, value)) {
    return false;
  }
  if (*value < 0.0) {
    log(QString("%1 phải >= 0.").arg(label));
    return false;
  }
  return true;
}

bool readNonZero(
  QWidget * root,
  const QString & object_name,
  double default_value,
  const QString & label,
  const LogFn & log,
  double * value)
{
  if (!readDouble(root, object_name, default_value, label, log, value)) {
    return false;
  }
  if (*value == 0.0) {
    log(QString("%1 phải khác 0.").arg(label));
    return false;
  }
  return true;
}

bool readPositiveInt(
  QWidget * root,
  const QString & object_name,
  int default_value,
  const QString & label,
  const LogFn & log,
  int * value)
{
  auto * edit = lineEdit(root, object_name);
  if (edit == nullptr || edit->text().trimmed().isEmpty()) {
    *value = default_value;
    return true;
  }
  bool ok = false;
  const int parsed = edit->text().trimmed().toInt(&ok);
  if (!ok || parsed <= 0) {
    log(QString("%1 phải là số nguyên > 0.").arg(label));
    return false;
  }
  *value = parsed;
  return true;
}

geometry_msgs::msg::Quaternion orientationFromRpyFields(
  QWidget * root,
  const QString & roll_name,
  const QString & pitch_name,
  const QString & yaw_name,
  const LogFn & log,
  bool * ok)
{
  *ok = true;
  const bool has_orientation =
    editHasText(root, roll_name) || editHasText(root, pitch_name) || editHasText(root, yaw_name);
  if (!has_orientation) {
    return defaultQuaternion();
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  if (!readDouble(root, roll_name, 0.0, "Roll (deg)", log, &roll) ||
    !readDouble(root, pitch_name, 0.0, "Pitch (deg)", log, &pitch) ||
    !readDouble(root, yaw_name, 0.0, "Yaw (deg)", log, &yaw))
  {
    *ok = false;
    return defaultQuaternion();
  }
  return rpyDegToQuaternion(roll, pitch, yaw);
}

geometry_msgs::msg::Quaternion orientationFromYawField(
  QWidget * root,
  const QString & yaw_name,
  const LogFn & log,
  bool * ok)
{
  *ok = true;
  if (!editHasText(root, yaw_name)) {
    return defaultQuaternion();
  }

  double yaw = 0.0;
  if (!readDouble(root, yaw_name, 0.0, "Yaw (deg)", log, &yaw)) {
    *ok = false;
    return defaultQuaternion();
  }
  return rpyDegToQuaternion(0.0, 0.0, yaw);
}

geometry_msgs::msg::Pose makePose(
  double x,
  double y,
  double z,
  const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;
  return pose;
}

geometry_msgs::msg::PoseStamped makeStampedPose(
  const std::string & frame_id,
  const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = frame_id;
  stamped.pose = pose;
  return stamped;
}

QString feedbackString(const MoveToPose::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const MoveToPoseCartesian::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const MoveGripper::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const PickPlace::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const CheckerBoard::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const DrlPickPlace::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.current_stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const RepeatabilityTest::Feedback & feedback)
{
  return QString("feedback index=%1 step=%2")
    .arg(feedback.current_index)
    .arg(QString::fromStdString(feedback.current_step));
}

template<typename ActionT>
QString resultString(const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
{
  if (!result.result) {
    return QString("result code=%1 empty result").arg(resultCodeToString(result.code));
  }
  return QString("result code=%1 success=%2 message=%3")
    .arg(resultCodeToString(result.code))
    .arg(result.result->success ? "true" : "false")
    .arg(QString::fromStdString(result.result->message));
}

template<>
QString resultString<RepeatabilityTest>(
  const rclcpp_action::ClientGoalHandle<RepeatabilityTest>::WrappedResult & result)
{
  if (!result.result) {
    return QString("result code=%1 empty result").arg(resultCodeToString(result.code));
  }
  return QString("result code=%1 success=%2 completed=%3 message=%4")
    .arg(resultCodeToString(result.code))
    .arg(result.result->success ? "true" : "false")
    .arg(result.result->completed_count)
    .arg(QString::fromStdString(result.result->message));
}

template<>
QString resultString<DrlPickPlace>(
  const rclcpp_action::ClientGoalHandle<DrlPickPlace>::WrappedResult & result)
{
  if (!result.result) {
    return QString("result code=%1 empty result").arg(resultCodeToString(result.code));
  }
  return QString("result code=%1 success=%2 failed_stage=%3 message=%4")
    .arg(resultCodeToString(result.code))
    .arg(result.result->success ? "true" : "false")
    .arg(QString::fromStdString(result.result->failed_stage))
    .arg(QString::fromStdString(result.result->message));
}

template<typename ActionT>
void sendGoal(
  const rclcpp::Node::SharedPtr & node,
  const std::string & action_name,
  const QString & label,
  const typename ActionT::Goal & goal,
  const LogFn & log)
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using Client = rclcpp_action::Client<ActionT>;
  static std::vector<std::shared_ptr<void>> keep_clients_alive;

  auto client = rclcpp_action::create_client<ActionT>(node, action_name);
  keep_clients_alive.push_back(client);

  log(QString("%1: gửi goal tới %2 ...").arg(label, QString::fromStdString(action_name)));
  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    log(QString("%1: action server %2 không sẵn sàng sau 2s.")
      .arg(label, QString::fromStdString(action_name)));
    return;
  }

  typename Client::SendGoalOptions options;
  options.goal_response_callback =
    [label, log](const typename GoalHandle::SharedPtr & goal_handle) {
      log(goal_handle ? QString("%1: goal accepted.").arg(label) :
        QString("%1: goal rejected.").arg(label));
    };
  options.feedback_callback =
    [label, log](
      typename GoalHandle::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
      if (feedback) {
        log(QString("%1: %2").arg(label, feedbackString(*feedback)));
      }
    };
  options.result_callback =
    [label, log](const typename GoalHandle::WrappedResult & result) {
      log(QString("%1: %2").arg(label, resultString<ActionT>(result)));
    };

  client->async_send_goal(goal, options);
}

}  // namespace

TaskActionController::TaskActionController(
  rclcpp::Node::SharedPtr node,
  QWidget * root,
  QObject * parent)
: QObject(parent), node_(std::move(node)), root_(root)
{
}

void TaskActionController::connectUiSignals()
{
  configureUi();

  connectButton("btnStartTask", [this]() {sendMovePose(false);});
  connectButton("btnResetTask", [this]() {sendMovePose(true);});
  connectButton("btnStopTask", [this]() {logCancelUnavailable("Move Pose");});

  connectButton("btnRLPlan", [this]() {logMovePoseRlUnavailable();});
  connectButton("btnRLExecute", [this]() {logMovePoseRlUnavailable();});
  connectButton("btnRLStop", [this]() {logCancelUnavailable("Move Pose RL");});

  connectButton("btnTaskGripperOpen", [this]() {
    sendGripper(kDefaultGripperOpen, true, "Gripper Open");
  });
  connectButton("btnTaskGripperClose", [this]() {
    const double position = root_->findChild<QLineEdit *>("txtGripperDistance") == nullptr ?
      kDefaultGripperClose : std::numeric_limits<double>::quiet_NaN();
    sendGripper(position, true, "Gripper Close");
  });
  connectButton("btnGripperRun", [this]() {
    sendGripper(std::numeric_limits<double>::quiet_NaN(), true, "Gripper Run");
  });

  connectButton("btnPickPlacePlan", [this]() {sendPickPlace(false);});
  connectButton("btnPickPlaceStart", [this]() {sendPickPlace(true);});
  connectButton("btnPickPlaceStop", [this]() {logCancelUnavailable("Pick Place");});

  connectButton("btnPickPlaceVisionPlan", [this]() {sendPickPlaceVision(false);});
  connectButton("btnPickPlaceVisionStart", [this]() {sendPickPlaceVision(true);});
  connectButton("btnPickPlaceVisionStop", [this]() {logCancelUnavailable("Pick Place Vision");});

  connectButton("btnPickPlaceRLPlan", [this]() {sendDrlPickPlace(false);});
  connectButton("btnPickPlaceRLStart", [this]() {sendDrlPickPlace(true);});
  connectButton("btnPickPlaceRLStop", [this]() {logCancelUnavailable("Pick Place RL");});

  connectButton("btnCheckBoardPlan", [this]() {sendCheckerBoard(false);});
  connectButton("btnCheckBoardStart", [this]() {sendCheckerBoard(true);});
  connectButton("btnCheckBoardStop", [this]() {logCancelUnavailable("Check Board");});

  connectButton("btnRepeatPlan", [this]() {sendRepeatabilityTest(false);});
  connectButton("btnRepeatStart", [this]() {sendRepeatabilityTest(true);});
  connectButton("btnRepeatStop", [this]() {logCancelUnavailable("Repeatability Test");});
}

void TaskActionController::configureUi()
{
  if (auto * label = root_->findChild<QLabel *>("lblMovePoseCartesian")) {
    label->hide();
  }
  if (auto * checkbox = root_->findChild<QCheckBox *>("chkMovePoseCartesian")) {
    checkbox->setText("Move Pose Cartesian");
    checkbox->setGeometry(0, 54, 337, 30);
  }

  addPlanButtonIfMissing("btnPickPlacePlan", "tabPickPlace");
  addPlanButtonIfMissing("btnPickPlaceVisionPlan", "tabPickPlaceVision");
  addPlanButtonIfMissing("btnPickPlaceRLPlan", "tabPickPlaceRL");
  addPlanButtonIfMissing("btnCheckBoardPlan", "tabCheckBoard");
  addRepeatAxisSelectorIfMissing();
}

void TaskActionController::addPlanButtonIfMissing(const QString & object_name, const QString & tab_name)
{
  if (root_->findChild<QPushButton *>(object_name)) {
    return;
  }
  auto * tab = root_->findChild<QWidget *>(tab_name);
  if (tab == nullptr) {
    appendActionLog(QString("Không tìm thấy tab %1 để thêm nút Plan.").arg(tab_name));
    return;
  }
  auto * button = new QPushButton("Plan", tab);
  button->setObjectName(object_name);
  button->setGeometry(132, 58, 72, 34);
  if (auto * start = tab->findChild<QPushButton *>(tab_name == "tabCheckBoard" ?
      "btnCheckBoardStart" : tab_name == "tabPickPlaceRL" ?
      "btnPickPlaceRLStart" : tab_name == "tabPickPlaceVision" ?
      "btnPickPlaceVisionStart" : "btnPickPlaceStart")) {
    button->setFont(start->font());
    button->setStyleSheet(start->styleSheet());
  }
  button->show();
}

void TaskActionController::addRepeatAxisSelectorIfMissing()
{
  if (root_->findChild<QComboBox *>("cbRepeatAxis")) {
    return;
  }
  auto * tab = root_->findChild<QWidget *>("tabRepeatability");
  if (tab == nullptr) {
    return;
  }
  auto * label = new QLabel("axis", tab);
  label->setObjectName("lblRepeatAxis");
  label->setGeometry(8, 408, 50, 22);
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("background:transparent; color:#102d3d; border:0px;");

  auto * combo = new QComboBox(tab);
  combo->setObjectName("cbRepeatAxis");
  combo->setGeometry(58, 406, 80, 28);
  combo->addItems({"X", "Y", "Z"});
  combo->show();
  label->show();
}

void TaskActionController::connectButton(const QString & object_name, const std::function<void()> & callback)
{
  auto * button = root_->findChild<QPushButton *>(object_name);
  if (button == nullptr) {
    appendActionLog(QString("Không tìm thấy nút %1.").arg(object_name));
    return;
  }
  connect(button, &QPushButton::clicked, this, callback);
}

void TaskActionController::appendActionLog(const QString & msg)
{
  QMetaObject::invokeMethod(
    root_,
    [this, msg]() {
      const QString line = QString("[%1] %2")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
        .arg(msg);
      auto * log = root_->findChild<QTextEdit *>("txtActionLog");
      if (log == nullptr) {
        log = root_->findChild<QTextEdit *>("txtROS2Log");
      }
      if (log != nullptr) {
        log->append(line);
      }
      if (auto * short_log = root_->findChild<QLabel *>("txtMainLog")) {
        short_log->setText(line);
      }
    },
    Qt::QueuedConnection);
}

void TaskActionController::logMovePoseRlUnavailable()
{
  appendActionLog(QString::fromUtf8(
    u8"Move Pose RL action chưa có mapping backend, chưa gửi goal."));
}

void TaskActionController::logCancelUnavailable(const QString & label)
{
  appendActionLog(QString("%1: cancel chưa implement.").arg(label));
}

void TaskActionController::sendMovePose(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double x = 0.40;
  double y = 0.10;
  double z = 0.35;
  double velocity = kDefaultVelocityScale;
  if (!readDouble(root_, "txtTargetX", x, "Move Pose X", log, &x) ||
    !readDouble(root_, "txtTargetY", y, "Move Pose Y", log, &y) ||
    !readDouble(root_, "txtTargetZ", z, "Move Pose Z", log, &z) ||
    !readVelocity(root_, "", velocity, "Move Pose velocity_scale", log, &velocity))
  {
    return;
  }

  bool ok = true;
  const auto orientation =
    orientationFromRpyFields(root_, "txtTargetRoll", "txtTargetPitch", "txtTargetYaw", log, &ok);
  if (!ok) {
    return;
  }

  const bool cartesian =
    root_->findChild<QCheckBox *>("chkMovePoseCartesian") != nullptr &&
    root_->findChild<QCheckBox *>("chkMovePoseCartesian")->isChecked();

  if (cartesian) {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = makePose(x, y, z, orientation);
    goal.velocity_scale = velocity;
    goal.execute = execute;
    sendGoal<MoveToPoseCartesian>(
      node_, "/move_to_pose_cartesian",
      execute ? "Move Pose Cartesian Start" : "Move Pose Cartesian Plan", goal, log);
  } else {
    MoveToPose::Goal goal;
    goal.target_pose = makePose(x, y, z, orientation);
    goal.velocity_scale = velocity;
    goal.execute = execute;
    sendGoal<MoveToPose>(
      node_, "/move_to_pose", execute ? "Move Pose Start" : "Move Pose Plan", goal, log);
  }
}

void TaskActionController::sendGripper(double position, bool execute, const QString & label)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double target = position;
  if (!std::isfinite(target) &&
    !readNonNegative(root_, "txtGripperDistance", kDefaultGripperClose, "Gripper position", log, &target))
  {
    return;
  }

  MoveGripper::Goal goal;
  goal.position = target;
  goal.execute = execute;
  sendGoal<MoveGripper>(node_, "/move_gripper", label, goal, log);
}

void TaskActionController::sendPickPlace(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double pick_x = 0.40;
  double pick_y = 0.10;
  double pick_z = 0.25;
  double place_x = 0.30;
  double place_y = 0.00;
  double place_z = 0.25;
  double gripper = kDefaultPickGripper;
  double velocity = kDefaultVelocityScale;
  if (!readDouble(root_, "pickPoseX", pick_x, "Pick X", log, &pick_x) ||
    !readDouble(root_, "pickPoseY", pick_y, "Pick Y", log, &pick_y) ||
    !readDouble(root_, "pickPoseZ", pick_z, "Pick Z", log, &pick_z) ||
    !readDouble(root_, "placePoseX", place_x, "Place X", log, &place_x) ||
    !readDouble(root_, "placePoseY", place_y, "Place Y", log, &place_y) ||
    !readDouble(root_, "placePoseZ", place_z, "Place Z", log, &place_z) ||
    !readNonNegative(root_, "", gripper, "PickPlace gripper", log, &gripper) ||
    !readVelocity(root_, "", velocity, "PickPlace velocity_scale", log, &velocity))
  {
    return;
  }

  bool ok = true;
  const auto pick_q = orientationFromYawField(root_, "pickPoseYaw", log, &ok);
  if (!ok) {
    return;
  }
  const auto place_q = orientationFromYawField(root_, "placePoseYaw", log, &ok);
  if (!ok) {
    return;
  }

  PickPlace::Goal goal;
  goal.pose_pick = makePose(pick_x, pick_y, pick_z, pick_q);
  goal.pose_place = makePose(place_x, place_y, place_z, place_q);
  goal.gripper = gripper;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<PickPlace>(node_, "/pickplace", execute ? "Pick Place Start" : "Pick Place Plan", goal, log);
}

void TaskActionController::sendPickPlaceVision(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  if (!editHasText(root_, "txtObjectX") && !editHasText(root_, "txtObjectY") &&
    !editHasText(root_, "txtObjectZ"))
  {
    appendActionLog(QString::fromUtf8(
      u8"Vision pose chưa có dữ liệu, dùng pose nhập tay hoặc không gửi goal."));
  }

  double pick_x = 0.40;
  double pick_y = 0.10;
  double pick_z = 0.25;
  double place_x = 0.30;
  double place_y = 0.00;
  double place_z = 0.25;
  double velocity = kDefaultVelocityScale;
  if (!readDouble(root_, "txtObjectX", pick_x, "Vision pick X", log, &pick_x) ||
    !readDouble(root_, "txtObjectY", pick_y, "Vision pick Y", log, &pick_y) ||
    !readDouble(root_, "txtObjectZ", pick_z, "Vision pick Z", log, &pick_z) ||
    !readDouble(root_, "visionPlacePoseX", place_x, "Vision place X", log, &place_x) ||
    !readDouble(root_, "visionPlacePoseY", place_y, "Vision place Y", log, &place_y) ||
    !readDouble(root_, "visionPlacePoseZ", place_z, "Vision place Z", log, &place_z) ||
    !readVelocity(root_, "", velocity, "PickPlace Vision velocity_scale", log, &velocity))
  {
    return;
  }

  bool ok = true;
  const auto place_q = orientationFromYawField(root_, "visionPlacePoseYaw", log, &ok);
  if (!ok) {
    return;
  }

  PickPlace::Goal goal;
  goal.pose_pick = makePose(pick_x, pick_y, pick_z, defaultQuaternion());
  goal.pose_place = makePose(place_x, place_y, place_z, place_q);
  goal.gripper = kDefaultPickGripper;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<PickPlace>(
    node_, "/pickplace", execute ? "Pick Place Vision Start" : "Pick Place Vision Plan", goal, log);
}

void TaskActionController::sendDrlPickPlace(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double place_x = 0.34;
  double place_y = -0.10;
  double place_z = 0.08;
  double gripper = kDefaultGripperClose;
  if (!readDouble(root_, "rlPlacePoseX", place_x, "DRL place X", log, &place_x) ||
    !readDouble(root_, "rlPlacePoseY", place_y, "DRL place Y", log, &place_y) ||
    !readDouble(root_, "rlPlacePoseZ", place_z, "DRL place Z", log, &place_z) ||
    !readNonNegative(root_, "", gripper, "DRL gripper close width", log, &gripper))
  {
    return;
  }

  bool ok = true;
  auto place_q = orientationFromYawField(root_, "rlPlacePoseYaw", log, &ok);
  if (!ok) {
    return;
  }
  if (!editHasText(root_, "rlPlacePoseYaw")) {
    place_q = drlRepeatQuaternion();
  }

  DrlPickPlace::Goal goal;
  goal.target_pick = makeStampedPose(
    "base_link", makePose(0.40, 0.05, 0.08, drlRepeatQuaternion()));
  goal.target_place = makeStampedPose("base_link", makePose(place_x, place_y, place_z, place_q));
  goal.gripper_close_width_m = gripper;
  goal.execute = execute;
  sendGoal<DrlPickPlace>(
    node_, "/drl_pickplace", execute ? "Pick Place RL Start" : "Pick Place RL Plan", goal, log);
}

void TaskActionController::sendCheckerBoard(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double step = 0.10;
  double velocity = kDefaultVelocityScale;
  if (!readDouble(root_, "txtCheckBoardStep", step, "CheckerBoard step", log, &step) ||
    !readVelocity(root_, "", velocity, "CheckerBoard velocity_scale", log, &velocity))
  {
    return;
  }

  CheckerBoard::Goal goal;
  goal.step = step;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<CheckerBoard>(
    node_, "/move_checker_board", execute ? "Check Board Start" : "Check Board Plan", goal, log);
}

void TaskActionController::sendRepeatabilityTest(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double retract_x = 0.40;
  double retract_y = 0.00;
  double retract_z = 0.18;
  double disturb1_x = 0.35;
  double disturb1_y = -0.08;
  double disturb1_z = 0.18;
  double disturb2_x = 0.45;
  double disturb2_y = 0.08;
  double disturb2_z = 0.18;
  double offset = 0.02;
  double velocity = kDefaultRepeatVelocityScale;
  int repeat_count = 3;

  if (!readDouble(root_, "repeatRetractX", retract_x, "Retract X", log, &retract_x) ||
    !readDouble(root_, "repeatRetractY", retract_y, "Retract Y", log, &retract_y) ||
    !readDouble(root_, "repeatRetractZ", retract_z, "Retract Z", log, &retract_z) ||
    !readDouble(root_, "repeatDisturb1X", disturb1_x, "Disturb1 X", log, &disturb1_x) ||
    !readDouble(root_, "repeatDisturb1Y", disturb1_y, "Disturb1 Y", log, &disturb1_y) ||
    !readDouble(root_, "repeatDisturb1Z", disturb1_z, "Disturb1 Z", log, &disturb1_z) ||
    !readDouble(root_, "repeatDisturb2X", disturb2_x, "Disturb2 X", log, &disturb2_x) ||
    !readDouble(root_, "repeatDisturb2Y", disturb2_y, "Disturb2 Y", log, &disturb2_y) ||
    !readDouble(root_, "repeatDisturb2Z", disturb2_z, "Disturb2 Z", log, &disturb2_z) ||
    !readNonZero(root_, "txtMeasOffset", offset, "meas_offset", log, &offset) ||
    !readPositiveInt(root_, "txtRepeatCount", repeat_count, "repeat_count", log, &repeat_count) ||
    !readVelocity(root_, "", velocity, "Repeatability velocity_scale", log, &velocity))
  {
    return;
  }

  uint8_t axis = RepeatabilityTest::Goal::AXIS_X;
  if (auto * combo = root_->findChild<QComboBox *>("cbRepeatAxis")) {
    const QString axis_text = combo->currentText().trimmed().toUpper();
    if (axis_text == "Y") {
      axis = RepeatabilityTest::Goal::AXIS_Y;
    } else if (axis_text == "Z") {
      axis = RepeatabilityTest::Goal::AXIS_Z;
    }
  }

  const auto q = drlRepeatQuaternion();
  RepeatabilityTest::Goal goal;
  goal.retract_pose = makeStampedPose("world", makePose(retract_x, retract_y, retract_z, q));
  goal.disturb_pose_1 = makeStampedPose("world", makePose(disturb1_x, disturb1_y, disturb1_z, q));
  goal.disturb_pose_2 = makeStampedPose("world", makePose(disturb2_x, disturb2_y, disturb2_z, q));
  goal.axis = axis;
  goal.meas_offset = offset;
  goal.repeat_count = repeat_count;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<RepeatabilityTest>(
    node_, "/repeatability_test",
    execute ? "Repeatability Test Start" : "Repeatability Test Plan", goal, log);
}

}  // namespace robot_gui
