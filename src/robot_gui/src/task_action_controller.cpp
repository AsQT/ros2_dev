#include "robot_gui/task_action_controller.hpp"

#include <cmath>
#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QFrame>
#include <QFont>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QWidget>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_task_manager/action/checker_board.hpp"
#include "robot_task_manager/action/drl_pick_place.hpp"
#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/action/move_pose_rl.hpp"
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
using MovePoseRl = robot_task_manager::action::MovePoseRl;
using MoveToPose = robot_task_manager::action::MoveToPose;
using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
using PickPlace = robot_task_manager::action::PickPlace;
using RepeatabilityTest = robot_task_manager::action::RepeatabilityTest;
using LogFn = std::function<void(const QString &)>;

constexpr double kDefaultVelocityScale = 0.1;
constexpr double kDefaultRepeatVelocityScale = 0.15;
constexpr double kDefaultOriX = 1.0;
constexpr double kDefaultOriY = 1.0;
constexpr double kDefaultOriZ = 0.0;
constexpr double kDefaultOriW = 0.0;
constexpr double kDrlRepeatOriX = 1.0;
constexpr double kDrlRepeatOriY = 1.0;
constexpr double kDrlRepeatOriZ = 0.0;
constexpr double kDrlRepeatOriW = 0.0;
constexpr double kDefaultMoveXMm = 400.0;
constexpr double kDefaultMoveYMm = 100.0;
constexpr double kDefaultMoveZMm = 350.0;
constexpr double kDefaultPickXMm = 400.0;
constexpr double kDefaultPickYMm = 100.0;
constexpr double kDefaultPickZMm = 250.0;
constexpr double kDefaultPlaceXMm = 300.0;
constexpr double kDefaultPlaceYMm = 0.0;
constexpr double kDefaultPlaceZMm = 250.0;
constexpr double kDefaultGripperOpenMm = 48.0;
constexpr double kDefaultGripperCloseMm = 28.0;
constexpr double kDefaultPickGripperMm = 10.0;
constexpr double kDefaultCheckerStepMm = 100.0;
constexpr double kDefaultRepeatRetractXMm = 350.0;
constexpr double kDefaultRepeatRetractYMm = 0.0;
constexpr double kDefaultRepeatRetractZMm = 100.0;
constexpr double kDefaultRepeatDisturb1XMm = 320.0;
constexpr double kDefaultRepeatDisturb1YMm = -50.0;
constexpr double kDefaultRepeatDisturb1ZMm = 120.0;
constexpr double kDefaultRepeatMeasOffsetMm = 50.0;
constexpr double kDefaultDrlPickXMm = 400.0;
constexpr double kDefaultDrlPickYMm = 50.0;
constexpr double kDefaultDrlPickZMm = 80.0;
constexpr double kDefaultDrlPlaceXMm = 340.0;
constexpr double kDefaultDrlPlaceYMm = -100.0;
constexpr double kDefaultDrlPlaceZMm = 80.0;
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

QString feedbackString(const MovePoseRl::Feedback & feedback)
{
  return QString("[MovePoseRL] %1 | %2%")
    .arg(QString::fromStdString(feedback.current_stage))
    .arg(feedback.progress, 0, 'f', 1);
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

template<>
QString resultString<MovePoseRl>(
  const rclcpp_action::ClientGoalHandle<MovePoseRl>::WrappedResult & result)
{
  if (!result.result) {
    return QString("[MovePoseRL] FAILED at result: empty result, code=%1")
      .arg(resultCodeToString(result.code));
  }
  if (result.result->success && result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return QString("[MovePoseRL] SUCCESS: %1")
      .arg(QString::fromStdString(result.result->message));
  }
  return QString("[MovePoseRL] FAILED at %1: %2")
    .arg(QString::fromStdString(result.result->failed_stage))
    .arg(QString::fromStdString(result.result->message));
}

bool serviceAvailable(
  const rclcpp::Node::SharedPtr & node,
  const std::string & service_name,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto services = node->get_service_names_and_types();
    if (services.find(service_name) != services.end()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  const auto services = node->get_service_names_and_types();
  return services.find(service_name) != services.end();
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

  connectButton("btnRLPlan", [this]() {sendMovePoseRl(false);});
  connectButton("btnRLExecute", [this]() {sendMovePoseRl(true);});
  connectButton("btnRLStop", [this]() {logCancelUnavailable("Move Pose RL");});

  connectButton("btnTaskGripperOpen", [this]() {
    sendGripper(kDefaultGripperOpenMm, true, "Gripper Open");
  });
  connectButton("btnTaskGripperClose", [this]() {
    const double position = root_->findChild<QLineEdit *>("txtGripperDistance") == nullptr ?
      kDefaultGripperCloseMm : std::numeric_limits<double>::quiet_NaN();
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
  auto * action_log = root_->findChild<QPlainTextEdit *>("txtActionLog");
  if (action_log == nullptr) {
    auto * log_frame = root_->findChild<QFrame *>("ShortLogArea");
    auto * old_large_label = root_->findChild<QLabel *>("txtMainLog");
    if (log_frame != nullptr && old_large_label != nullptr) {
      action_log = new QPlainTextEdit(log_frame);
      action_log->setObjectName("txtActionLog");
      action_log->setGeometry(old_large_label->geometry());
      action_log->show();
      old_large_label->hide();
    }
  }
  if (action_log != nullptr) {
    action_log->setReadOnly(true);
    action_log->setLineWrapMode(QPlainTextEdit::NoWrap);
    QFont log_font("DejaVu Sans Mono");
    log_font.setPointSize(8);
    action_log->setFont(log_font);
    action_log->setStyleSheet(
      "QPlainTextEdit#txtActionLog {"
      "font-family: 'DejaVu Sans Mono';"
      "font-size: 8pt;"
      "color: #111111;"
      "background-color: white;"
      "border: 1px solid #b8c6d1;"
      "border-radius: 5px;"
      "padding: 4px;"
      "}");
    action_log->setPlainText("[Action Log] ready");
  } else if (auto * fallback_label = root_->findChild<QLabel *>("txtMainLog")) {
    fallback_label->setWordWrap(true);
    fallback_label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    QFont log_font("DejaVu Sans Mono");
    log_font.setPointSize(8);
    fallback_label->setFont(log_font);
    fallback_label->setStyleSheet(
      "QLabel#txtMainLog {"
      "font-family: 'DejaVu Sans Mono';"
      "font-size: 8pt;"
      "color: #111111;"
      "background-color: white;"
      "padding: 6px;"
      "}");
  }

  if (auto * label = root_->findChild<QLabel *>("lblMovePoseCartesian")) {
    label->hide();
  }
  if (auto * checkbox = root_->findChild<QCheckBox *>("chkMovePoseCartesian")) {
    checkbox->setText("Move Pose Cartesian");
    checkbox->setGeometry(0, 54, 337, 30);
    updateMovePoseCartesianStyle(checkbox->isChecked());
    connect(checkbox, &QCheckBox::toggled, this, [this](bool checked) {
      updateMovePoseCartesianStyle(checked);
    });
  }

  addPlanButtonIfMissing("btnPickPlacePlan", "tabPickPlace");
  addPlanButtonIfMissing("btnPickPlaceVisionPlan", "tabPickPlaceVision");
  addPlanButtonIfMissing("btnPickPlaceRLPlan", "tabPickPlaceRL");
  addPlanButtonIfMissing("btnCheckBoardPlan", "tabCheckBoard");
  addRepeatAxisSelectorIfMissing();
}

void TaskActionController::updateMovePoseCartesianStyle(bool checked)
{
  auto * checkbox = root_->findChild<QCheckBox *>("chkMovePoseCartesian");
  if (checkbox == nullptr) {
    return;
  }

  if (checked) {
    checkbox->setStyleSheet(
      "QCheckBox {"
      "color: #005A70;"
      "font-weight: 600;"
      "background-color: #D9F4F7;"
      "border: 1px solid #01BABE;"
      "border-radius: 4px;"
      "padding: 3px 6px;"
      "spacing: 8px;"
      "}"
      "QCheckBox::indicator {"
      "width: 18px;"
      "height: 18px;"
      "border: 1px solid #01BABE;"
      "border-radius: 3px;"
      "background-color: #01BABE;"
      "}");
    return;
  }

  checkbox->setStyleSheet(
    "QCheckBox {"
    "color: #333333;"
    "background-color: transparent;"
    "border: 1px solid transparent;"
    "border-radius: 4px;"
    "padding: 3px 6px;"
    "spacing: 8px;"
    "}"
    "QCheckBox::indicator {"
    "width: 18px;"
    "height: 18px;"
    "border: 1px solid #b8cfd8;"
    "border-radius: 3px;"
    "background-color: white;"
    "}");
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
      if (auto * action_log = root_->findChild<QPlainTextEdit *>("txtActionLog")) {
        action_log->appendPlainText(line);
        return;
      }
      if (auto * log = root_->findChild<QTextEdit *>("txtROS2Log")) {
        log->append(line);
      }
      if (auto * short_log = root_->findChild<QLabel *>("txtMainLog")) {
        short_log->setText(line);
      }
    },
    Qt::QueuedConnection);
}

std::optional<double> TaskActionController::readMmAsMeter(
  QLineEdit * edit,
  double default_mm,
  const QString & field_name)
{
  double value_mm = default_mm;
  if (edit != nullptr && !edit->text().trimmed().isEmpty()) {
    bool ok = false;
    value_mm = edit->text().trimmed().toDouble(&ok);
    if (!ok || !std::isfinite(value_mm)) {
      appendActionLog(QString("[Input Error] %1 invalid mm value").arg(field_name));
      return std::nullopt;
    }
  }

  const double value_m = value_mm / 1000.0;
  appendActionLog(QString("%1=%2 mm -> %3 m")
    .arg(field_name)
    .arg(value_mm, 0, 'f', 3)
    .arg(value_m, 0, 'f', 3));
  return value_m;
}

void TaskActionController::setMovePoseRlBusy(bool busy)
{
  QMetaObject::invokeMethod(
    root_,
    [this, busy]() {
      if (auto * plan = root_->findChild<QPushButton *>("btnRLPlan")) {
        plan->setEnabled(!busy);
      }
      if (auto * execute = root_->findChild<QPushButton *>("btnRLExecute")) {
        execute->setEnabled(!busy);
      }
    },
    Qt::QueuedConnection);
}

void TaskActionController::logCancelUnavailable(const QString & label)
{
  appendActionLog(QString("%1: cancel chưa implement.").arg(label));
}

void TaskActionController::sendMovePose(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double velocity = kDefaultVelocityScale;
  const auto x = readMmAsMeter(lineEdit(root_, "txtTargetX"), kDefaultMoveXMm, "[Move Pose] X");
  const auto y = readMmAsMeter(lineEdit(root_, "txtTargetY"), kDefaultMoveYMm, "[Move Pose] Y");
  const auto z = readMmAsMeter(lineEdit(root_, "txtTargetZ"), kDefaultMoveZMm, "[Move Pose] Z");
  if (!x || !y || !z ||
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
    goal.target_pose = makePose(*x, *y, *z, orientation);
    goal.velocity_scale = velocity;
    goal.execute = execute;
    sendGoal<MoveToPoseCartesian>(
      node_, "/move_to_pose_cartesian",
      execute ? "Move Pose Cartesian Start" : "Move Pose Cartesian Plan", goal, log);
  } else {
    MoveToPose::Goal goal;
    goal.target_pose = makePose(*x, *y, *z, orientation);
    goal.velocity_scale = velocity;
    goal.execute = execute;
    sendGoal<MoveToPose>(
      node_, "/move_to_pose", execute ? "Move Pose Start" : "Move Pose Plan", goal, log);
  }
}

void TaskActionController::sendGripper(double position, bool execute, const QString & label)
{
  std::optional<double> target;
  if (std::isfinite(position)) {
    target = position / 1000.0;
    appendActionLog(QString("[Gripper] width=%1 mm -> %2 m")
      .arg(position, 0, 'f', 3)
      .arg(*target, 0, 'f', 3));
  } else {
    target = readMmAsMeter(
      lineEdit(root_, "txtGripperDistance"),
      kDefaultGripperCloseMm,
      "[Gripper] width");
  }
  if (!target) {
    return;
  }
  if (*target < 0.0) {
    appendActionLog("[Input Error] [Gripper] width must be >= 0 mm");
    return;
  }

  MoveGripper::Goal goal;
  goal.position = *target;
  goal.execute = execute;
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  sendGoal<MoveGripper>(node_, "/move_gripper", label, goal, log);
}

void TaskActionController::sendPickPlace(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double velocity = kDefaultVelocityScale;
  const auto pick_x = readMmAsMeter(lineEdit(root_, "pickPoseX"), kDefaultPickXMm, "[Pick Place] pick X");
  const auto pick_y = readMmAsMeter(lineEdit(root_, "pickPoseY"), kDefaultPickYMm, "[Pick Place] pick Y");
  const auto pick_z = readMmAsMeter(lineEdit(root_, "pickPoseZ"), kDefaultPickZMm, "[Pick Place] pick Z");
  const auto place_x = readMmAsMeter(lineEdit(root_, "placePoseX"), kDefaultPlaceXMm, "[Pick Place] place X");
  const auto place_y = readMmAsMeter(lineEdit(root_, "placePoseY"), kDefaultPlaceYMm, "[Pick Place] place Y");
  const auto place_z = readMmAsMeter(lineEdit(root_, "placePoseZ"), kDefaultPlaceZMm, "[Pick Place] place Z");
  const double gripper = kDefaultPickGripperMm / 1000.0;
  appendActionLog(QString("[Pick Place] gripper=%1 mm -> %2 m")
    .arg(kDefaultPickGripperMm, 0, 'f', 3)
    .arg(gripper, 0, 'f', 3));
  if (!pick_x || !pick_y || !pick_z || !place_x || !place_y || !place_z ||
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
  goal.pose_pick = makePose(*pick_x, *pick_y, *pick_z, pick_q);
  goal.pose_place = makePose(*place_x, *place_y, *place_z, place_q);
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

  double velocity = kDefaultVelocityScale;
  const auto pick_x = readMmAsMeter(lineEdit(root_, "txtObjectX"), kDefaultPickXMm, "[Pick Place Vision] pick X");
  const auto pick_y = readMmAsMeter(lineEdit(root_, "txtObjectY"), kDefaultPickYMm, "[Pick Place Vision] pick Y");
  const auto pick_z = readMmAsMeter(lineEdit(root_, "txtObjectZ"), kDefaultPickZMm, "[Pick Place Vision] pick Z");
  const auto place_x = readMmAsMeter(lineEdit(root_, "visionPlacePoseX"), kDefaultPlaceXMm, "[Pick Place Vision] place X");
  const auto place_y = readMmAsMeter(lineEdit(root_, "visionPlacePoseY"), kDefaultPlaceYMm, "[Pick Place Vision] place Y");
  const auto place_z = readMmAsMeter(lineEdit(root_, "visionPlacePoseZ"), kDefaultPlaceZMm, "[Pick Place Vision] place Z");
  if (!pick_x || !pick_y || !pick_z || !place_x || !place_y || !place_z ||
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
  goal.pose_pick = makePose(*pick_x, *pick_y, *pick_z, defaultQuaternion());
  goal.pose_place = makePose(*place_x, *place_y, *place_z, place_q);
  goal.gripper = kDefaultPickGripperMm / 1000.0;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<PickPlace>(
    node_, "/pickplace", execute ? "Pick Place Vision Start" : "Pick Place Vision Plan", goal, log);
}

void TaskActionController::sendDrlPickPlace(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  const auto place_x = readMmAsMeter(lineEdit(root_, "rlPlacePoseX"), kDefaultDrlPlaceXMm, "[Pick Place RL] place X");
  const auto place_y = readMmAsMeter(lineEdit(root_, "rlPlacePoseY"), kDefaultDrlPlaceYMm, "[Pick Place RL] place Y");
  const auto place_z = readMmAsMeter(lineEdit(root_, "rlPlacePoseZ"), kDefaultDrlPlaceZMm, "[Pick Place RL] place Z");
  const double gripper = kDefaultGripperCloseMm / 1000.0;
  appendActionLog(QString("[Pick Place RL] gripper_close_width=%1 mm -> %2 m")
    .arg(kDefaultGripperCloseMm, 0, 'f', 3)
    .arg(gripper, 0, 'f', 3));
  if (!place_x || !place_y || !place_z) {
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
    "base_link",
    makePose(
      kDefaultDrlPickXMm / 1000.0,
      kDefaultDrlPickYMm / 1000.0,
      kDefaultDrlPickZMm / 1000.0,
      drlRepeatQuaternion()));
  goal.target_place = makeStampedPose("base_link", makePose(*place_x, *place_y, *place_z, place_q));
  goal.gripper_close_width_m = gripper;
  goal.execute = execute;
  sendGoal<DrlPickPlace>(
    node_, "/drl_pickplace", execute ? "Pick Place RL Start" : "Pick Place RL Plan", goal, log);
}

void TaskActionController::sendMovePoseRl(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double velocity = kDefaultVelocityScale;
  const auto x = readMmAsMeter(lineEdit(root_, "rlPosePositionX"), kDefaultMoveXMm, "[MovePoseRL] X");
  const auto y = readMmAsMeter(lineEdit(root_, "rlPosePositionY"), kDefaultMoveYMm, "[MovePoseRL] Y");
  const auto z = readMmAsMeter(lineEdit(root_, "rlPosePositionZ"), kDefaultMoveZMm, "[MovePoseRL] Z");
  if (!x || !y || !z ||
    !readVelocity(root_, "txtVelocityScale", velocity, "MovePoseRL velocity_scale", log, &velocity))
  {
    return;
  }

  bool ok = true;
  const auto orientation = orientationFromRpyFields(
    root_,
    "rlPoseOrientationRoll",
    "rlPoseOrientationPitch",
    "rlPoseOrientationYaw",
    log,
    &ok);
  if (!ok) {
    return;
  }

  MovePoseRl::Goal goal;
  goal.target_pose = makePose(*x, *y, *z, orientation);
  goal.velocity_scale = velocity;
  goal.execute = execute;

  appendActionLog(execute ?
    "[MovePoseRL] Sending execute goal..." :
    "[MovePoseRL] Sending plan-only goal...");
  setMovePoseRlBusy(true);

  auto node = node_;
  QPointer<TaskActionController> self(this);
  std::thread([self, node, goal, execute]() {
    using GoalHandle = rclcpp_action::ClientGoalHandle<MovePoseRl>;
    using Client = rclcpp_action::Client<MovePoseRl>;
    static std::vector<std::shared_ptr<void>> keep_clients_alive;

    if (!self) {
      return;
    }

    auto client = rclcpp_action::create_client<MovePoseRl>(node, "/move_pose_rl");
    keep_clients_alive.push_back(client);

    auto fail_preflight = [self](const QString & missing) {
      if (!self) {
        return;
      }
      self->appendActionLog(QString("[MovePoseRL] Backend not ready: missing %1.").arg(missing));
      self->setMovePoseRlBusy(false);
    };

    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
      fail_preflight("/move_pose_rl action server");
      return;
    }

    const std::vector<std::string> required_services = execute ?
      std::vector<std::string>{
        "/drl_unified_planner_node/set_parameters",
        "/drl/clear_trajectory",
        "/drl/plan",
        "/drl/execute_forward",
        "/drl/get_execution_status"} :
      std::vector<std::string>{
        "/drl_unified_planner_node/set_parameters",
        "/drl/clear_trajectory",
        "/drl/plan"};

    for (const auto & service : required_services) {
      if (!serviceAvailable(node, service, std::chrono::seconds(1))) {
        if (service == "/drl_unified_planner_node/set_parameters") {
          if (!self) {
            return;
          }
          self->appendActionLog(QString(
            "[MovePoseRL] Backend not ready: missing %1. "
            "Please launch DRL planner node before using move_pose_rl.")
            .arg(QString::fromStdString(service)));
          self->setMovePoseRlBusy(false);
          return;
        }
        fail_preflight(QString::fromStdString(service));
        return;
      }
    }

    typename Client::SendGoalOptions options;
    options.goal_response_callback =
      [self](const GoalHandle::SharedPtr & goal_handle) {
        if (!self) {
          return;
        }
        if (goal_handle) {
          self->appendActionLog("[MovePoseRL] goal accepted.");
          return;
        }
        self->appendActionLog("[MovePoseRL] FAILED at goal_response: goal rejected");
        self->setMovePoseRlBusy(false);
      };
    options.feedback_callback =
      [self](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const MovePoseRl::Feedback> feedback) {
        if (!self) {
          return;
        }
        if (feedback) {
          self->appendActionLog(feedbackString(*feedback));
        }
      };
    options.result_callback =
      [self](const GoalHandle::WrappedResult & result) {
        if (!self) {
          return;
        }
        self->appendActionLog(resultString<MovePoseRl>(result));
        self->setMovePoseRlBusy(false);
      };

    client->async_send_goal(goal, options);
  }).detach();
}

void TaskActionController::sendCheckerBoard(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double velocity = kDefaultVelocityScale;
  const auto step = readMmAsMeter(
    lineEdit(root_, "txtCheckBoardStep"),
    kDefaultCheckerStepMm,
    "[Check Board] step");
  if (!step ||
    !readVelocity(root_, "", velocity, "CheckerBoard velocity_scale", log, &velocity))
  {
    return;
  }

  CheckerBoard::Goal goal;
  goal.step = *step;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<CheckerBoard>(
    node_, "/move_checker_board", execute ? "Check Board Start" : "Check Board Plan", goal, log);
}

void TaskActionController::sendRepeatabilityTest(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double velocity = kDefaultRepeatVelocityScale;
  int repeat_count = 3;

  const auto retract_x = readMmAsMeter(lineEdit(root_, "repeatRetractX"), kDefaultRepeatRetractXMm, "[Repeatability] retract X");
  const auto retract_y = readMmAsMeter(lineEdit(root_, "repeatRetractY"), kDefaultRepeatRetractYMm, "[Repeatability] retract Y");
  const auto retract_z = readMmAsMeter(lineEdit(root_, "repeatRetractZ"), kDefaultRepeatRetractZMm, "[Repeatability] retract Z");
  const auto disturb1_x = readMmAsMeter(lineEdit(root_, "repeatDisturb1X"), kDefaultRepeatDisturb1XMm, "[Repeatability] disturb1 X");
  const auto disturb1_y = readMmAsMeter(lineEdit(root_, "repeatDisturb1Y"), kDefaultRepeatDisturb1YMm, "[Repeatability] disturb1 Y");
  const auto disturb1_z = readMmAsMeter(lineEdit(root_, "repeatDisturb1Z"), kDefaultRepeatDisturb1ZMm, "[Repeatability] disturb1 Z");
  const auto offset = readMmAsMeter(lineEdit(root_, "txtMeasOffset"), kDefaultRepeatMeasOffsetMm, "[Repeatability] meas_offset");
  if (!retract_x || !retract_y || !retract_z || !disturb1_x || !disturb1_y || !disturb1_z ||
    !offset ||
    !readPositiveInt(root_, "txtRepeatCount", repeat_count, "repeat_count", log, &repeat_count) ||
    !readVelocity(root_, "", velocity, "Repeatability velocity_scale", log, &velocity))
  {
    return;
  }
  if (*offset == 0.0) {
    appendActionLog("[Input Error] [Repeatability] meas_offset must be non-zero mm");
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
  goal.retract_pose = makeStampedPose("world", makePose(*retract_x, *retract_y, *retract_z, q));
  goal.disturb_pose_1 = makeStampedPose("world", makePose(*disturb1_x, *disturb1_y, *disturb1_z, q));
  goal.axis = axis;
  goal.meas_offset = *offset;
  goal.repeat_count = repeat_count;
  goal.velocity_scale = velocity;
  goal.execute = execute;
  sendGoal<RepeatabilityTest>(
    node_, "/repeatability_test",
    execute ? "Repeatability Test Start" : "Repeatability Test Plan", goal, log);
}

}  // namespace robot_gui
