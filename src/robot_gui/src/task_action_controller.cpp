#include "robot_gui/task_action_controller.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <QAbstractButton>
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
#include <QSignalBlocker>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QRadioButton>
#include <QTabWidget>
#include <QTextEdit>
#include <QTimer>
#include <QWidget>

#include "action_msgs/srv/cancel_goal.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_task_manager/action/checker_board.hpp"
#include "robot_task_manager/action/drl_pick_place.hpp"
#include "robot_task_manager/action/go_home.hpp"
#include "robot_task_manager/action/move_gripper.hpp"
#include "robot_task_manager/action/move_pose_rl.hpp"
#include "robot_task_manager/action/move_to_pose.hpp"
#include "robot_task_manager/action/move_to_pose_cartesian.hpp"
#include "robot_task_manager/action/move_to_pose_obstacle.hpp"
#include "robot_task_manager/action/pick_place.hpp"
#include "robot_task_manager/action/repeatability_test.hpp"

namespace robot_gui
{
namespace
{
using CheckerBoard = robot_task_manager::action::CheckerBoard;
using DrlPickPlace = robot_task_manager::action::DrlPickPlace;
using GoHome = robot_task_manager::action::GoHome;
using MoveGripper = robot_task_manager::action::MoveGripper;
using MovePoseRl = robot_task_manager::action::MovePoseRl;
using MoveToPose = robot_task_manager::action::MoveToPose;
using MoveToPoseCartesian = robot_task_manager::action::MoveToPoseCartesian;
using MoveToPoseObstacle = robot_task_manager::action::MoveToPoseObstacle;
using PickPlace = robot_task_manager::action::PickPlace;
using RepeatabilityTest = robot_task_manager::action::RepeatabilityTest;
using LogFn = std::function<void(const QString &)>;

constexpr double DEFAULT_GUI_VELOCITY_SCALE = 0.1;
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

// codex.md "Auto Planning cho tab Move Pose RL":
// Default RPY orientation when the GUI orientation fields are empty — must be
// R=180, P=0, Y=90 deg (the config used to keep RL planning stable), NOT an
// identity quaternion.
constexpr double kMovePoseRlDefaultRollDeg = 180.0;
constexpr double kMovePoseRlDefaultPitchDeg = 0.0;
constexpr double kMovePoseRlDefaultYawDeg = 90.0;
// Auto Plan timer interval fallback when the spinbox is absent (range 1.0-2.0s).
constexpr int kMovePoseRlAutoPlanIntervalMs = 1500;
// Trained DRL workspace limits (metres) the GUI enforces before sending a goal
// (codex.md section 6: X 250-500, Y -150-150, Z 20-300 mm).
constexpr double kRlWorkspaceMinXm = 0.250;
constexpr double kRlWorkspaceMaxXm = 0.500;
constexpr double kRlWorkspaceMinYm = -0.150;
constexpr double kRlWorkspaceMaxYm = 0.150;
constexpr double kRlWorkspaceMinZm = 0.020;
constexpr double kRlWorkspaceMaxZm = 0.300;
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

QString feedbackString(const MoveToPoseObstacle::Feedback & feedback)
{
  return QString("feedback stage=%1 progress=%2")
    .arg(QString::fromStdString(feedback.current_stage))
    .arg(feedback.progress, 0, 'f', 2);
}

QString feedbackString(const GoHome::Feedback & feedback)
{
  return QString("feedback step=%1 progress=%2")
    .arg(QString::fromStdString(feedback.current_step))
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
QString resultString<MoveToPoseObstacle>(
  const rclcpp_action::ClientGoalHandle<MoveToPoseObstacle>::WrappedResult & result)
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

// slot_key identifies which Stop button can cancel this goal (e.g. "MovePose",
// "PickPlaceRL"). When non-empty, a cancel handler is registered on
// `controller` as soon as the goal is accepted, and cleared once the goal
// reaches a terminal state (result_callback) — so Stop always reflects
// whether there is actually something to cancel right now.
template<typename ActionT>
void sendGoal(
  const rclcpp::Node::SharedPtr & node,
  const std::string & action_name,
  const QString & label,
  const typename ActionT::Goal & goal,
  const LogFn & log,
  TaskActionController * controller = nullptr,
  const QString & slot_key = QString())
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
    [label, log, controller, slot_key, client](const typename GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        log(QString("%1: goal rejected.").arg(label));
        return;
      }
      log(QString("%1: goal accepted.").arg(label));
      if (controller != nullptr && !slot_key.isEmpty()) {
        controller->registerCancelHandle(slot_key, [client, goal_handle, label, log]() {
          client->async_cancel_goal(
            goal_handle,
            [label, log](const typename Client::CancelResponse::SharedPtr & response) {
              const bool accepted = response &&
                response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE;
              log(accepted ? QString("%1: cancel accepted.").arg(label) :
                QString("%1: cancel rejected hoặc goal đã kết thúc.").arg(label));
            });
        });
      }
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
    [label, log, controller, slot_key](const typename GoalHandle::WrappedResult & result) {
      log(QString("%1: %2").arg(label, resultString<ActionT>(result)));
      if (controller != nullptr && !slot_key.isEmpty()) {
        controller->clearCancelHandle(slot_key);
      }
    };

  client->async_send_goal(goal, options);
}

}  // namespace

TaskActionController::TaskActionController(
  std::shared_ptr<RobotGuiNode> node,
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
  connectButton("btnStopTask", [this]() {requestCancel("MovePose", "Move Pose");});
  connectButton("btnGoHome", [this]() {sendGoHome();});

  connectButton("btnRLPlan", [this]() {sendMovePoseRl(false);});
  connectButton("btnRLExecute", [this]() {
    // Execute must not race with Auto Plan (codex.md section 13): turn Auto Plan
    // off first, then run the execute goal.
    if (auto_plan_timer_ != nullptr && auto_plan_timer_->isActive()) {
      setAutoPlanEnabled(false, "disabled before Execute");
    }
    sendMovePoseRl(true);
  });
  connectButton("btnRLStop", [this]() {
    if (auto_plan_timer_ != nullptr && auto_plan_timer_->isActive()) {
      setAutoPlanEnabled(false, "stopped by Stop button");
      appendActionLog("[MovePoseRL AutoPlan] stopped by Stop button");
    }
    requestCancel("MovePoseRL", "Move Pose RL");
  });
  setupAutoPlan();
  setupWoodTarget();

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
  connectButton("btnPickPlaceStop", [this]() {requestCancel("PickPlace", "Pick Place");});

  connectButton("btnPickPlaceVisionPlan", [this]() {sendPickPlaceVision(false);});
  connectButton("btnPickPlaceVisionStart", [this]() {sendPickPlaceVision(true);});
  connectButton("btnPickPlaceVisionStop", [this]() {requestCancel("PickPlaceVision", "Pick Place Vision");});

  connectButton("btnPickPlaceRLPlan", [this]() {sendDrlPickPlace(false);});
  connectButton("btnPickPlaceRLStart", [this]() {sendDrlPickPlace(true);});
  connectButton("btnPickPlaceRLStop", [this]() {requestCancel("PickPlaceRL", "Pick Place RL");});

  connectButton("btnCheckBoardPlan", [this]() {sendCheckerBoard(false);});
  connectButton("btnCheckBoardStart", [this]() {sendCheckerBoard(true);});
  connectButton("btnCheckBoardStop", [this]() {requestCancel("CheckBoard", "Check Board");});

  connectButton("btnRepeatPlan", [this]() {sendRepeatabilityTest(false);});
  connectButton("btnRepeatStart", [this]() {sendRepeatabilityTest(true);});
  connectButton("btnRepeatStop", [this]() {requestCancel("Repeatability", "Repeatability Test");});
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
  // Superseded by cbMovePoseMode (codex2.md 11.1: a list/combo replaces the
  // old Joint/Cartesian toggle) — hidden rather than deleted so nothing
  // that still queries it by objectName breaks.
  if (auto * checkbox = root_->findChild<QCheckBox *>("chkMovePoseCartesian")) {
    checkbox->hide();
  }

  addPlanButtonIfMissing("btnPickPlacePlan", "tabPickPlace");
  addPlanButtonIfMissing("btnPickPlaceVisionPlan", "tabPickPlaceVision");
  addPlanButtonIfMissing("btnPickPlaceRLPlan", "tabPickPlaceRL");
  addPlanButtonIfMissing("btnCheckBoardPlan", "tabCheckBoard");

  // Log toggles are now static widgets in robot_gui.ui (codex.md section 2/9:
  // layout must live in the .ui, not be created in code). Here we only wire
  // their ON/OFF text so the state is unmistakable.
  setupLogToggle("chkMovePoseLog");
  setupVisionObstacleToggle("chkMoveObstacleUseVision");
  setupLogToggle("chkMovePoseRlLog");
  setupLogToggle("chkPickPlaceLog");
  setupLogToggle("chkPickPlaceVisionLog");
  setupLogToggle("chkPickPlaceRlLog");
  setupLogToggle("chkCheckBoardLog");
  setupLogToggle("chkRepeatabilityLog");
}

void TaskActionController::setupLogToggle(const QString & object_name)
{
  auto * toggle = root_->findChild<QAbstractButton *>(object_name);
  if (toggle == nullptr) {
    appendActionLog(QString("Không tìm thấy log toggle %1 trong .ui.").arg(object_name));
    return;
  }
  toggle->setChecked(false);
  toggle->setText("Log: OFF");
  connect(toggle, &QAbstractButton::toggled, this, [toggle](bool checked) {
    toggle->setText(checked ? "Log: ON" : "Log: OFF");
  });
}

bool TaskActionController::isLogEnabled(const QString & toggle_object_name) const
{
  auto * toggle = root_->findChild<QAbstractButton *>(toggle_object_name);
  return toggle != nullptr && toggle->isChecked();
}

// codex2.md section 10: MoveToPoseObstacle needs a minimal way for the GUI
// user to opt into /vision/box_objects instead of always sending
// use_vision_obstacle=false. Same ON/OFF toggle pattern as setupLogToggle.
void TaskActionController::setupVisionObstacleToggle(const QString & object_name)
{
  auto * toggle = root_->findChild<QAbstractButton *>(object_name);
  if (toggle == nullptr) {
    appendActionLog(QString("Không tìm thấy toggle %1 trong .ui.").arg(object_name));
    return;
  }
  toggle->setChecked(false);
  toggle->setText("Vision Obstacle: OFF");
  connect(toggle, &QAbstractButton::toggled, this, [toggle](bool checked) {
    toggle->setText(checked ? "Vision Obstacle: ON" : "Vision Obstacle: OFF");
  });
}

bool TaskActionController::isVisionObstacleEnabled() const
{
  auto * toggle = root_->findChild<QAbstractButton *>("chkMoveObstacleUseVision");
  return toggle != nullptr && toggle->isChecked();
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
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s", msg.toStdString().c_str());
  }
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

std::optional<double> TaskActionController::readVelocityScale(
  QLineEdit * edit,
  double default_value,
  const QString & field_name)
{
  double value = default_value;
  if (edit != nullptr && !edit->text().trimmed().isEmpty()) {
    bool ok = false;
    value = edit->text().trimmed().toDouble(&ok);
    if (!ok || !std::isfinite(value)) {
      appendActionLog(QString("[Input Error] %1 invalid velocity_scale").arg(field_name));
      return std::nullopt;
    }
  }

  if (value <= 0.0 || value > 1.0) {
    appendActionLog(QString("[Input Error] %1 must be in range (0, 1]").arg(field_name));
    return std::nullopt;
  }

  return value;
}

void TaskActionController::setMovePoseRlBusy(bool busy)
{
  // Atomic flag drives Auto Plan anti-overlap (read from the timer tick, which
  // runs on the GUI thread; written here from either the GUI thread or a ROS
  // executor thread in the result callback).
  move_pose_rl_busy_.store(busy);
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

// Reads the Move Pose RL X/Y/Z line edits (mm -> m) exactly like sendMovePoseRl
// (same field names, same defaults, no sign inversion on Y), but without the
// per-axis Action-Log spam that would flood the log every Auto Plan tick.
bool TaskActionController::readMovePoseRlTargetMeters(double & x, double & y, double & z) const
{
  auto parse = [](QWidget * root, const QString & name, double default_mm, double & out_m) -> bool {
    double mm = default_mm;
    if (auto * edit = root ? root->findChild<QLineEdit *>(name) : nullptr) {
      const QString text = edit->text().trimmed();
      if (!text.isEmpty()) {
        bool ok = false;
        mm = text.toDouble(&ok);
        if (!ok || !std::isfinite(mm)) {
          return false;
        }
      }
    }
    out_m = mm / 1000.0;
    return true;
  };
  return parse(root_, "rlPosePositionX", kDefaultMoveXMm, x) &&
         parse(root_, "rlPosePositionY", kDefaultMoveYMm, y) &&
         parse(root_, "rlPosePositionZ", kDefaultMoveZMm, z);
}

void TaskActionController::setupAutoPlan()
{
  auto * toggle = root_->findChild<QAbstractButton *>("btnMovePoseRlAutoPlan");
  if (toggle == nullptr) {
    appendActionLog("Không tìm thấy nút Auto Plan (btnMovePoseRlAutoPlan) trong .ui.");
    return;
  }
  toggle->setChecked(false);
  toggle->setText("Auto Plan: OFF");

  auto_plan_timer_ = new QTimer(this);
  auto_plan_timer_->setSingleShot(false);
  connect(auto_plan_timer_, &QTimer::timeout, this, &TaskActionController::onAutoPlanTick);

  connect(toggle, &QAbstractButton::toggled, this, [this](bool checked) {
    setAutoPlanEnabled(checked, checked ? "button ON" : "button OFF");
  });

  // Auto-off when the user leaves the Move Pose RL tab so we never keep
  // planning in the background for a tab the user is no longer looking at.
  if (auto * tabs = root_->findChild<QTabWidget *>("taskModeTabs")) {
    connect(tabs, &QTabWidget::currentChanged, this, [this, tabs](int index) {
      QWidget * page = tabs->widget(index);
      const bool on_rl_tab = page != nullptr && page->objectName() == "tabMovePoseRL";
      if (!on_rl_tab && auto_plan_timer_ != nullptr && auto_plan_timer_->isActive()) {
        setAutoPlanEnabled(false, "left Move Pose RL tab");
      }
    });
  }
}

void TaskActionController::setAutoPlanEnabled(bool on, const QString & reason)
{
  if (auto * toggle = root_->findChild<QAbstractButton *>("btnMovePoseRlAutoPlan")) {
    // Block signals so a programmatic toggle (Stop/Execute/tab-change) does not
    // re-enter setAutoPlanEnabled through the toggled() connection.
    const QSignalBlocker blocker(toggle);
    toggle->setChecked(on);
    toggle->setText(on ? "Auto Plan: ON" : "Auto Plan: OFF");
  }
  if (auto_plan_timer_ == nullptr) {
    return;
  }
  if (on) {
    int interval_ms = kMovePoseRlAutoPlanIntervalMs;
    if (auto * spin = root_->findChild<QDoubleSpinBox *>("spinMovePoseRlAutoPlanIntervalSec")) {
      interval_ms = static_cast<int>(std::clamp(spin->value(), 1.0, 2.0) * 1000.0);
    }
    auto_plan_cycle_ = 0;
    auto_plan_timer_->start(interval_ms);
    appendActionLog(QString("[MovePoseRL AutoPlan] ON (interval=%1 ms) — %2")
      .arg(interval_ms).arg(reason));
    onAutoPlanTick();  // fire first plan immediately, don't wait a full interval
  } else {
    if (auto_plan_timer_->isActive()) {
      auto_plan_timer_->stop();
    }
    appendActionLog(QString("[MovePoseRL AutoPlan] OFF — %1").arg(reason));
  }
}

void TaskActionController::onAutoPlanTick()
{
  // Anti-overlap: never send a second /move_pose_rl goal while one is running.
  if (move_pose_rl_busy_.load()) {
    appendActionLog("[MovePoseRL AutoPlan] previous plan still running, skip this cycle");
    return;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  if (!movePoseRlOrientationDeg(roll, pitch, yaw)) {
    appendActionLog("[MovePoseRL AutoPlan] invalid orientation input, disabling Auto Plan");
    setAutoPlanEnabled(false, "invalid orientation input");
    return;
  }
  const auto orientation = rpyDegToQuaternion(roll, pitch, yaw);

  // Target: wood detection (Wood Target ON) or manual X/Y/Z. Returns false
  // (skip this cycle, no goal) if no valid wood / outside workspace / invalid.
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  QString source;
  if (!movePoseRlTargetPosition(true, x, y, z, source)) {
    return;  // keep timer running; the helper already logged the reason
  }

  double velocity = DEFAULT_GUI_VELOCITY_SCALE;
  if (const auto v = readVelocityScale(
      lineEdit(root_, "txtVelocityScale"), DEFAULT_GUI_VELOCITY_SCALE,
      "[MovePoseRL AutoPlan] velocity_scale"))
  {
    velocity = *v;
  }

  MovePoseRl::Goal goal;
  goal.target_pose = makePose(x, y, z, orientation);
  goal.velocity_scale = velocity;
  goal.execute = false;  // Auto Plan NEVER executes the robot.
  goal.enable_metrics_log = isLogEnabled("chkMovePoseRlLog");

  ++auto_plan_cycle_;
  appendActionLog(QString("[MovePoseRL AutoPlan] cycle #%1").arg(auto_plan_cycle_));
  appendActionLog(QString("[MovePoseRL AutoPlan] target_source=%1").arg(source));
  appendActionLog(QString("[MovePoseRL AutoPlan] target_base=(%1, %2, %3), execute=false")
    .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
  appendActionLog(QString("[MovePoseRL AutoPlan] rpy_deg=(%1, %2, %3)")
    .arg(roll, 0, 'f', 1).arg(pitch, 0, 'f', 1).arg(yaw, 0, 'f', 1));
  appendActionLog("[MovePoseRL AutoPlan] goal sent");

  dispatchMovePoseRlGoal(goal, false);
}

bool TaskActionController::movePoseRlOrientationDeg(
  double & roll_deg, double & pitch_deg, double & yaw_deg)
{
  // Default fixed RL orientation R=180/P=0/Y=90 when the RPY fields are empty
  // (never identity — codex.md section 9). If any field has text, use the user
  // values (blank individual fields fall back to the default for that axis).
  roll_deg = kMovePoseRlDefaultRollDeg;
  pitch_deg = kMovePoseRlDefaultPitchDeg;
  yaw_deg = kMovePoseRlDefaultYawDeg;
  const bool has_orientation =
    editHasText(root_, "rlPoseOrientationRoll") ||
    editHasText(root_, "rlPoseOrientationPitch") ||
    editHasText(root_, "rlPoseOrientationYaw");
  if (!has_orientation) {
    return true;
  }
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  return
    readDouble(root_, "rlPoseOrientationRoll", kMovePoseRlDefaultRollDeg, "Roll (deg)", log, &roll_deg) &&
    readDouble(root_, "rlPoseOrientationPitch", kMovePoseRlDefaultPitchDeg, "Pitch (deg)", log, &pitch_deg) &&
    readDouble(root_, "rlPoseOrientationYaw", kMovePoseRlDefaultYawDeg, "Yaw (deg)", log, &yaw_deg);
}

bool TaskActionController::movePoseRlTargetPosition(
  bool for_auto_plan, double & x, double & y, double & z, QString & source)
{
  const QString prefix = for_auto_plan ? "[MovePoseRL AutoPlan]" : "[MovePoseRL]";

  if (wood_target_enabled_) {
    source = "vision_wood";
    RobotGuiNode::WoodTarget wt;
    std::string err;
    if (!node_->move_pose_rl_wood_target(wt, err)) {
      // No silent fallback to manual target (codex.md section 4/20).
      appendActionLog(for_auto_plan ?
        QString("[MovePoseRL AutoPlan] no valid wood target; skip (%1)")
          .arg(QString::fromStdString(err)) :
        QString("[MovePoseRL] Wood Target ON but no valid /vision/wood_objects; "
          "skip planning (%1)").arg(QString::fromStdString(err)));
      return false;
    }
    const QString frame_in = QString::fromStdString(wt.frame_in.empty() ? "(none)" : wt.frame_in);
    appendActionLog(QString(
        "%1 selected wood target: source=/vision/wood_objects wood_id=%2 "
        "confidence=%3 frame_in=%4 selected_rule=highest_confidence")
      .arg(prefix).arg(wt.wood_id).arg(wt.confidence, 0, 'f', 3).arg(frame_in));
    appendActionLog(QString(
        "%1 transform wood target: frame_in=%2 frame_out=base_link "
        "target_base=(%3, %4, %5) offset=(%6, %7, %8)")
      .arg(prefix).arg(frame_in)
      .arg(wt.x_m, 0, 'f', 3).arg(wt.y_m, 0, 'f', 3).arg(wt.z_m, 0, 'f', 3)
      .arg(wt.x_offset_m, 0, 'f', 3).arg(wt.y_offset_m, 0, 'f', 3).arg(wt.z_offset_m, 0, 'f', 3));
    x = wt.x_m;
    y = wt.y_m;
    z = wt.z_m;
  } else {
    source = "manual_gui";
    if (!readMovePoseRlTargetMeters(x, y, z)) {
      appendActionLog(QString("%1 invalid X/Y/Z input").arg(prefix));
      if (for_auto_plan) {
        setAutoPlanEnabled(false, "invalid target input");
      }
      return false;
    }
  }

  // Workspace guard — reject (never silently clamp) when using wood target or in
  // Auto Plan. Manual single Plan keeps its prior behaviour (server validates).
  const bool enforce_workspace = wood_target_enabled_ || for_auto_plan;
  if (enforce_workspace &&
    (x < kRlWorkspaceMinXm || x > kRlWorkspaceMaxXm ||
    y < kRlWorkspaceMinYm || y > kRlWorkspaceMaxYm ||
    z < kRlWorkspaceMinZm || z > kRlWorkspaceMaxZm))
  {
    appendActionLog(QString(
        "%1 %2 target outside trained workspace: target=(%3, %4, %5); "
        "workspace x=[0.250,0.500], y=[-0.150,0.150], z=[0.020,0.300]")
      .arg(prefix).arg(wood_target_enabled_ ? "wood" : "manual")
      .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
    return false;
  }
  return true;
}

void TaskActionController::setupWoodTarget()
{
  auto * toggle = root_->findChild<QAbstractButton *>("btnMovePoseRlVisionTarget");
  if (toggle == nullptr) {
    appendActionLog("Không tìm thấy nút Wood Target (btnMovePoseRlVisionTarget) trong .ui.");
    return;
  }
  toggle->setChecked(false);
  toggle->setText("Wood Target: OFF");
  wood_target_enabled_ = false;
  connect(toggle, &QAbstractButton::toggled, this, [this, toggle](bool checked) {
    wood_target_enabled_ = checked;
    toggle->setText(checked ? "Wood Target: ON" : "Wood Target: OFF");
    // Grey out the manual X/Y/Z fields so it is obvious they are unused.
    for (const char * name : {"rlPosePositionX", "rlPosePositionY", "rlPosePositionZ"}) {
      if (auto * edit = root_->findChild<QLineEdit *>(name)) {
        edit->setEnabled(!checked);
      }
    }
    appendActionLog(checked ?
      "[MovePoseRL] Wood Target: ON — target from /vision/wood_objects "
      "(manual X/Y/Z ignored)" :
      "[MovePoseRL] Wood Target: OFF — target from manual X/Y/Z");
  });
}

void TaskActionController::registerCancelHandle(const QString & slot_key, std::function<void()> canceller)
{
  std::lock_guard<std::mutex> lock(cancel_mutex_);
  cancel_handlers_[slot_key] = std::move(canceller);
}

void TaskActionController::clearCancelHandle(const QString & slot_key)
{
  std::lock_guard<std::mutex> lock(cancel_mutex_);
  cancel_handlers_.erase(slot_key);
}

// Stop button handler for every action-calling tab: cancels the goal
// currently registered for slot_key via a real action-client cancel call
// (codex2.md section 7 — must not just disable UI). If no goal is in
// flight (already completed, or never started), this is a harmless no-op
// with a status message, never an error.
void TaskActionController::requestCancel(const QString & slot_key, const QString & label)
{
  std::function<void()> canceller;
  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    auto it = cancel_handlers_.find(slot_key);
    if (it != cancel_handlers_.end()) {
      canceller = it->second;
    }
  }
  if (!canceller) {
    appendActionLog(QString("%1: không có goal đang chạy để cancel.").arg(label));
    return;
  }
  appendActionLog(QString("%1: gửi yêu cầu cancel...").arg(label));
  canceller();
}

void TaskActionController::sendMovePose(bool execute)
{
  // Planning mode selector (codex2.md 11.1): prefer the new combo box
  // (Joint / Cartesian / MoveToPoseObstacle); fall back to the legacy
  // checkbox if the combo somehow isn't present, so this still degrades
  // gracefully rather than crashing.
  QString mode = "MoveToPose (Joint)";
  if (auto * combo = root_->findChild<QComboBox *>("cbMovePoseMode")) {
    mode = combo->currentText();
  } else if (auto * checkbox = root_->findChild<QCheckBox *>("chkMovePoseCartesian");
    checkbox != nullptr && checkbox->isChecked())
  {
    mode = "MoveToPoseCartesian";
  }

  if (mode == "MoveToPoseObstacle") {
    sendMoveToPoseObstacle(execute);
    return;
  }

  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtMovePoseVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[Move Pose] velocity_scale");
  const auto x = readMmAsMeter(lineEdit(root_, "txtTargetX"), kDefaultMoveXMm, "[Move Pose] X");
  const auto y = readMmAsMeter(lineEdit(root_, "txtTargetY"), kDefaultMoveYMm, "[Move Pose] Y");
  const auto z = readMmAsMeter(lineEdit(root_, "txtTargetZ"), kDefaultMoveZMm, "[Move Pose] Z");
  if (!velocity || !x || !y || !z) {
    return;
  }
  appendActionLog(QString("[Move Pose] velocity_scale=%1").arg(*velocity, 0, 'f', 3));

  bool ok = true;
  const auto orientation =
    orientationFromRpyFields(root_, "txtTargetRoll", "txtTargetPitch", "txtTargetYaw", log, &ok);
  if (!ok) {
    return;
  }

  if (mode == "MoveToPoseCartesian") {
    MoveToPoseCartesian::Goal goal;
    goal.target_pose = makePose(*x, *y, *z, orientation);
    goal.velocity_scale = *velocity;
    goal.execute = execute;
    goal.enable_tcp_log = isLogEnabled("chkMovePoseLog");
    appendActionLog(QString("[MoveToPoseCartesian GUI] enable_tcp_log=%1")
      .arg(goal.enable_tcp_log ? "true" : "false"));
    sendGoal<MoveToPoseCartesian>(
      node_, "/move_to_pose_cartesian",
      execute ? "Move Pose Cartesian Start" : "Move Pose Cartesian Plan", goal, log,
      this, "MovePose");
  } else {
    MoveToPose::Goal goal;
    goal.target_pose = makePose(*x, *y, *z, orientation);
    goal.velocity_scale = *velocity;
    goal.execute = execute;
    goal.enable_tcp_log = isLogEnabled("chkMovePoseLog");
    appendActionLog(QString("[MoveToPose GUI] enable_tcp_log=%1")
      .arg(goal.enable_tcp_log ? "true" : "false"));
    sendGoal<MoveToPose>(
      node_, "/move_to_pose", execute ? "Move Pose Start" : "Move Pose Plan", goal, log,
      this, "MovePose");
  }
}

// codex2.md 11.2: MoveToPoseObstacle uses the same target pose fields as
// Move Pose. The "Vision Obstacle" toggle (chkMoveObstacleUseVision) is the
// minimal addition from codex2.md section 10: OFF keeps the original
// behavior (use_vision_obstacle=false/require_obstacle=false, no obstacle
// required — this is what existing fallback tests exercise), ON opts into
// reading /vision/box_objects and requires the server to resolve a real
// obstacle (require_obstacle=true) rather than silently planning without one.
void TaskActionController::sendMoveToPoseObstacle(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtMovePoseVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[MoveToPoseObstacle] velocity_scale");
  const auto x = readMmAsMeter(lineEdit(root_, "txtTargetX"), kDefaultMoveXMm, "[MoveToPoseObstacle] X");
  const auto y = readMmAsMeter(lineEdit(root_, "txtTargetY"), kDefaultMoveYMm, "[MoveToPoseObstacle] Y");
  const auto z = readMmAsMeter(lineEdit(root_, "txtTargetZ"), kDefaultMoveZMm, "[MoveToPoseObstacle] Z");
  if (!velocity || !x || !y || !z) {
    return;
  }
  appendActionLog(QString("[MoveToPoseObstacle] velocity_scale=%1").arg(*velocity, 0, 'f', 3));

  bool ok = true;
  const auto orientation =
    orientationFromRpyFields(root_, "txtTargetRoll", "txtTargetPitch", "txtTargetYaw", log, &ok);
  if (!ok) {
    return;
  }

  const bool use_vision_obstacle = isVisionObstacleEnabled();
  appendActionLog(
    use_vision_obstacle ?
    "[MoveToPoseObstacle] Vision Obstacle ON: use_vision_obstacle=true, "
    "require_obstacle=true, obstacle_class=box." :
    "[MoveToPoseObstacle] Vision Obstacle OFF: sending without a required "
    "obstacle (use_vision_obstacle=false, require_obstacle=false).");

  MoveToPoseObstacle::Goal goal;
  goal.target_pose = makePose(*x, *y, *z, orientation);
  goal.velocity_scale = *velocity;
  goal.execute = execute;
  goal.use_vision_obstacle = use_vision_obstacle;
  goal.obstacle_class = "box";
  goal.require_obstacle = use_vision_obstacle;
  goal.use_fallback_obstacle = false;
  goal.enable_metrics_log = isLogEnabled("chkMovePoseLog");
  appendActionLog(QString("[MoveToPoseObstacle GUI] enable_metrics_log=%1")
    .arg(goal.enable_metrics_log ? "true" : "false"));
  sendGoal<MoveToPoseObstacle>(
    node_, "/move_to_pose_obstacle",
    execute ? "MoveToPoseObstacle Start" : "MoveToPoseObstacle Plan", goal, log,
    this, "MovePose");
}

void TaskActionController::sendGoHome()
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  GoHome::Goal goal;
  goal.start = true;
  goal.execute = true;
  sendGoal<GoHome>(node_, "/gohome", "GoHome", goal, log, this, "MovePose");
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
  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtPickPlaceVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[Pick Place] velocity_scale");
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
  if (!velocity || !pick_x || !pick_y || !pick_z || !place_x || !place_y || !place_z) {
    return;
  }
  appendActionLog(QString("[Pick Place] velocity_scale=%1").arg(*velocity, 0, 'f', 3));

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
  goal.velocity_scale = *velocity;
  goal.execute = execute;
  goal.enable_tcp_log = isLogEnabled("chkPickPlaceLog");
  appendActionLog(QString("[PickPlace GUI] enable_tcp_log=%1")
    .arg(goal.enable_tcp_log ? "true" : "false"));
  sendGoal<PickPlace>(
    node_, "/pickplace", execute ? "Pick Place Start" : "Pick Place Plan", goal, log,
    this, "PickPlace");
}

// codex2.md section 5: pick pose must come from the highest-confidence
// `wood` detection (never from `box`). If no fresh wood detection is
// available, no goal is sent — the manual X/Y/Z fields are only a
// last-resort fallback for testing without a camera.
void TaskActionController::sendPickPlaceVision(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};

  geometry_msgs::msg::Pose wood_pose;
  float wood_confidence = 0.0f;
  std::string wood_error;
  const bool have_wood = node_->best_wood_pose("base_link", wood_pose, wood_confidence, wood_error);

  std::optional<double> pick_x;
  std::optional<double> pick_y;
  std::optional<double> pick_z;
  if (have_wood) {
    pick_x = wood_pose.position.x;
    pick_y = wood_pose.position.y;
    pick_z = wood_pose.position.z;
    appendActionLog(QString(
      "[Pick Place Vision] wood detected (base_link): x=%1 y=%2 z=%3 confidence=%4")
      .arg(*pick_x, 0, 'f', 4).arg(*pick_y, 0, 'f', 4).arg(*pick_z, 0, 'f', 4)
      .arg(wood_confidence, 0, 'f', 3));
    if (auto * edit = lineEdit(root_, "txtObjectX")) {edit->setText(QString::number(*pick_x * 1000.0, 'f', 1));}
    if (auto * edit = lineEdit(root_, "txtObjectY")) {edit->setText(QString::number(*pick_y * 1000.0, 'f', 1));}
    if (auto * edit = lineEdit(root_, "txtObjectZ")) {edit->setText(QString::number(*pick_z * 1000.0, 'f', 1));}
  } else if (editHasText(root_, "txtObjectX") || editHasText(root_, "txtObjectY") ||
    editHasText(root_, "txtObjectZ"))
  {
    appendActionLog(QString::fromStdString(
      "[Pick Place Vision] Không có wood detection (" + wood_error +
      "); dùng pose nhập tay."));
    pick_x = readMmAsMeter(lineEdit(root_, "txtObjectX"), kDefaultPickXMm, "[Pick Place Vision] pick X");
    pick_y = readMmAsMeter(lineEdit(root_, "txtObjectY"), kDefaultPickYMm, "[Pick Place Vision] pick Y");
    pick_z = readMmAsMeter(lineEdit(root_, "txtObjectZ"), kDefaultPickZMm, "[Pick Place Vision] pick Z");
  } else {
    appendActionLog(QString::fromStdString(
      "[Pick Place Vision] Không tìm thấy wood (" + wood_error + "); không gửi goal."));
    return;
  }

  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtPickPlaceVisionVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[Pick Place Vision] velocity_scale");
  const auto place_x = readMmAsMeter(lineEdit(root_, "visionPlacePoseX"), kDefaultPlaceXMm, "[Pick Place Vision] place X");
  const auto place_y = readMmAsMeter(lineEdit(root_, "visionPlacePoseY"), kDefaultPlaceYMm, "[Pick Place Vision] place Y");
  const auto place_z = readMmAsMeter(lineEdit(root_, "visionPlacePoseZ"), kDefaultPlaceZMm, "[Pick Place Vision] place Z");
  if (!velocity || !pick_x || !pick_y || !pick_z || !place_x || !place_y || !place_z) {
    return;
  }
  appendActionLog(QString("[Pick Place Vision] velocity_scale=%1").arg(*velocity, 0, 'f', 3));

  bool ok = true;
  const auto place_q = orientationFromYawField(root_, "visionPlacePoseYaw", log, &ok);
  if (!ok) {
    return;
  }

  PickPlace::Goal goal;
  goal.pose_pick = makePose(*pick_x, *pick_y, *pick_z, defaultQuaternion());
  goal.pose_place = makePose(*place_x, *place_y, *place_z, place_q);
  goal.gripper = kDefaultPickGripperMm / 1000.0;
  goal.velocity_scale = *velocity;
  goal.execute = execute;
  goal.enable_tcp_log = isLogEnabled("chkPickPlaceVisionLog");
  appendActionLog(QString("[PickPlaceVision GUI] enable_tcp_log=%1")
    .arg(goal.enable_tcp_log ? "true" : "false"));
  sendGoal<PickPlace>(
    node_, "/pickplace", execute ? "Pick Place Vision Start" : "Pick Place Vision Plan", goal, log,
    this, "PickPlaceVision");
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

  // codex2.md section 6: pick target also comes from the highest-confidence
  // `wood` detection when available (same selection rule as Pick Place
  // Vision). DrlPickPlace has no built-in fallback pick pose input in this
  // tab, so — unlike Pick Place Vision — falling back to the pre-existing
  // fixed default keeps this tab usable without a camera rather than
  // blocking every call.
  geometry_msgs::msg::Pose pick_pose = makePose(
    kDefaultDrlPickXMm / 1000.0,
    kDefaultDrlPickYMm / 1000.0,
    kDefaultDrlPickZMm / 1000.0,
    drlRepeatQuaternion());
  float wood_confidence = 0.0f;
  std::string wood_error;
  if (node_->best_wood_pose("base_link", pick_pose, wood_confidence, wood_error)) {
    appendActionLog(QString(
      "[Pick Place RL] wood detected (base_link): x=%1 y=%2 z=%3 confidence=%4")
      .arg(pick_pose.position.x, 0, 'f', 4).arg(pick_pose.position.y, 0, 'f', 4)
      .arg(pick_pose.position.z, 0, 'f', 4).arg(wood_confidence, 0, 'f', 3));
    pick_pose.orientation = drlRepeatQuaternion();
  } else {
    appendActionLog(QString::fromStdString(
      "[Pick Place RL] Không có wood detection (" + wood_error + "); dùng pick pose mặc định."));
  }

  DrlPickPlace::Goal goal;
  goal.target_pick = makeStampedPose("base_link", pick_pose);
  goal.target_place = makeStampedPose("base_link", makePose(*place_x, *place_y, *place_z, place_q));
  goal.gripper_close_width_m = gripper;
  goal.execute = execute;
  goal.enable_metrics_log = isLogEnabled("chkPickPlaceRlLog");
  appendActionLog(QString("[DrlPickPlace GUI] enable_metrics_log=%1")
    .arg(goal.enable_metrics_log ? "true" : "false"));
  sendGoal<DrlPickPlace>(
    node_, "/drl_pickplace", execute ? "Pick Place RL Start" : "Pick Place RL Plan", goal, log,
    this, "PickPlaceRL");
}

void TaskActionController::sendMovePoseRl(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  double velocity = DEFAULT_GUI_VELOCITY_SCALE;
  if (!readVelocity(root_, "txtVelocityScale", velocity, "MovePoseRL velocity_scale", log, &velocity)) {
    return;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  if (!movePoseRlOrientationDeg(roll, pitch, yaw)) {
    return;
  }
  const auto orientation = rpyDegToQuaternion(roll, pitch, yaw);

  // Target comes from wood detection when Wood Target is ON, else manual X/Y/Z.
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  QString source;
  if (!movePoseRlTargetPosition(false, x, y, z, source)) {
    return;
  }

  appendActionLog(QString("[MovePoseRL] target_source=%1").arg(source));
  appendActionLog(QString("[MovePoseRL] target=(%1, %2, %3)")
    .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
  appendActionLog(QString("[MovePoseRL] target_rpy_deg=(%1, %2, %3)")
    .arg(roll, 0, 'f', 1).arg(pitch, 0, 'f', 1).arg(yaw, 0, 'f', 1));
  appendActionLog(QString("[MovePoseRL] target_quat=(%1, %2, %3, %4)")
    .arg(orientation.x, 0, 'f', 4).arg(orientation.y, 0, 'f', 4)
    .arg(orientation.z, 0, 'f', 4).arg(orientation.w, 0, 'f', 4));
  appendActionLog(QString("[MovePoseRL] velocity_scale=%1").arg(velocity, 0, 'f', 3));

  MovePoseRl::Goal goal;
  goal.target_pose = makePose(x, y, z, orientation);
  goal.velocity_scale = velocity;
  goal.execute = execute;
  goal.enable_metrics_log = isLogEnabled("chkMovePoseRlLog");
  appendActionLog(QString("[MovePoseRL GUI] enable_metrics_log=%1")
    .arg(goal.enable_metrics_log ? "true" : "false"));

  appendActionLog(execute ?
    "[MovePoseRL] Sending execute goal..." :
    "[MovePoseRL] Sending plan-only goal...");
  dispatchMovePoseRlGoal(goal, execute);
}

// Shared async dispatch for both the manual Plan/Execute buttons and Auto Plan:
// runs the DRL preflight service checks off the GUI thread, sends the goal, and
// wires feedback/result/cancel. Anti-overlap for Auto Plan relies on
// setMovePoseRlBusy() flipping move_pose_rl_busy_.
void TaskActionController::dispatchMovePoseRlGoal(
  const MovePoseRl::Goal & goal, bool execute)
{
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
      [self, client](const GoalHandle::SharedPtr & goal_handle) {
        if (!self) {
          return;
        }
        if (goal_handle) {
          self->appendActionLog("[MovePoseRL] goal accepted.");
          self->registerCancelHandle("MovePoseRL", [self, client, goal_handle]() {
            client->async_cancel_goal(
              goal_handle,
              [self](const typename Client::CancelResponse::SharedPtr & response) {
                if (!self) {
                  return;
                }
                const bool accepted = response &&
                  response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE;
                self->appendActionLog(accepted ?
                  "[MovePoseRL] cancel accepted." :
                  "[MovePoseRL] cancel rejected hoặc goal đã kết thúc.");
              });
          });
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
        self->clearCancelHandle("MovePoseRL");
        self->setMovePoseRlBusy(false);
      };

    client->async_send_goal(goal, options);
  }).detach();
}

void TaskActionController::sendCheckerBoard(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtCheckBoardVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[Check Board] velocity_scale");
  const auto step = readMmAsMeter(
    lineEdit(root_, "txtCheckBoardStep"),
    kDefaultCheckerStepMm,
    "[Check Board] step");
  if (!velocity || !step) {
    return;
  }
  appendActionLog(QString("[Check Board] velocity_scale=%1").arg(*velocity, 0, 'f', 3));

  CheckerBoard::Goal goal;
  goal.step = *step;
  goal.velocity_scale = *velocity;
  goal.execute = execute;
  goal.enable_tcp_log = isLogEnabled("chkCheckBoardLog");
  appendActionLog(QString("[CheckBoard GUI] enable_tcp_log=%1")
    .arg(goal.enable_tcp_log ? "true" : "false"));
  sendGoal<CheckerBoard>(
    node_, "/move_checker_board", execute ? "Check Board Start" : "Check Board Plan", goal, log,
    this, "CheckBoard");
}

void TaskActionController::sendRepeatabilityTest(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtRepeatabilityVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[Repeatability] velocity_scale");
  int repeat_count = 3;

  const auto retract_x = readMmAsMeter(lineEdit(root_, "repeatRetractX"), kDefaultRepeatRetractXMm, "[Repeatability] retract X");
  const auto retract_y = readMmAsMeter(lineEdit(root_, "repeatRetractY"), kDefaultRepeatRetractYMm, "[Repeatability] retract Y");
  const auto retract_z = readMmAsMeter(lineEdit(root_, "repeatRetractZ"), kDefaultRepeatRetractZMm, "[Repeatability] retract Z");
  const auto disturb1_x = readMmAsMeter(lineEdit(root_, "repeatDisturb1X"), kDefaultRepeatDisturb1XMm, "[Repeatability] disturb1 X");
  const auto disturb1_y = readMmAsMeter(lineEdit(root_, "repeatDisturb1Y"), kDefaultRepeatDisturb1YMm, "[Repeatability] disturb1 Y");
  const auto disturb1_z = readMmAsMeter(lineEdit(root_, "repeatDisturb1Z"), kDefaultRepeatDisturb1ZMm, "[Repeatability] disturb1 Z");
  const auto offset = readMmAsMeter(lineEdit(root_, "txtMeasOffset"), kDefaultRepeatMeasOffsetMm, "[Repeatability] meas_offset");
  if (!velocity || !retract_x || !retract_y || !retract_z || !disturb1_x || !disturb1_y || !disturb1_z ||
    !offset ||
    !readPositiveInt(root_, "txtRepeatCount", repeat_count, "repeat_count", log, &repeat_count))
  {
    return;
  }
  appendActionLog(QString("[Repeatability] velocity_scale=%1").arg(*velocity, 0, 'f', 3));
  if (*offset == 0.0) {
    appendActionLog("[Input Error] [Repeatability] meas_offset must be non-zero mm");
    return;
  }

  uint8_t axis = RepeatabilityTest::Goal::AXIS_X;
  if (auto * radio_y = root_->findChild<QRadioButton *>("radioRepeatAxisY"); radio_y && radio_y->isChecked()) {
    axis = RepeatabilityTest::Goal::AXIS_Y;
  } else if (auto * radio_z = root_->findChild<QRadioButton *>("radioRepeatAxisZ"); radio_z && radio_z->isChecked()) {
    axis = RepeatabilityTest::Goal::AXIS_Z;
  }

  const auto q = drlRepeatQuaternion();
  RepeatabilityTest::Goal goal;
  goal.retract_pose = makeStampedPose("world", makePose(*retract_x, *retract_y, *retract_z, q));
  goal.disturb_pose_1 = makeStampedPose("world", makePose(*disturb1_x, *disturb1_y, *disturb1_z, q));
  goal.axis = axis;
  goal.meas_offset = *offset;
  goal.repeat_count = repeat_count;
  goal.velocity_scale = *velocity;
  goal.execute = execute;
  goal.enable_tcp_log = isLogEnabled("chkRepeatabilityLog");
  appendActionLog(QString("[Repeatability GUI] enable_tcp_log=%1")
    .arg(goal.enable_tcp_log ? "true" : "false"));
  sendGoal<RepeatabilityTest>(
    node_, "/repeatability_test",
    execute ? "Repeatability Test Start" : "Repeatability Test Plan", goal, log,
    this, "Repeatability");
}

}  // namespace robot_gui
