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
// codex.md (MovePoseRL vision wood approach): when the MovePoseRL target comes from
// a detected wood, the raw wood centre (z~0.070 m) is too low/close to the object
// for the RL policy to plan/execute reliably (final_dist > accept radius,
// FAILED_FORWARD near fraction=1.0). Aim at an approach point this many metres ABOVE
// the wood instead. Manual GUI targets are NOT offset. 0.050 m keeps the target well
// inside the trained Z range [0.020, 0.300].
constexpr double kMovePoseRlVisionApproachZOffsetM = 0.050;
// codex.md (AutoPlan fail cooldown): if an Auto Plan cycle fails, do not re-send a
// goal for a target within kAutoPlanSameTargetThresholdM of the failed one until
// kAutoPlanFailCooldownSec has elapsed — avoids spamming the same failing plan.
constexpr double kAutoPlanSameTargetThresholdM = 0.010;
constexpr double kAutoPlanFailCooldownSec = 5.0;
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
// codex.md (fixed place update): default/fixed place pose moved to
// x=260 mm, y=-130 mm, z=100 mm (0.260, -0.130, 0.100 m). Used by PickPlace and
// PickPlaceVision when the place fields are left empty. Pick defaults unchanged.
constexpr double kDefaultPlaceXMm = 260.0;
constexpr double kDefaultPlaceYMm = -130.0;
constexpr double kDefaultPlaceZMm = 100.0;
constexpr double kDefaultGripperOpenMm = 48.0;
constexpr double kDefaultGripperCloseMm = 28.0;
constexpr double kDefaultPickGripperMm = 14.0;
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
// codex.md (fixed place update): PickPlaceRL default/fixed place pose also moved
// to the same x=260 mm, y=-130 mm, z=100 mm. DRL pick defaults unchanged.
constexpr double kDefaultDrlPlaceXMm = 260.0;
constexpr double kDefaultDrlPlaceYMm = -130.0;
constexpr double kDefaultDrlPlaceZMm = 100.0;
constexpr double kPi = 3.14159265358979323846;
// codex.md Phase 3: auto-loop tick period. Initial requirement is 1 second.
constexpr int kAutoLoopPeriodMs = 1000;
// codex.md (goal-rejected fix): after this many consecutive rejected goals the
// auto-loop stops (AUTO_ERROR) instead of spamming goals + /gohome_2 forever.
constexpr int kMaxGoalRejectRetries = 3;
// The /pickplace server rejects velocity_scale > 0.2 (handle_goal). The GUI
// clamps to this before sending so a slightly-too-fast field value cannot cause
// an endless "goal rejected" loop.
constexpr double kPickPlaceServerMaxVelocity = 0.2;
// codex.md Phase 6: pre_pick Z offset the /pickplace server uses (for logging
// only — the authoritative value is the server's pre_pick_z_offset_m param).
constexpr double kPrePickZOffsetM = 0.05;

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

// codex.md (goal-rejected fix, Phase 3): normalize a quaternion before sending.
// Several GUI defaults use non-unit quaternions (e.g. the downward (1,1,0,0)),
// which are geometrically fine but should be normalized so downstream planners
// never see a non-unit orientation. Falls back to identity if degenerate.
geometry_msgs::msg::Quaternion normalizeQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  geometry_msgs::msg::Quaternion out;
  if (!std::isfinite(n) || n < 1e-9) {
    out.x = 0.0; out.y = 0.0; out.z = 0.0; out.w = 1.0;
    return out;
  }
  out.x = q.x / n;
  out.y = q.y / n;
  out.z = q.z / n;
  out.w = q.w / n;
  return out;
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
  const QString & slot_key = QString(),
  const std::function<void(rclcpp_action::ResultCode)> & on_terminal = nullptr,
  const std::function<void()> & on_accepted = nullptr,
  const std::function<void()> & on_rejected = nullptr)
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
    // codex.md (busy-lock fix): a missing action server is a terminal failure —
    // release the lock here or the GUI stays busy forever.
    if (on_rejected) {
      on_rejected();
    }
    return;
  }

  typename Client::SendGoalOptions options;
  options.goal_response_callback =
    [label, log, controller, slot_key, client, on_accepted, on_rejected](
      const typename GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        log(QString("%1: goal rejected.").arg(label));
        // codex.md (busy-lock fix): rejected goals never reach result_callback,
        // so the lock MUST be released right here.
        if (on_rejected) {
          on_rejected();
        }
        return;
      }
      log(QString("%1: goal accepted.").arg(label));
      if (on_accepted) {
        on_accepted();
      }
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
    [label, log, controller, slot_key, on_terminal](const typename GoalHandle::WrappedResult & result) {
      log(QString("%1: %2").arg(label, resultString<ActionT>(result)));
      if (controller != nullptr && !slot_key.isEmpty()) {
        controller->clearCancelHandle(slot_key);
      }
      // codex.md Phase 3 / busy-lock fix: release the PickPlace/PickPlace RL
      // target lock on ANY terminal state (succeeded/aborted/canceled).
      if (on_terminal) {
        on_terminal(result.code);
      }
    };

  client->async_send_goal(goal, options);
}

}  // namespace

const char * autoLoopStateToString(AutoLoopState state)
{
  switch (state) {
    case AutoLoopState::IDLE: return "AUTO_IDLE";
    case AutoLoopState::MOVING_HOME_2: return "AUTO_MOVING_HOME_2";
    case AutoLoopState::WAITING_DETECTION: return "AUTO_WAITING_DETECTION";
    case AutoLoopState::DISPATCHING_TASK: return "AUTO_DISPATCHING_TASK";
    case AutoLoopState::TASK_RUNNING: return "AUTO_TASK_RUNNING";
    case AutoLoopState::STOPPED: return "AUTO_STOPPED";
    case AutoLoopState::AUTO_ERROR: return "AUTO_ERROR";
  }
  return "AUTO_UNKNOWN";
}

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
  connectButton("btnPickPlaceVisionStop", [this]() {
    // Stop also disables the auto-loop (codex.md Phase 3 step 3) but does not
    // itself force-cancel a running goal beyond the existing safe cancel.
    if (vision_loop_.enabled.load()) {
      setAutoLoopEnabled(vision_loop_, false, "Stop button");
    }
    requestCancel("PickPlaceVision", "Pick Place Vision");
  });

  connectButton("btnPickPlaceRLPlan", [this]() {sendDrlPickPlace(false);});
  connectButton("btnPickPlaceRLStart", [this]() {sendDrlPickPlace(true);});
  connectButton("btnPickPlaceRLStop", [this]() {
    if (rl_loop_.enabled.load()) {
      setAutoLoopEnabled(rl_loop_, false, "Stop button");
    }
    requestCancel("PickPlaceRL", "Pick Place RL");
  });

  connectButton("btnCheckBoardPlan", [this]() {sendCheckerBoard(false);});
  connectButton("btnCheckBoardStart", [this]() {sendCheckerBoard(true);});
  connectButton("btnCheckBoardStop", [this]() {requestCancel("CheckBoard", "Check Board");});

  connectButton("btnRepeatPlan", [this]() {sendRepeatabilityTest(false);});
  connectButton("btnRepeatStart", [this]() {sendRepeatabilityTest(true);});
  connectButton("btnRepeatStop", [this]() {requestCancel("Repeatability", "Repeatability Test");});
}

void TaskActionController::configureUi()
{
  // codex.md (GUI log font resize): the log area was hard to read at 8 pt, so
  // double it to 16 pt. Kept monospace ("DejaVu Sans Mono") so columns line up.
  // Display-only — no PickPlace/auto-loop/action logic changes. QPlainTextEdit /
  // QTextEdit keep their own scrollbars, so a larger font just shows fewer lines
  // at once and scrolls, without breaking the layout.
  constexpr int kLogFontPt = 16;  // 2 x the previous 8 pt

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
    // Ensure a vertical scrollbar is available now that fewer lines fit.
    action_log->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    QFont log_font("DejaVu Sans Mono");
    log_font.setPointSize(kLogFontPt);
    action_log->setFont(log_font);
    action_log->setStyleSheet(
      "QPlainTextEdit#txtActionLog {"
      "font-family: 'DejaVu Sans Mono';"
      "font-size: 16pt;"
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
    log_font.setPointSize(kLogFontPt);
    fallback_label->setFont(log_font);
    fallback_label->setStyleSheet(
      "QLabel#txtMainLog {"
      "font-family: 'DejaVu Sans Mono';"
      "font-size: 16pt;"
      "color: #111111;"
      "background-color: white;"
      "padding: 6px;"
      "}");
  }

  // Secondary console log (QTextEdit in robot_gui.ui) — keep it in sync so every
  // log surface uses the same doubled, monospace font (codex.md point 5).
  if (auto * ros2_log = root_->findChild<QTextEdit *>("txtROS2Log")) {
    QFont log_font("DejaVu Sans Mono");
    log_font.setPointSize(kLogFontPt);
    ros2_log->setFont(log_font);
    ros2_log->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
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

  // codex.md Phase 2: MANUAL/AUTO selector for the regular PickPlace tab. Kept
  // exactly as before (codex.md: do NOT remove the existing PickPlace mode) —
  // this tab has no auto-loop. Default MANUAL so changing the mode never moves
  // the robot on its own (motion only on Start/Plan).
  addModeComboIfMissing("tabPickPlace", "cbPickPlaceMode");

  // codex.md Phase 2/3/4/8: PickPlaceVision is now the primary auto/manual tab.
  // Add its MANUAL/AUTO combo, Start/Stop Auto toggle and gripper spinbox in
  // code, and the same set to PickPlaceRL (which already had cbPickPlaceRLMode).
  setupPickPlaceAutoControls(
    "tabPickPlaceVision", "cbPickPlaceVisionMode", "btnPickPlaceVisionAuto",
    "spinPickPlaceVisionGripper", kDefaultPickGripperMm, vision_loop_);
  setupPickPlaceAutoControls(
    "tabPickPlaceRL", "cbPickPlaceRLMode", "btnPickPlaceRLAuto",
    "spinPickPlaceRLGripper", kDefaultGripperCloseMm, rl_loop_);

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
  // codex.md (AutoPlan fail cooldown): toggling Auto Plan OFF/ON resets the
  // same-target fail cache so a fresh session always plans at least once.
  auto_plan_fail_valid_ = false;
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

  // codex.md (AutoPlan fail cooldown): if this (near-identical) target just failed,
  // do NOT re-send a goal until the cooldown elapses — avoids spamming the planner.
  if (autoPlanTargetInFailCooldown(x, y, z)) {
    appendActionLog(QString(
        "[MovePoseRL AutoPlan] skip_reason=RECENT_SAME_TARGET_FAILED target=(%1, %2, %3) "
        "same_target_threshold_m=%4 fail_cooldown_s=%5")
      .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3)
      .arg(kAutoPlanSameTargetThresholdM, 0, 'f', 3).arg(kAutoPlanFailCooldownSec, 0, 'f', 1));
    return;
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

  dispatchMovePoseRlGoal(goal, false, /*from_auto_plan=*/true);
}

// codex.md (AutoPlan fail cooldown): skip re-sending a target that just failed.
bool TaskActionController::autoPlanTargetInFailCooldown(double x, double y, double z) const
{
  if (!auto_plan_fail_valid_) {
    return false;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - auto_plan_fail_time_).count();
  if (elapsed >= kAutoPlanFailCooldownSec) {
    return false;  // cooldown expired -> allow re-plan
  }
  const double dx = x - auto_plan_fail_x_;
  const double dy = y - auto_plan_fail_y_;
  const double dz = z - auto_plan_fail_z_;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  return dist <= kAutoPlanSameTargetThresholdM;  // same target within threshold
}

void TaskActionController::recordAutoPlanFailure(double x, double y, double z)
{
  auto_plan_fail_valid_ = true;
  auto_plan_fail_x_ = x;
  auto_plan_fail_y_ = y;
  auto_plan_fail_z_ = z;
  auto_plan_fail_time_ = std::chrono::steady_clock::now();
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
    // codex.md: aim at an approach point ABOVE the wood, not the raw (low) centre.
    // Only Z is offset; X/Y stay on the wood. Manual GUI targets are never offset.
    const double raw_wood_x = wt.x_m;
    const double raw_wood_y = wt.y_m;
    const double raw_wood_z = wt.z_m;
    const double approach_offset_z = kMovePoseRlVisionApproachZOffsetM;
    x = raw_wood_x;
    y = raw_wood_y;
    z = raw_wood_z + approach_offset_z;
    source = "vision_wood_approach";
    appendActionLog(QString("%1 raw_wood_base=(%2, %3, %4)")
      .arg(prefix).arg(raw_wood_x, 0, 'f', 3).arg(raw_wood_y, 0, 'f', 3).arg(raw_wood_z, 0, 'f', 3));
    appendActionLog(QString("%1 vision_target_offset=(0.000, 0.000, %2)")
      .arg(prefix).arg(approach_offset_z, 0, 'f', 3));
    appendActionLog(QString("%1 target_after_offset=(%2, %3, %4)")
      .arg(prefix).arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
    appendActionLog(QString("%1 target_source=vision_wood_approach offset_source=default")
      .arg(prefix));
    if (approach_offset_z <= 0.0) {
      appendActionLog(QString(
          "%1 WARNING: MovePoseRL vision target uses raw wood center; "
          "low-Z target may fail planning/execution").arg(prefix));
    }
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

// ---------------------------------------------------------------------------
// codex.md Phase 2/3: MANUAL/AUTO mode helpers (shared by PickPlace + RL).
// ---------------------------------------------------------------------------

void TaskActionController::addModeComboIfMissing(
  const QString & tab_name, const QString & combo_name)
{
  if (root_->findChild<QComboBox *>(combo_name)) {
    return;
  }
  auto * tab = root_->findChild<QWidget *>(tab_name);
  if (tab == nullptr) {
    appendActionLog(QString("Không tìm thấy tab %1 để thêm mode combo.").arg(tab_name));
    return;
  }
  auto * combo = new QComboBox(tab);
  combo->setObjectName(combo_name);
  combo->addItem("MANUAL");
  combo->addItem("AUTO");
  combo->setCurrentIndex(0);  // MANUAL default — never auto-moves the robot.
  combo->setGeometry(212, 58, 96, 34);
  combo->show();
  // Selecting a mode only changes state; it must not send any goal.
  connect(combo, &QComboBox::currentTextChanged, this, [this, combo_name](const QString & text) {
    appendActionLog(QString("[%1] mode = %2 (chỉ đổi trạng thái, robot chạy khi bấm Start/Plan)")
      .arg(combo_name, text));
  });
}

PickPlaceMode TaskActionController::currentPickPlaceMode(const QString & combo_name) const
{
  if (auto * combo = root_->findChild<QComboBox *>(combo_name)) {
    return pickPlaceModeFromString(combo->currentText().toStdString());
  }
  return PickPlaceMode::MANUAL;
}

// codex.md Phase 2/3/4/8: build the MANUAL/AUTO combo, the Start/Stop Auto
// toggle and the gripper close-width spinbox for one Pick/Place tab, entirely in
// code so robot_gui.ui is untouched and nothing is lost on a rebuild.
void TaskActionController::setupPickPlaceAutoControls(
  const QString & tab_name, const QString & mode_combo, const QString & auto_button,
  const QString & gripper_spin, double default_gripper_mm, AutoLoopContext & loop)
{
  auto * tab = root_->findChild<QWidget *>(tab_name);
  if (tab == nullptr) {
    appendActionLog(QString("Không tìm thấy tab %1 để thêm auto controls.").arg(tab_name));
    return;
  }
  // source_tab / action_type = tab name without the leading "tab".
  loop.source_tab = tab_name.startsWith("tab") ? tab_name.mid(3) : tab_name;
  loop.mode_combo = mode_combo;
  loop.auto_button = auto_button;

  // MANUAL/AUTO combo (default MANUAL — never auto-moves the robot on its own).
  addModeComboIfMissing(tab_name, mode_combo);
  if (auto * combo = tab->findChild<QComboBox *>(mode_combo)) {
    combo->setGeometry(118, 44, 66, 28);
  }

  // Start/Stop Auto toggle. Default OFF; only turning it ON arms the 1 s loop.
  if (tab->findChild<QAbstractButton *>(auto_button) == nullptr) {
    auto * btn = new QPushButton("Auto: OFF", tab);
    btn->setObjectName(auto_button);
    btn->setCheckable(true);
    btn->setChecked(false);
    btn->setGeometry(188, 44, 74, 28);
    btn->setStyleSheet(
      "QPushButton{background:#eef2f4; color:#555555; border:1px solid #b8cfd8; border-radius:4px;} "
      "QPushButton:checked{background:#01BABE; color:white; font-weight:600; border:1px solid #01BABE;}");
    btn->show();
    connect(btn, &QPushButton::toggled, this, [this, &loop](bool checked) {
      setAutoLoopEnabled(loop, checked, checked ? "button ON" : "button OFF");
    });
  }

  // Gripper close-width spinbox (mm). Self-describing via the " mm" suffix.
  if (tab->findChild<QDoubleSpinBox *>(gripper_spin) == nullptr) {
    auto * spin = new QDoubleSpinBox(tab);
    spin->setObjectName(gripper_spin);
    spin->setRange(0.0, 60.0);
    spin->setDecimals(1);
    spin->setSingleStep(1.0);
    spin->setSuffix(" mm");
    spin->setValue(default_gripper_mm);
    spin->setToolTip("Gripper close width (mm) sent to the pick/place goal");
    spin->setGeometry(266, 44, 66, 28);
    spin->show();
  }

  // Shared 1 s auto-loop timer for this tab.
  if (loop.timer == nullptr) {
    loop.timer = new QTimer(this);
    loop.timer->setSingleShot(false);
    connect(loop.timer, &QTimer::timeout, this, [this, &loop]() {onAutoLoopTick(loop);});
  }

  // Auto-off when the user leaves this tab so we never keep dispatching in the
  // background for a tab the user is no longer looking at.
  if (auto * tabs = root_->findChild<QTabWidget *>("taskModeTabs")) {
    connect(tabs, &QTabWidget::currentChanged, this, [this, tabs, tab_name, &loop](int index) {
      QWidget * page = tabs->widget(index);
      const bool on_tab = page != nullptr && page->objectName() == tab_name;
      if (!on_tab && loop.enabled.load()) {
        setAutoLoopEnabled(loop, false, "left tab");
      }
    });
  }
}

double TaskActionController::pickPlaceGripperMeters(
  const QString & spin_name, double default_mm, QString & source) const
{
  if (auto * spin = root_->findChild<QDoubleSpinBox *>(spin_name)) {
    source = "GUI";
    return spin->value() / 1000.0;
  }
  source = "DEFAULT";
  return default_mm / 1000.0;
}

// codex.md Phase 3/4: turn the 1 s auto-loop for one tab on or off. Turning it
// on never happens implicitly — only the Start Auto toggle calls this with on=true.
void TaskActionController::setAutoLoopEnabled(
  AutoLoopContext & loop, bool on, const QString & reason)
{
  if (auto * btn = root_->findChild<QAbstractButton *>(loop.auto_button)) {
    const QSignalBlocker blocker(btn);  // don't re-enter via toggled()
    btn->setChecked(on);
    btn->setText(on ? "Auto: ON" : "Auto: OFF");
  }
  loop.enabled.store(on);
  if (on) {
    loop.need_home_2.store(true);  // first cycle always parks at home_2 first
    loop.try_detect_from_current_pose.store(false);
    loop.last_dispatch_from_current_pose.store(false);
    loop.consecutive_goal_reject_count.store(0);  // fresh start, clear reject count
    appendActionLog(QString(
        "[%1] auto_loop_enabled=true auto_loop_tick_period_sec=%2 source_tab=%1 — %3")
      .arg(loop.source_tab).arg(kAutoLoopPeriodMs / 1000.0, 0, 'f', 2).arg(reason));
    setAutoLoopState(loop, AutoLoopState::IDLE, "auto loop started");
    if (loop.timer != nullptr) {
      loop.timer->start(kAutoLoopPeriodMs);
    }
    onAutoLoopTick(loop);  // first tick immediately
  } else {
    // Stop only halts the timer. A goal already in flight keeps running and its
    // terminal callback still releases the lock (codex.md busy fix step 6); the
    // state moves to STOPPED here and the terminal keeps it STOPPED.
    if (loop.timer != nullptr && loop.timer->isActive()) {
      loop.timer->stop();
    }
    appendActionLog(QString("[%1] auto_loop_enabled=false — %2").arg(loop.source_tab, reason));
    loop.try_detect_from_current_pose.store(false);
    loop.last_dispatch_from_current_pose.store(false);
    setAutoLoopState(loop, AutoLoopState::STOPPED, "auto loop stopped");
  }
}

// One 1 s tick of the shared auto-loop. Starts a new pick cycle only when the
// arm is free (no goal running, not moving to home_2); otherwise it skips.
void TaskActionController::onAutoLoopTick(AutoLoopContext & loop)
{
  if (!loop.enabled.load()) {
    return;  // AUTO_STOPPED: do nothing
  }
  const AutoLoopState state = loop.state.load();
  appendActionLog(QString("[%1] auto_loop_tick auto_loop_state=%2")
    .arg(loop.source_tab, autoLoopStateToString(state)));

  // Never send a new goal while one is in flight (codex.md Phase 3 step 4). Once
  // the goal is accepted the state is AUTO_TASK_RUNNING, so the log reads
  // `auto_loop_state=AUTO_TASK_RUNNING skip_reason=TASK_RUNNING` (codex.md busy
  // fix) rather than staying stuck at AUTO_DISPATCHING_TASK.
  if (pickplace_running_.load() || state == AutoLoopState::TASK_RUNNING) {
    appendActionLog(QString("[%1] auto_loop_state=%2 skip_reason=TASK_RUNNING")
      .arg(loop.source_tab, autoLoopStateToString(state)));
    return;
  }
  if (state == AutoLoopState::MOVING_HOME_2) {
    appendActionLog(QString("[%1] skip_reason=MOVING_HOME_2").arg(loop.source_tab));
    return;
  }
  if (state == AutoLoopState::WAITING_DETECTION || state == AutoLoopState::DISPATCHING_TASK) {
    appendActionLog(QString("[%1] skip_reason=CYCLE_IN_PROGRESS").arg(loop.source_tab));
    return;
  }

  // Idle → start one cycle (always AUTO semantics: home_2 + vision snapshot).
  setAutoLoopState(loop, AutoLoopState::IDLE, "tick start cycle");
  if (loop.source_tab == "PickPlaceVision") {
    sendPickPlaceVision(true, &loop);
  } else {
    sendDrlPickPlace(true, &loop);
  }
}

std::vector<WoodCandidate> TaskActionController::collectWoodCandidates(
  const QString & action_type) const
{
  std::vector<WoodCandidate> candidates;
  std::string err;
  if (!node_->wood_candidates_base("base_link", candidates, err)) {
    // No fresh WoodArray at all (mock without camera). Not an error here —
    // AUTO will report NO_WOOD_DETECTED, MANUAL falls back to default/GUI.
    return candidates;
  }
  // Pre-filter (codex.md Phase 3 step 2): reject non-finite or out-of-workspace
  // poses. Confidence/IK/age filtering is left to the vision node; we only apply
  // the checks the GUI can do safely without a camera.
  //
  // codex.md (RL trained XY gate): the PickPlaceRL path gates on X/Y ONLY against
  // the RL-trained region (robot_drl/config.py DEFAULT_WORKSPACE_MIN/MAX) and
  // never rejects on Z — a wood outside the trained XY box is out-of-distribution
  // for the policy and must not be dispatched. PickPlaceVision keeps the original
  // X/Y/Z workspace pre-filter unchanged.
  const bool rl_path = (action_type == "PickPlaceRL");
  const RlTrainedXYBounds rl_xy{
    kRlWorkspaceMinXm, kRlWorkspaceMaxXm, kRlWorkspaceMinYm, kRlWorkspaceMaxYm};
  for (auto & c : candidates) {
    if (!c.valid) {
      continue;
    }
    if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) {
      c.valid = false;
      c.invalid_reason = "non_finite_pose";
      continue;
    }
    if (rl_path) {
      // X/Y only — Z is never a rejection reason for the RL trained gate.
      if (!woodInsideRlTrainedXY(c.x, c.y, rl_xy)) {
        c.valid = false;
        c.invalid_reason = "OUT_OF_RL_TRAINED_XY";
      }
    } else if (
      c.x < kRlWorkspaceMinXm || c.x > kRlWorkspaceMaxXm ||
      c.y < kRlWorkspaceMinYm || c.y > kRlWorkspaceMaxYm ||
      c.z < kRlWorkspaceMinZm || c.z > kRlWorkspaceMaxZm)
    {
      c.valid = false;
      c.invalid_reason = "outside_workspace";
    }
  }
  return candidates;
}

// codex.md (RL trained XY gate): surface every wood the PickPlaceRL X/Y gate
// rejected. Only OUT_OF_RL_TRAINED_XY candidates are logged (Vision uses a
// different reason), so this is a no-op on the PickPlaceVision path.
void TaskActionController::logRlTrainedXyRejects(
  const QString & source_tab, const std::vector<WoodCandidate> & candidates)
{
  for (const auto & c : candidates) {
    if (c.invalid_reason == "OUT_OF_RL_TRAINED_XY") {
      appendActionLog(QString(
          "[%1] OUT_OF_RL_TRAINED_XY wood_id=%2 pose_xy=(%3, %4) "
          "trained_x=[%5, %6] trained_y=[%7, %8] (Z not checked)")
        .arg(source_tab).arg(c.wood_id)
        .arg(c.x, 0, 'f', 4).arg(c.y, 0, 'f', 4)
        .arg(kRlWorkspaceMinXm, 0, 'f', 3).arg(kRlWorkspaceMaxXm, 0, 'f', 3)
        .arg(kRlWorkspaceMinYm, 0, 'f', 3).arg(kRlWorkspaceMaxYm, 0, 'f', 3));
    }
  }
}

void TaskActionController::logResolution(
  const QString & source_tab, PickPlaceMode mode, bool home2_used, bool from_auto,
  double gripper_m, const QString & gripper_source, const QString & snapshot_time,
  const PickPlaceResolution & r)
{
  appendActionLog(QString("[%1] source_tab=%1 pickplace_mode=%2 from_auto_loop=%3 home_2_used=%4")
    .arg(source_tab, pickPlaceModeToString(mode))
    .arg(from_auto ? "true" : "false").arg(home2_used ? "true" : "false"));
  appendActionLog(QString(
      "[%1] number_of_wood_detected=%2 number_of_valid_wood=%3 selected_wood_id=%4 "
      "selected_reason=%5 distance_to_place_xy=%6")
    .arg(source_tab).arg(r.number_of_wood_detected).arg(r.number_of_valid_wood)
    .arg(r.selected_wood_id)
    .arg(r.selected_reason.empty() ? QString("(none)") : QString::fromStdString(r.selected_reason))
    .arg(r.distance_to_place_xy, 0, 'f', 4));
  appendActionLog(QString(
      "[%1] pick_pose_source=%2 place_pose_source=%3 target_locked=%4")
    .arg(source_tab)
    .arg(QString::fromStdString(r.pick_pose_source), QString::fromStdString(r.place_pose_source))
    .arg(pickplace_running_.load() ? "true" : "false"));
  // codex.md Phase 5: yaw provenance of the pick orientation.
  appendActionLog(QString(
      "[%1] target_has_yaw=%2 target_yaw_rad=%3 pick_orientation_source=%4")
    .arg(source_tab)
    .arg(r.target_has_yaw ? "true" : "false")
    .arg(r.target_yaw_rad, 0, 'f', 4)
    .arg(r.pick_orientation_source.empty() ?
      QString("(none)") : QString::fromStdString(r.pick_orientation_source)));
  // codex.md Phase 6/8: pre_pick offset + gripper command actually dispatched.
  appendActionLog(QString(
      "[%1] pre_pick_z_offset_m=%2 gripper_command=%3 gripper_source=%4 gripper_unit=m")
    .arg(source_tab).arg(kPrePickZOffsetM, 0, 'f', 3)
    .arg(gripper_m, 0, 'f', 4).arg(gripper_source));
  // codex.md Phase 7: single vision snapshot locked at dispatch; the pick pose is
  // never re-read from /vision/wood_objects while the goal runs.
  appendActionLog(QString(
      "[%1] vision_snapshot_time=%2 selected_wood_pose_at_dispatch=(%3, %4, %5) "
      "target_updated_during_execution=false")
    .arg(source_tab, snapshot_time)
    .arg(r.pick_pose.position.x, 0, 'f', 4)
    .arg(r.pick_pose.position.y, 0, 'f', 4)
    .arg(r.pick_pose.position.z, 0, 'f', 4));
  if (!r.ok) {
    appendActionLog(QString("[%1] fail_reason=%2")
      .arg(source_tab, QString::fromStdString(r.fail_reason)));
  }
}

bool TaskActionController::tryLockPickPlace(const QString & action_type)
{
  bool expected = false;
  if (!pickplace_running_.compare_exchange_strong(expected, true)) {
    appendActionLog(QString(
        "[%1] target_locked=true: một lần PickPlace/PickPlaceRL đang chạy, bỏ qua Start mới")
      .arg(action_type));
    return false;
  }
  appendActionLog(QString("[%1] target_locked=true reason=DISPATCH_START action=%1")
    .arg(action_type));
  return true;
}

void TaskActionController::unlockPickPlace()
{
  pickplace_running_.store(false);
}

// codex.md (busy-lock fix): log every auto-loop state transition so a stuck
// state (e.g. AUTO_DISPATCHING_TASK held forever) is visible in the log.
void TaskActionController::setAutoLoopState(
  AutoLoopContext & loop, AutoLoopState next, const QString & reason)
{
  const AutoLoopState prev = loop.state.exchange(next);
  if (prev != next) {
    appendActionLog(QString("[%1] auto_loop_state_change old=%2 new=%3 reason=%4")
      .arg(loop.source_tab, autoLoopStateToString(prev), autoLoopStateToString(next), reason));
  }
}

// codex.md (busy-lock fix + post-place direct detect): the single result-aware
// terminal path. Idempotent unlock (only when this goal held the lock). On
// SUCCEEDED it arms a direct detection at the current pose (skip /gohome_2 next
// tick); on any non-success it forces a re-home before the next detection.
void TaskActionController::onPickPlaceGoalTerminal(
  AutoLoopContext * loop, bool locked, const QString & tag,
  bool succeeded, const QString & code_str)
{
  if (locked) {
    unlockPickPlace();
  }
  appendActionLog(QString(
      "[%1] action_result_terminal code=%2 target_locked=false pickplace_running=false")
    .arg(tag, code_str));
  if (!loop) {
    return;  // manual Start/Plan: no loop policy to update.
  }
  appendActionLog(QString("[%1] previous_goal_result=%2").arg(tag, code_str));
  if (succeeded) {
    // Robot ended at the place pose. Try detecting more wood there before homing.
    loop->need_home_2.store(false);
    loop->try_detect_from_current_pose.store(true);
    loop->last_dispatch_from_current_pose.store(false);
    appendActionLog(QString("[%1] post_place_direct_detect=true reason=previous_goal_succeeded")
      .arg(tag));
  } else {
    // ABORTED / CANCELED / other: robot position is uncertain -> re-home first.
    loop->need_home_2.store(true);
    loop->try_detect_from_current_pose.store(false);
    loop->last_dispatch_from_current_pose.store(false);
    appendActionLog(QString("[%1] post_place_direct_detect=false reason=previous_goal_not_succeeded")
      .arg(tag));
    appendActionLog(QString("[%1] need_home_2=true reason=PREVIOUS_GOAL_NOT_SUCCEEDED").arg(tag));
  }
  setAutoLoopState(
    *loop, loop->enabled.load() ? AutoLoopState::IDLE : AutoLoopState::STOPPED,
    "goal terminal");
}

// codex.md (goal-rejected fix): a goal was ACCEPTED. The arm may now leave
// home_2, so the next auto cycle must re-home before detecting; reset the reject
// counter and move the loop to AUTO_TASK_RUNNING.
void TaskActionController::onPickPlaceGoalAccepted(
  AutoLoopContext * loop, const QString & tag, const QString & action)
{
  if (loop) {
    loop->need_home_2.store(true);
    loop->try_detect_from_current_pose.store(false);
    loop->consecutive_goal_reject_count.store(0);
    setAutoLoopState(*loop, AutoLoopState::TASK_RUNNING, "goal accepted");
  }
  appendActionLog(QString("[%1] action_goal_accepted action=%2 auto_loop_state=%3")
    .arg(tag, action, loop ? "AUTO_TASK_RUNNING" : "n/a"));
}

// codex.md (goal-rejected fix + post-place policy): a goal was REJECTED before
// the arm moved. If it was dispatched from home_2, keep need_home_2=false and
// retry without re-homing; if it was dispatched from a post-place direct-detect
// pose, force re-home before the next detection. Past the retry limit, stop the
// loop with AUTO_ERROR instead of spamming goals.
void TaskActionController::onPickPlaceGoalRejected(
  AutoLoopContext * loop, bool locked, const QString & tag, const QString & action)
{
  if (locked) {
    unlockPickPlace();
  }
  appendActionLog(QString(
      "[%1] action_goal_rejected action=%2 target_locked=false pickplace_running=false")
    .arg(tag, action));

  if (!loop) {
    return;  // manual Start/Plan: no loop state, nothing more to do.
  }
  appendActionLog(QString("[%1] previous_goal_result=REJECTED").arg(tag));
  appendActionLog(QString("[%1] post_place_direct_detect=false reason=previous_goal_rejected")
    .arg(tag));
  loop->try_detect_from_current_pose.store(false);
  const bool rejected_after_direct_detect = loop->last_dispatch_from_current_pose.exchange(false);
  // The goal never reached execution. If it was sent from home_2, keep the old
  // no-rehome retry behavior. If it was sent after a post-place direct detect,
  // the current pose is not trusted, so re-home before trying vision again.
  loop->need_home_2.store(rejected_after_direct_detect);
  if (rejected_after_direct_detect) {
    appendActionLog(QString("[%1] need_home_2=true reason=PREVIOUS_GOAL_NOT_SUCCEEDED").arg(tag));
  }
  const int count = loop->consecutive_goal_reject_count.fetch_add(1) + 1;
  if (count >= kMaxGoalRejectRetries) {
    appendActionLog(QString(
        "[%1] fail_reason=PICKPLACE_GOAL_REJECTED_RETRY_LIMIT consecutive_goal_reject_count=%2")
      .arg(tag).arg(count));
    setAutoLoopEnabled(*loop, false, "goal rejected retry limit");
    setAutoLoopState(*loop, AutoLoopState::AUTO_ERROR, "reject retry limit reached");
  } else {
    if (rejected_after_direct_detect) {
      appendActionLog(QString(
          "[%1] consecutive_goal_reject_count=%2 (retry next tick after re-home; current pose not trusted)")
        .arg(tag).arg(count));
    } else {
      appendActionLog(QString(
          "[%1] consecutive_goal_reject_count=%2 (retry next tick, no re-home, robot still at home_2)")
        .arg(tag).arg(count));
    }
    setAutoLoopState(
      *loop, loop->enabled.load() ? AutoLoopState::IDLE : AutoLoopState::STOPPED,
      "goal rejected, retry");
  }
}

void TaskActionController::goHome2ThenRun(
  const QString & action_type, bool execute, std::function<void()> on_home_ok,
  std::function<void()> on_home_fail)
{
  auto node = node_;
  QPointer<TaskActionController> self(this);
  const QString at = action_type;
  std::thread([self, node, at, execute, on_home_ok, on_home_fail]() {
    using GoalHandle = rclcpp_action::ClientGoalHandle<GoHome>;
    using Client = rclcpp_action::Client<GoHome>;
    static std::vector<std::shared_ptr<void>> keep_clients_alive;
    auto fail = [self, execute, on_home_fail]() {
      if (!self) {
        return;
      }
      // Only an execute cycle holds the lock (plan-only never locks), so only it
      // may release it — a plan-only home_2 failure must not free a concurrent
      // Start's lock (codex.md busy-lock fix).
      if (execute) {
        self->unlockPickPlace();
      }
      if (on_home_fail) {
        on_home_fail();
      }
    };
    if (!self) {
      return;
    }
    auto client = rclcpp_action::create_client<GoHome>(node, "/gohome_2");
    keep_clients_alive.push_back(client);

    self->appendActionLog(QString("[%1] AUTO: về home_2 trước (/gohome_2)...").arg(at));
    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
      self->appendActionLog(QString("[%1] fail_reason=HOME_2_PLAN_FAILED: /gohome_2 "
        "action server không sẵn sàng.").arg(at));
      fail();
      return;
    }

    GoHome::Goal goal;
    goal.start = true;
    goal.execute = execute;

    typename Client::SendGoalOptions options;
    options.goal_response_callback =
      [self, at, fail](const GoalHandle::SharedPtr & gh) {
        if (self && !gh) {
          self->appendActionLog(QString("[%1] fail_reason=HOME_2_PLAN_FAILED: goal rejected.").arg(at));
          fail();
        }
      };
    options.feedback_callback =
      [self, at](GoalHandle::SharedPtr, const std::shared_ptr<const GoHome::Feedback> fb) {
        if (self && fb) {
          self->appendActionLog(QString("[%1] home_2: %2").arg(at, feedbackString(*fb)));
        }
      };
    options.result_callback =
      [self, at, execute, on_home_ok, fail](const GoalHandle::WrappedResult & result) {
        if (!self) {
          return;
        }
        const bool ok = result.result && result.result->success &&
          result.code == rclcpp_action::ResultCode::SUCCEEDED;
        if (ok) {
          self->appendActionLog(QString("[%1] home_2 reached; bắt đầu lấy vision/wood.").arg(at));
          on_home_ok();
        } else {
          const QString reason = execute ? "HOME_2_EXEC_FAILED" : "HOME_2_PLAN_FAILED";
          const QString msg = result.result ?
            QString::fromStdString(result.result->message) : QString("no result");
          self->appendActionLog(QString("[%1] fail_reason=%2: %3").arg(at, reason, msg));
          fail();
        }
      };
    client->async_send_goal(goal, options);
  }).detach();
}

// codex.md Phase 2/3/5: the regular PickPlace tab now runs through the shared
// MANUAL/AUTO resolver. MANUAL keeps the previous behaviour (GUI pick/place or
// defaults, wood used only when no GUI pick is typed); AUTO goes to home_2
// first, then requires a wood detection (no default-pick fallback).
void TaskActionController::sendPickPlace(bool execute)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  const PickPlaceMode mode = currentPickPlaceMode("cbPickPlaceMode");

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

  PickPlaceResolveInput in;
  in.mode = mode;
  in.has_gui_pick =
    editHasText(root_, "pickPoseX") || editHasText(root_, "pickPoseY") || editHasText(root_, "pickPoseZ");
  in.gui_pick_pose = makePose(*pick_x, *pick_y, *pick_z, pick_q);
  in.gui_pick_has_yaw = editHasText(root_, "pickPoseYaw");
  in.has_gui_place =
    editHasText(root_, "placePoseX") || editHasText(root_, "placePoseY") || editHasText(root_, "placePoseZ");
  in.gui_place_pose = makePose(*place_x, *place_y, *place_z, place_q);
  in.default_pick_pose = makePose(
    kDefaultPickXMm / 1000.0, kDefaultPickYMm / 1000.0, kDefaultPickZMm / 1000.0, pick_q);
  in.fixed_place_pose = makePose(
    kDefaultPlaceXMm / 1000.0, kDefaultPlaceYMm / 1000.0, kDefaultPlaceZMm / 1000.0, place_q);

  const bool enable_log = isLogEnabled("chkPickPlaceLog");
  // codex.md (goal-rejected fix): clamp to the /pickplace server max (0.2) so a
  // faster velocity field cannot cause a "goal rejected" loop.
  double vel = *velocity;
  if (vel > kPickPlaceServerMaxVelocity) {
    appendActionLog(QString(
        "[Pick Place] velocity_scale=%1 > %2 (server /pickplace max) -> clamped to %2")
      .arg(vel, 0, 'f', 3).arg(kPickPlaceServerMaxVelocity, 0, 'f', 3));
    vel = kPickPlaceServerMaxVelocity;
  }

  // Builds and sends the /pickplace goal from a resolution. `locked` true means
  // release the target lock when the goal terminates.
  auto dispatch = [this, log, gripper, vel, enable_log, execute](
    const PickPlaceResolution & r, bool locked) {
    PickPlace::Goal goal;
    goal.pose_pick = r.pick_pose;
    goal.pose_pick.orientation = normalizeQuaternion(goal.pose_pick.orientation);
    goal.pose_place = r.place_pose;
    goal.pose_place.orientation = normalizeQuaternion(goal.pose_place.orientation);
    goal.gripper = gripper;
    goal.velocity_scale = vel;
    goal.execute = execute;
    goal.enable_tcp_log = enable_log;
    appendActionLog(QString(
        "[PickPlace] send /pickplace goal: execute=%1 velocity_scale=%2 gripper=%3 "
        "pose_pick=(%4, %5, %6) pose_place=(%7, %8, %9)")
      .arg(goal.execute ? "true" : "false")
      .arg(goal.velocity_scale, 0, 'f', 4).arg(goal.gripper, 0, 'f', 4)
      .arg(goal.pose_pick.position.x, 0, 'f', 4).arg(goal.pose_pick.position.y, 0, 'f', 4)
      .arg(goal.pose_pick.position.z, 0, 'f', 4)
      .arg(goal.pose_place.position.x, 0, 'f', 4).arg(goal.pose_place.position.y, 0, 'f', 4)
      .arg(goal.pose_place.position.z, 0, 'f', 4));
    // codex.md busy-lock fix: the regular PickPlace tab has no auto-loop, but it
    // still takes the lock on Start, so it needs the same rejected/terminal
    // release paths (loop == nullptr).
    std::function<void(rclcpp_action::ResultCode)> on_terminal =
      [this, locked](rclcpp_action::ResultCode code) {
        onPickPlaceGoalTerminal(
          nullptr, locked, "PickPlace",
          code == rclcpp_action::ResultCode::SUCCEEDED, resultCodeToString(code));
      };
    std::function<void()> on_rejected = [this, locked]() {
      onPickPlaceGoalRejected(nullptr, locked, "PickPlace", "PickPlace");
    };
    sendGoal<PickPlace>(
      node_, "/pickplace", execute ? "Pick Place Start" : "Pick Place Plan", goal, log,
      this, "PickPlace", on_terminal, /*on_accepted=*/nullptr, on_rejected);
  };

  if (mode == PickPlaceMode::AUTO) {
    // Lock first so a second Start cannot race the home_2 -> vision -> pick chain.
    if (execute && !tryLockPickPlace("PickPlace")) {
      return;
    }
    goHome2ThenRun("PickPlace", execute, [this, in, dispatch, execute, gripper]() {
      // Re-read wood candidates AFTER home_2 is reached (codex.md Phase 2 AUTO).
      PickPlaceResolveInput auto_in = in;
      auto_in.wood_candidates = collectWoodCandidates("PickPlace");
      const QString snap = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
      const PickPlaceResolution r = resolvePickPlaceTargets(auto_in);
      logResolution("PickPlace", PickPlaceMode::AUTO, /*home2_used=*/true, /*from_auto=*/false,
        gripper, "DEFAULT", snap, r);
      if (!r.ok) {
        if (execute) {
          unlockPickPlace();
        }
        return;
      }
      dispatch(r, /*locked=*/execute);
    });
    return;
  }

  // MANUAL — resolve immediately, no home_2 move.
  in.wood_candidates = collectWoodCandidates("PickPlace");
  const QString snap = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  const PickPlaceResolution r = resolvePickPlaceTargets(in);
  logResolution("PickPlace", PickPlaceMode::MANUAL, /*home2_used=*/false, /*from_auto=*/false,
    gripper, "DEFAULT", snap, r);
  if (!r.ok) {
    return;
  }
  const bool locked = execute && tryLockPickPlace("PickPlace");
  dispatch(r, locked);
}

// codex.md Phase 3/4: shared AUTO cycle for PickPlaceVision + PickPlaceRL.
void TaskActionController::runAutoPickCycle(
  const QString & source_tab, bool execute, AutoLoopContext * loop,
  const PickPlaceResolveInput & in, double gripper_m, const QString & gripper_source,
  std::function<void(const PickPlaceResolution & r, bool locked)> dispatch)
{
  // Lock first so a second Start / overlapping tick cannot race the
  // home_2 -> vision -> pick chain (codex.md Phase 3 target lock).
  if (execute && !tryLockPickPlace(source_tab)) {
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::IDLE, "lock busy, skip cycle");
    }
    return;
  }

  auto detect_and_dispatch = [this, source_tab, execute, loop, in, gripper_m, gripper_source,
    dispatch](bool home2_used) {
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::WAITING_DETECTION,
        home2_used ? "home_2 reached, reading wood_objects" : "reading wood_objects at current pose");
    }

    // codex.md (post-place direct detect): consume the "try at current pose" flag
    // set by a previous SUCCEEDED place. This detection is happening WITHOUT a
    // prior /gohome_2 while the arm is still at the place pose.
    const bool direct = loop != nullptr && loop->try_detect_from_current_pose.exchange(false);
    if (direct) {
      appendActionLog(QString("[%1] direct_detect_from_current_pose=true").arg(source_tab));
    }

    // Single vision snapshot at the moment of resolve (codex.md Phase 7).
    PickPlaceResolveInput auto_in = in;
    auto_in.wood_candidates = collectWoodCandidates(source_tab);
    logRlTrainedXyRejects(source_tab, auto_in.wood_candidates);
    const QString snap = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
    const PickPlaceResolution r = resolvePickPlaceTargets(auto_in);
    logResolution(source_tab, PickPlaceMode::AUTO, home2_used, /*from_auto=*/loop != nullptr,
      gripper_m, gripper_source, snap, r);
    if (direct) {
      appendActionLog(QString("[%1] wood_detected_before_home2=%2 wood_valid_before_home2=%3")
        .arg(source_tab).arg(r.number_of_wood_detected).arg(r.number_of_valid_wood));
    }

    if (!r.ok) {
      if (direct) {
        // Post-place direct detect found nothing at the current pose -> re-home on
        // the next tick and detect again from home_2.
        appendActionLog(QString("[%1] direct_detect_result=NO_WOOD").arg(source_tab));
        if (loop) {
          loop->need_home_2.store(true);
        }
        appendActionLog(QString("[%1] need_home_2=true reason=NO_WOOD_FROM_CURRENT_POSE")
          .arg(source_tab));
      }
      // Otherwise the arm is still parked at home_2: keep need_home_2=false and do
      // NOT re-issue /gohome_2 every tick (codex.md Phase 3 note).
      if (execute) {
        unlockPickPlace();
      }
      if (loop) {
        setAutoLoopState(*loop, AutoLoopState::IDLE, "no valid target");
      }
      return;
    }

    if (direct) {
      appendActionLog(QString("[%1] direct_detect_result=WOOD_FOUND number_of_valid_wood=%2")
        .arg(source_tab).arg(r.number_of_valid_wood));
      appendActionLog(QString("[%1] skip_home_2_reason=WOOD_FOUND_FROM_CURRENT_POSE").arg(source_tab));
      appendActionLog(QString("[%1] dispatch next PickPlace without /gohome_2").arg(source_tab));
    }
    // DISPATCHING_TASK only covers the send window. need_home_2 becomes true only
    // once the goal is ACCEPTED (onPickPlaceGoalAccepted); a rejected goal never
    // moves the arm and must not trigger a re-home next tick.
    if (loop) {
      loop->last_dispatch_from_current_pose.store(direct);
      setAutoLoopState(*loop, AutoLoopState::DISPATCHING_TASK, "sending goal");
    }
    dispatch(r, /*locked=*/execute);
  };

  // Skip home_2 only when the loop already parked there last tick with no object.
  const bool skip_home2 = loop != nullptr && !loop->need_home_2.load();
  if (skip_home2) {
    detect_and_dispatch(/*home2_used=*/false);
    return;
  }
  if (loop) {
    setAutoLoopState(*loop, AutoLoopState::MOVING_HOME_2, "moving to home_2");
  }
  goHome2ThenRun(
    source_tab, execute,
    [this, loop, detect_and_dispatch]() {
      if (loop) {
        loop->need_home_2.store(false);
      }
      detect_and_dispatch(/*home2_used=*/true);
    },
    [this, loop]() {  // on_home_fail (unlock already done inside goHome2ThenRun)
      if (loop) {
        setAutoLoopState(*loop, AutoLoopState::IDLE, "home_2 failed");
      }
    });
}

// codex.md Phase 2/3/4/5/7/8: PickPlaceVision is the primary auto/manual vision
// tab. It runs through the SAME shared resolver + AUTO cycle as PickPlaceRL.
// MANUAL: GUI object pose (txtObjectX/Y/Z) if typed, else nearest valid wood to
// place, else the default pick. AUTO / Auto-loop: park at home_2, take one vision
// snapshot, pick the nearest valid wood to place, and dispatch /pickplace.
void TaskActionController::sendPickPlaceVision(bool execute, AutoLoopContext * loop)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  // The Auto loop always drives AUTO semantics; a manual button press follows the
  // MANUAL/AUTO combo.
  const PickPlaceMode mode = loop != nullptr ?
    PickPlaceMode::AUTO : currentPickPlaceMode("cbPickPlaceVisionMode");

  const auto velocity = readVelocityScale(
    lineEdit(root_, "txtPickPlaceVisionVelocity"),
    DEFAULT_GUI_VELOCITY_SCALE,
    "[Pick Place Vision] velocity_scale");
  const auto pick_x = readMmAsMeter(lineEdit(root_, "txtObjectX"), kDefaultPickXMm, "[Pick Place Vision] pick X");
  const auto pick_y = readMmAsMeter(lineEdit(root_, "txtObjectY"), kDefaultPickYMm, "[Pick Place Vision] pick Y");
  const auto pick_z = readMmAsMeter(lineEdit(root_, "txtObjectZ"), kDefaultPickZMm, "[Pick Place Vision] pick Z");
  const auto place_x = readMmAsMeter(lineEdit(root_, "visionPlacePoseX"), kDefaultPlaceXMm, "[Pick Place Vision] place X");
  const auto place_y = readMmAsMeter(lineEdit(root_, "visionPlacePoseY"), kDefaultPlaceYMm, "[Pick Place Vision] place Y");
  const auto place_z = readMmAsMeter(lineEdit(root_, "visionPlacePoseZ"), kDefaultPlaceZMm, "[Pick Place Vision] place Z");
  if (!velocity || !pick_x || !pick_y || !pick_z || !place_x || !place_y || !place_z) {
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::IDLE, "invalid input, skip cycle");
    }
    return;
  }
  appendActionLog(QString("[Pick Place Vision] velocity_scale=%1").arg(*velocity, 0, 'f', 3));

  bool ok = true;
  const auto place_q = orientationFromYawField(root_, "visionPlacePoseYaw", log, &ok);
  if (!ok) {
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::IDLE, "invalid input, skip cycle");
    }
    return;
  }

  QString gripper_source;
  const double gripper = pickPlaceGripperMeters(
    "spinPickPlaceVisionGripper", kDefaultPickGripperMm, gripper_source);

  PickPlaceResolveInput in;
  in.mode = mode;
  in.has_gui_pick =
    editHasText(root_, "txtObjectX") || editHasText(root_, "txtObjectY") || editHasText(root_, "txtObjectZ");
  in.gui_pick_pose = makePose(*pick_x, *pick_y, *pick_z, defaultQuaternion());
  in.gui_pick_has_yaw = false;  // Vision tab has no pick-yaw field
  in.has_gui_place =
    editHasText(root_, "visionPlacePoseX") || editHasText(root_, "visionPlacePoseY") ||
    editHasText(root_, "visionPlacePoseZ");
  in.gui_place_pose = makePose(*place_x, *place_y, *place_z, place_q);
  in.default_pick_pose = makePose(
    kDefaultPickXMm / 1000.0, kDefaultPickYMm / 1000.0, kDefaultPickZMm / 1000.0, defaultQuaternion());
  in.fixed_place_pose = makePose(
    kDefaultPlaceXMm / 1000.0, kDefaultPlaceYMm / 1000.0, kDefaultPlaceZMm / 1000.0, place_q);

  const bool enable_log = isLogEnabled("chkPickPlaceVisionLog");
  // codex.md (goal-rejected fix): /pickplace rejects velocity_scale > 0.2, so
  // clamp before sending to avoid an endless "goal rejected" loop.
  double vel = *velocity;
  if (vel > kPickPlaceServerMaxVelocity) {
    appendActionLog(QString(
        "[Pick Place Vision] velocity_scale=%1 > %2 (server /pickplace max) -> clamped to %2")
      .arg(vel, 0, 'f', 3).arg(kPickPlaceServerMaxVelocity, 0, 'f', 3));
    vel = kPickPlaceServerMaxVelocity;
  }

  auto dispatch = [this, log, gripper, vel, enable_log, execute, loop](
    const PickPlaceResolution & r, bool locked) {
    PickPlace::Goal goal;
    goal.pose_pick = r.pick_pose;      // orientation carries vision yaw when present
    goal.pose_pick.orientation = normalizeQuaternion(goal.pose_pick.orientation);
    goal.pose_place = r.place_pose;
    goal.pose_place.orientation = normalizeQuaternion(goal.pose_place.orientation);
    goal.gripper = gripper;
    goal.velocity_scale = vel;
    goal.execute = execute;
    goal.enable_tcp_log = enable_log;
    const QString tag = loop ? loop->source_tab : QString("PickPlaceVision");
    // codex.md (goal-rejected fix, Phase 3): log the FULL goal before sending so
    // any reject can be matched to exactly what was sent (units: m, gripper m).
    appendActionLog(QString(
        "[%1] send /pickplace goal: source_tab=%1 from_auto_loop=%2 execute=%3 "
        "velocity_scale=%4 gripper=%5 enable_tcp_log=%6")
      .arg(tag).arg(loop ? "true" : "false").arg(goal.execute ? "true" : "false")
      .arg(goal.velocity_scale, 0, 'f', 4).arg(goal.gripper, 0, 'f', 4)
      .arg(goal.enable_tcp_log ? "true" : "false"));
    appendActionLog(QString(
        "[%1]   pose_pick=(%2, %3, %4) q=(%5, %6, %7, %8)")
      .arg(tag)
      .arg(goal.pose_pick.position.x, 0, 'f', 4).arg(goal.pose_pick.position.y, 0, 'f', 4)
      .arg(goal.pose_pick.position.z, 0, 'f', 4)
      .arg(goal.pose_pick.orientation.x, 0, 'f', 4).arg(goal.pose_pick.orientation.y, 0, 'f', 4)
      .arg(goal.pose_pick.orientation.z, 0, 'f', 4).arg(goal.pose_pick.orientation.w, 0, 'f', 4));
    appendActionLog(QString(
        "[%1]   pose_place=(%2, %3, %4) q=(%5, %6, %7, %8)")
      .arg(tag)
      .arg(goal.pose_place.position.x, 0, 'f', 4).arg(goal.pose_place.position.y, 0, 'f', 4)
      .arg(goal.pose_place.position.z, 0, 'f', 4)
      .arg(goal.pose_place.orientation.x, 0, 'f', 4).arg(goal.pose_place.orientation.y, 0, 'f', 4)
      .arg(goal.pose_place.orientation.z, 0, 'f', 4).arg(goal.pose_place.orientation.w, 0, 'f', 4));
    std::function<void(rclcpp_action::ResultCode)> on_terminal =
      [this, loop, tag, locked](rclcpp_action::ResultCode code) {
        onPickPlaceGoalTerminal(
          loop, locked, tag,
          code == rclcpp_action::ResultCode::SUCCEEDED, resultCodeToString(code));
      };
    std::function<void()> on_accepted = [this, loop, tag]() {
      onPickPlaceGoalAccepted(loop, tag, "PickPlace");
    };
    std::function<void()> on_rejected = [this, loop, tag, locked]() {
      onPickPlaceGoalRejected(loop, locked, tag, "PickPlace");
    };
    sendGoal<PickPlace>(
      node_, "/pickplace", execute ? "Pick Place Vision Start" : "Pick Place Vision Plan", goal, log,
      this, "PickPlaceVision", on_terminal, on_accepted, on_rejected);
  };

  if (mode == PickPlaceMode::AUTO) {
    runAutoPickCycle("PickPlaceVision", execute, loop, in, gripper, gripper_source, dispatch);
    return;
  }

  // MANUAL — resolve immediately from the latest /vision/wood_objects snapshot,
  // no home_2 move.
  in.wood_candidates = collectWoodCandidates("PickPlaceVision");
  const QString snap = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  const PickPlaceResolution r = resolvePickPlaceTargets(in);
  logResolution(
    "PickPlaceVision", PickPlaceMode::MANUAL, /*home2_used=*/false, /*from_auto=*/false,
    gripper, gripper_source, snap, r);
  if (!r.ok) {
    return;
  }
  const bool locked = execute && tryLockPickPlace("PickPlaceVision");
  dispatch(r, locked);
}

// codex.md Phase 2/3/4/5/7/8: PickPlace RL runs through the SAME shared resolver
// AND the SAME shared AUTO cycle (runAutoPickCycle) as PickPlaceVision, so their
// MANUAL/AUTO + auto-loop behaviour cannot drift. This tab has no manual pick
// input, so MANUAL uses wood-nearest-place when available and otherwise the fixed
// default pick; AUTO / Auto-loop park at home_2 then require a wood detection.
void TaskActionController::sendDrlPickPlace(bool execute, AutoLoopContext * loop)
{
  const LogFn log = [this](const QString & msg) {appendActionLog(msg);};
  // codex.md (Plan/Start in-flight fix): block any new /drl_pickplace dispatch
  // (Plan, Start, or auto-loop) while a previous /drl_pickplace goal is still in
  // flight. Plan (execute=false) does NOT take the execute-only target lock, so
  // without this guard a Start pressed during a running Plan reached the server
  // in parallel and got rejected. This applies to both manual buttons and the
  // auto-loop tick.
  if (drl_pickplace_in_flight_.load()) {
    const QString tag = loop ? loop->source_tab : QString("PickPlaceRL");
    appendActionLog(QString(
        "[%1] skip_reason=DRL_PICKPLACE_GOAL_IN_FLIGHT execute=%2 — /drl_pickplace "
        "goal trước chưa terminal, không gửi goal mới")
      .arg(tag).arg(execute ? "true" : "false"));
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::IDLE, "drl_pickplace goal in flight");
    }
    return;
  }
  const PickPlaceMode mode = loop != nullptr ?
    PickPlaceMode::AUTO : currentPickPlaceMode("cbPickPlaceRLMode");

  const auto place_x = readMmAsMeter(lineEdit(root_, "rlPlacePoseX"), kDefaultDrlPlaceXMm, "[Pick Place RL] place X");
  const auto place_y = readMmAsMeter(lineEdit(root_, "rlPlacePoseY"), kDefaultDrlPlaceYMm, "[Pick Place RL] place Y");
  const auto place_z = readMmAsMeter(lineEdit(root_, "rlPlacePoseZ"), kDefaultDrlPlaceZMm, "[Pick Place RL] place Z");
  QString gripper_source;
  const double gripper = pickPlaceGripperMeters(
    "spinPickPlaceRLGripper", kDefaultGripperCloseMm, gripper_source);
  appendActionLog(QString("[Pick Place RL] gripper_command=%1 m gripper_source=%2")
    .arg(gripper, 0, 'f', 4).arg(gripper_source));
  if (!place_x || !place_y || !place_z) {
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::IDLE, "invalid input, skip cycle");
    }
    return;
  }

  bool ok = true;
  auto place_q = orientationFromYawField(root_, "rlPlacePoseYaw", log, &ok);
  if (!ok) {
    if (loop) {
      setAutoLoopState(*loop, AutoLoopState::IDLE, "invalid input, skip cycle");
    }
    return;
  }
  if (!editHasText(root_, "rlPlacePoseYaw")) {
    place_q = drlRepeatQuaternion();
  }

  PickPlaceResolveInput in;
  in.mode = mode;
  in.has_gui_pick = false;  // no manual pick field on the RL tab
  in.has_gui_place =
    editHasText(root_, "rlPlacePoseX") || editHasText(root_, "rlPlacePoseY") || editHasText(root_, "rlPlacePoseZ");
  in.gui_place_pose = makePose(*place_x, *place_y, *place_z, place_q);
  in.default_pick_pose = makePose(
    kDefaultDrlPickXMm / 1000.0, kDefaultDrlPickYMm / 1000.0, kDefaultDrlPickZMm / 1000.0,
    drlRepeatQuaternion());
  in.fixed_place_pose = makePose(
    kDefaultDrlPlaceXMm / 1000.0, kDefaultDrlPlaceYMm / 1000.0, kDefaultDrlPlaceZMm / 1000.0, place_q);

  const bool enable_log = isLogEnabled("chkPickPlaceRlLog");

  auto dispatch = [this, log, gripper, enable_log, execute, loop](
    const PickPlaceResolution & r, bool locked) {
    DrlPickPlace::Goal goal;
    // codex.md Phase 5: use the resolver's pick orientation — the RL default pick
    // pose is already seeded with the DRL-required downward quaternion, and a wood
    // yaw (when present) is folded in as a downward+yaw orientation (tool Z stays
    // down). No unconditional override anymore.
    geometry_msgs::msg::Pose rl_pick = r.pick_pose;
    rl_pick.orientation = normalizeQuaternion(rl_pick.orientation);
    geometry_msgs::msg::Pose rl_place = r.place_pose;
    rl_place.orientation = normalizeQuaternion(rl_place.orientation);
    goal.target_pick = makeStampedPose("base_link", rl_pick);
    goal.target_place = makeStampedPose("base_link", rl_place);
    goal.gripper_close_width_m = gripper;
    goal.execute = execute;
    goal.enable_metrics_log = enable_log;
    const QString tag = loop ? loop->source_tab : QString("PickPlaceRL");
    // codex.md (goal-rejected fix, Phase 3): log the full goal before sending.
    appendActionLog(QString(
        "[%1] send /drl_pickplace goal: source_tab=%1 from_auto_loop=%2 execute=%3 "
        "gripper_close_width_m=%4 enable_metrics_log=%5")
      .arg(tag).arg(loop ? "true" : "false").arg(goal.execute ? "true" : "false")
      .arg(goal.gripper_close_width_m, 0, 'f', 4).arg(goal.enable_metrics_log ? "true" : "false"));
    appendActionLog(QString("[%1]   target_pick=(%2, %3, %4) q=(%5, %6, %7, %8)")
      .arg(tag)
      .arg(rl_pick.position.x, 0, 'f', 4).arg(rl_pick.position.y, 0, 'f', 4)
      .arg(rl_pick.position.z, 0, 'f', 4)
      .arg(rl_pick.orientation.x, 0, 'f', 4).arg(rl_pick.orientation.y, 0, 'f', 4)
      .arg(rl_pick.orientation.z, 0, 'f', 4).arg(rl_pick.orientation.w, 0, 'f', 4));
    appendActionLog(QString("[%1]   target_place=(%2, %3, %4) q=(%5, %6, %7, %8)")
      .arg(tag)
      .arg(rl_place.position.x, 0, 'f', 4).arg(rl_place.position.y, 0, 'f', 4)
      .arg(rl_place.position.z, 0, 'f', 4)
      .arg(rl_place.orientation.x, 0, 'f', 4).arg(rl_place.orientation.y, 0, 'f', 4)
      .arg(rl_place.orientation.z, 0, 'f', 4).arg(rl_place.orientation.w, 0, 'f', 4));
    // codex.md (Plan/Start in-flight fix): mark the /drl_pickplace goal in flight
    // right before sending. Cleared on any terminal or rejection below (rejection
    // also fires when the action server is unavailable), so the guard can never
    // stay stuck busy.
    drl_pickplace_in_flight_.store(true);
    appendActionLog(QString("[%1] pickplace_rl_goal_in_flight=true reason=%2")
      .arg(tag, execute ? "START_DISPATCH" : "PLAN_DISPATCH"));
    std::function<void(rclcpp_action::ResultCode)> on_terminal =
      [this, loop, tag, locked](rclcpp_action::ResultCode code) {
        drl_pickplace_in_flight_.store(false);
        appendActionLog(QString("[%1] pickplace_rl_goal_in_flight=false result=%2")
          .arg(tag, resultCodeToString(code)));
        onPickPlaceGoalTerminal(
          loop, locked, tag,
          code == rclcpp_action::ResultCode::SUCCEEDED, resultCodeToString(code));
      };
    std::function<void()> on_accepted = [this, loop, tag]() {
      onPickPlaceGoalAccepted(loop, tag, "DrlPickPlace");
    };
    std::function<void()> on_rejected = [this, loop, tag, locked]() {
      drl_pickplace_in_flight_.store(false);
      appendActionLog(QString("[%1] pickplace_rl_goal_in_flight=false result=REJECTED").arg(tag));
      onPickPlaceGoalRejected(loop, locked, tag, "DrlPickPlace");
    };
    sendGoal<DrlPickPlace>(
      node_, "/drl_pickplace", execute ? "Pick Place RL Start" : "Pick Place RL Plan", goal, log,
      this, "PickPlaceRL", on_terminal, on_accepted, on_rejected);
  };

  if (mode == PickPlaceMode::AUTO) {
    runAutoPickCycle("PickPlaceRL", execute, loop, in, gripper, gripper_source, dispatch);
    return;
  }

  // MANUAL — resolve immediately from the latest /vision/wood_objects snapshot
  // (wood-nearest-place, else default pick).
  in.wood_candidates = collectWoodCandidates("PickPlaceRL");
  logRlTrainedXyRejects("PickPlaceRL", in.wood_candidates);
  const QString snap = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  const PickPlaceResolution r = resolvePickPlaceTargets(in);
  logResolution(
    "PickPlaceRL", PickPlaceMode::MANUAL, /*home2_used=*/false, /*from_auto=*/false,
    gripper, gripper_source, snap, r);
  if (!r.ok) {
    return;
  }
  const bool locked = execute && tryLockPickPlace("PickPlaceRL");
  dispatch(r, locked);
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
  const MovePoseRl::Goal & goal, bool execute, bool from_auto_plan)
{
  setMovePoseRlBusy(true);

  auto node = node_;
  QPointer<TaskActionController> self(this);
  std::thread([self, node, goal, execute, from_auto_plan]() {
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
    const auto goal_target = goal.target_pose.position;
    options.result_callback =
      [self, from_auto_plan, goal_target](const GoalHandle::WrappedResult & result) {
        if (!self) {
          return;
        }
        self->appendActionLog(resultString<MovePoseRl>(result));
        self->clearCancelHandle("MovePoseRL");
        self->setMovePoseRlBusy(false);
        // codex.md (AutoPlan fail cooldown): only Auto Plan cycles feed the cache.
        // A failed plan arms the cooldown for this target; a success clears it. The
        // cache is read on the GUI thread (onAutoPlanTick), so marshal the write
        // onto the GUI thread to avoid a data race.
        if (from_auto_plan) {
          const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
          QMetaObject::invokeMethod(
            self->root_,
            [self, ok, goal_target]() {
              if (!self) {
                return;
              }
              if (ok) {
                self->auto_plan_fail_valid_ = false;
              } else {
                self->recordAutoPlanFailure(goal_target.x, goal_target.y, goal_target.z);
                self->appendActionLog(QString(
                    "[MovePoseRL AutoPlan] plan failed; arming same-target cooldown "
                    "target=(%1, %2, %3) fail_cooldown_s=%4")
                  .arg(goal_target.x, 0, 'f', 3).arg(goal_target.y, 0, 'f', 3)
                  .arg(goal_target.z, 0, 'f', 3).arg(kAutoPlanFailCooldownSec, 0, 'f', 1));
              }
            },
            Qt::QueuedConnection);
        }
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
