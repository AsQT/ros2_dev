#pragma once

#include <atomic>
#include <chrono>
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

// codex.md Phase 3/4 auto-loop state machine (shared by PickPlaceVision and
// PickPlaceRL). One tick == 1 second; the loop only starts a new pick cycle when
// nothing is running and the robot is not moving to home_2.
enum class AutoLoopState
{
  IDLE,               // free; next tick may start a cycle
  MOVING_HOME_2,      // /gohome_2 goal in flight, skip ticks
  WAITING_DETECTION,  // home_2 reached, taking a vision snapshot
  DISPATCHING_TASK,   // goal being sent
  TASK_RUNNING,       // pick/place goal in flight, skip ticks
  STOPPED,            // Auto loop off
  AUTO_ERROR,         // unrecoverable input error; loop disabled
};

const char * autoLoopStateToString(AutoLoopState state);

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

  // codex.md Phase 3/4: per-tab auto-loop runtime state. Held as two named
  // members (vision_loop_/rl_loop_) — never copied — so the atomics are safe to
  // read from the 1 s timer tick (GUI thread) and write from ROS callbacks.
  struct AutoLoopContext
  {
    QString source_tab;    // "PickPlaceVision" / "PickPlaceRL" (also action_type)
    QString mode_combo;    // objectName of the MANUAL/AUTO combo
    QString auto_button;   // objectName of the Start/Stop Auto toggle
    QTimer * timer{nullptr};
    std::atomic<AutoLoopState> state{AutoLoopState::STOPPED};
    std::atomic<bool> enabled{false};
    // False right after a no-wood tick (robot still parked at home_2) so the next
    // tick skips /gohome_2 instead of spamming it; true ONLY after a goal is
    // accepted (the arm may leave home_2). A rejected goal keeps this false so the
    // loop does not re-home every second (codex.md goal-rejected fix Phase 4).
    std::atomic<bool> need_home_2{true};
    // Consecutive rejected goals; after kMaxGoalRejectRetries the loop goes
    // AUTO_ERROR instead of spamming goals forever.
    std::atomic<int> consecutive_goal_reject_count{0};
    // codex.md (post-place direct detect): set true after a SUCCEEDED place so the
    // next tick tries a vision snapshot at the CURRENT pose first (skip /gohome_2);
    // if a valid wood is found it picks again directly, otherwise it re-homes.
    std::atomic<bool> try_detect_from_current_pose{false};
    // True only for a goal dispatched from a post-place direct-detect snapshot.
    // If that goal is rejected / server-unavailable before acceptance, the robot
    // is still at the previous place pose, not home_2, so the next detect must
    // re-home first. Goals dispatched after an actual home_2 snapshot keep the
    // old no-rehome retry behavior.
    std::atomic<bool> last_dispatch_from_current_pose{false};
  };

  void sendMovePose(bool execute);
  void sendMoveToPoseObstacle(bool execute);
  void sendGoHome();
  void sendGripper(double position, bool execute, const QString & label);
  void sendPickPlace(bool execute);
  void sendPickPlaceVision(bool execute, AutoLoopContext * loop = nullptr);
  void sendDrlPickPlace(bool execute, AutoLoopContext * loop = nullptr);

  // codex.md Phase 2/3: MANUAL/AUTO mode support shared by PickPlace and
  // PickPlace RL. The combo boxes are created in code (like the Plan buttons)
  // so the large robot_gui.ui does not have to change.
  void addModeComboIfMissing(const QString & tab_name, const QString & combo_name);
  PickPlaceMode currentPickPlaceMode(const QString & combo_name) const;

  // codex.md Phase 2/8: adds the MANUAL/AUTO combo, the Start/Stop Auto toggle
  // and the gripper close-width spinbox to a Pick/Place tab, all in code so the
  // .ui stays untouched. Idempotent (skips widgets that already exist).
  void setupPickPlaceAutoControls(
    const QString & tab_name, const QString & mode_combo, const QString & auto_button,
    const QString & gripper_spin, double default_gripper_mm, AutoLoopContext & loop);
  // Reads the gripper close width (mm -> m) from a tab's spinbox; falls back to
  // default_mm when the spinbox is absent. `source` set to GUI or DEFAULT.
  double pickPlaceGripperMeters(
    const QString & spin_name, double default_mm, QString & source) const;

  // codex.md Phase 3/4 shared auto-loop driver.
  void setAutoLoopEnabled(AutoLoopContext & loop, bool on, const QString & reason);
  void onAutoLoopTick(AutoLoopContext & loop);
  // Logs `auto_loop_state_change old=.. new=.. reason=..` and stores the state.
  void setAutoLoopState(AutoLoopContext & loop, AutoLoopState next, const QString & reason);
  // Result-aware terminal handler (SUCCEEDED / ABORTED / CANCELED). Unlocks the
  // target (only when `locked`) and sets the post-place policy (codex.md):
  //   SUCCEEDED -> need_home_2=false, try_detect_from_current_pose=true
  //   otherwise -> need_home_2=true (must re-home before the next detection).
  // `succeeded`/`code_str` come from the ResultCode so the header stays free of
  // the rclcpp_action dependency.
  void onPickPlaceGoalTerminal(
    AutoLoopContext * loop, bool locked, const QString & tag,
    bool succeeded, const QString & code_str);
  // codex.md (goal-rejected fix + post-place policy): handle a rejected
  // /pickplace or /drl_pickplace goal. Releases the lock, keeps need_home_2=false
  // only when the rejected goal was dispatched from home_2, and forces re-home
  // when it was dispatched from a post-place direct-detect pose. Counts
  // consecutive rejects and, past kMaxGoalRejectRetries, drops the loop to
  // AUTO_ERROR and stops it instead of spamming goals.
  void onPickPlaceGoalRejected(
    AutoLoopContext * loop, bool locked, const QString & tag, const QString & action);
  // codex.md (goal-rejected fix): a goal was accepted — the arm may leave the
  // current pose, so re-home before the next detection unless the terminal result
  // is SUCCEEDED and arms post-place direct detect. Resets the reject counter.
  void onPickPlaceGoalAccepted(AutoLoopContext * loop, const QString & tag, const QString & action);

  // codex.md Phase 3/4: shared AUTO cycle used by both PickPlaceVision and
  // PickPlaceRL so their state machine cannot drift. Locks the target, moves to
  // home_2 (unless the loop already parked there with no object), snapshots the
  // vision candidates once, resolves + logs, and dispatches via `dispatch`.
  // `loop` == nullptr means a single manual AUTO Start press (no looping).
  void runAutoPickCycle(
    const QString & source_tab, bool execute, AutoLoopContext * loop,
    const PickPlaceResolveInput & in, double gripper_m, const QString & gripper_source,
    std::function<void(const PickPlaceResolution & r, bool locked)> dispatch);
  // Collects fresh wood candidates (base_link) and marks the ones outside the
  // trained workspace / with non-finite pose as invalid, so the resolver counts
  // but never selects them.
  std::vector<WoodCandidate> collectWoodCandidates(const QString & action_type) const;
  // codex.md (RL trained XY gate): logs one OUT_OF_RL_TRAINED_XY line per wood
  // that the PickPlaceRL X/Y gate rejected, so the reason a wood is not dispatched
  // to the RL pick is visible in the Action Log (Z is never a rejection reason).
  void logRlTrainedXyRejects(
    const QString & source_tab, const std::vector<WoodCandidate> & candidates);
  // Logs every codex.md Phase 5/7/8/9 field for one resolved goal to the Action
  // Log (source_tab, mode, auto-loop, yaw, gripper, snapshot, pre_pick...).
  void logResolution(
    const QString & source_tab, PickPlaceMode mode, bool home2_used, bool from_auto,
    double gripper_m, const QString & gripper_source, const QString & snapshot_time,
    const PickPlaceResolution & r);
  // AUTO helper: sends /gohome_2 (execute) and, only on success, runs on_home_ok
  // on the ROS executor thread. On failure logs HOME_2_*, releases the lock and
  // runs on_home_fail (used by the auto-loop to reset its state to IDLE).
  void goHome2ThenRun(
    const QString & action_type, bool execute, std::function<void()> on_home_ok,
    std::function<void()> on_home_fail = nullptr);
  // Target lock (codex.md Phase 3): true while a PickPlace/PickPlace RL run is
  // in flight so a second Start press cannot start an overlapping run / retarget.
  bool tryLockPickPlace(const QString & action_type);
  void unlockPickPlace();
  void sendMovePoseRl(bool execute);
  void dispatchMovePoseRlGoal(
    const robot_task_manager::action::MovePoseRl::Goal & goal, bool execute,
    bool from_auto_plan = false);
  // codex.md (AutoPlan fail cooldown): true when `target` is within
  // kAutoPlanSameTargetThresholdM of the last failed Auto Plan target and still
  // inside kAutoPlanFailCooldownSec — the caller then skips the cycle.
  bool autoPlanTargetInFailCooldown(double x, double y, double z) const;
  void recordAutoPlanFailure(double x, double y, double z);
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
  // codex.md (AutoPlan fail cooldown): remembers the last Auto Plan target that
  // failed so the same (near-identical) target is not re-sent every cycle. Reset
  // when Auto Plan is toggled OFF/ON and cleared on a successful plan.
  bool auto_plan_fail_valid_{false};
  double auto_plan_fail_x_{0.0};
  double auto_plan_fail_y_{0.0};
  double auto_plan_fail_z_{0.0};
  std::chrono::steady_clock::time_point auto_plan_fail_time_{};
  // codex.md Phase 3 target lock (see tryLockPickPlace/unlockPickPlace).
  std::atomic<bool> pickplace_running_{false};
  // codex.md (Plan/Start in-flight fix): true from the moment ANY /drl_pickplace
  // goal is sent (Plan execute=false OR Start execute=true, manual OR auto-loop)
  // until that goal reaches a terminal/rejected state. Unlike pickplace_running_
  // (execute-only target lock), this guard blocks a second /drl_pickplace goal —
  // Plan and Start block each other — so the server never sees two parallel goals
  // and rejects one. Cleared on SUCCEEDED/ABORTED/CANCELED/REJECTED and on server
  // unavailable. Dedicated to the RL tab so PickPlaceVision is untouched.
  std::atomic<bool> drl_pickplace_in_flight_{false};

  // codex.md Phase 3/4 auto-loop contexts (one per tab, shared driver).
  AutoLoopContext vision_loop_;
  AutoLoopContext rl_loop_;
};

}  // namespace robot_gui
