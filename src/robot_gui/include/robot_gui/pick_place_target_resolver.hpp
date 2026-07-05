#pragma once

// pick_place_target_resolver — shared, side-effect-free target resolution for
// the PickPlace / PickPlace RL pipeline (codex.md Phase 3 & 5).
//
// codex.md is explicit that the PickPlace *action* core must not be modified;
// all of the mode (MANUAL/AUTO) logic, pick/place pose resolution, multi-wood
// filtering and "nearest valid wood to place" selection live in this
// standalone layer instead. Both the regular PickPlace and the PickPlace RL
// GUI paths call the same resolvePickPlaceTargets() so their behaviour cannot
// drift apart.
//
// This header has NO ROS runtime dependency beyond the geometry_msgs message
// type (already a robot_gui dependency) and is deliberately free of any I/O,
// TF, or logging so it can be exercised by a plain self-test executable
// (see test/pick_place_resolver_selftest.cpp) without a running robot.

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace robot_gui
{

// Base yaw (rad) baked into the downward pick orientation. The historical
// default pick quaternion used across the GUI is (x,y,z,w)=(1,1,0,0) normalized,
// which is exactly RPY = (180deg, 0, 90deg): tool Z pointing straight down with a
// 90deg neutral wrist yaw. Keeping this base means makeDownwardPickOrientationWithYaw(0)
// reproduces the previous default orientation bit-for-bit (codex.md Phase 5).
inline constexpr double kPickDownwardBaseYawRad = M_PI / 2.0;

// Pure RPY (radians) -> quaternion (same ZYX convention as the GUI's
// rpyDegToQuaternion). Free of ROS/Qt so the resolver self-test can call it.
inline geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
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

// Downward pick orientation (tool Z down, roll=180deg, pitch=0) rotated about the
// base/pick Z axis by `yaw_rad` (codex.md Phase 5: gripper keeps tool Z down, yaw
// only spins about Z). At yaw_rad==0 this equals the legacy default orientation.
inline geometry_msgs::msg::Quaternion makeDownwardPickOrientationWithYaw(double yaw_rad)
{
  return rpyToQuaternion(M_PI, 0.0, kPickDownwardBaseYawRad + yaw_rad);
}

enum class PickPlaceMode
{
  MANUAL,
  AUTO,
};

inline PickPlaceMode pickPlaceModeFromString(const std::string & text)
{
  return (text == "AUTO" || text == "auto" || text == "Auto") ? PickPlaceMode::AUTO
                                                              : PickPlaceMode::MANUAL;
}

inline const char * pickPlaceModeToString(PickPlaceMode mode)
{
  return mode == PickPlaceMode::AUTO ? "AUTO" : "MANUAL";
}

// A single wood detection, already transformed into base_link (codex.md Phase 3
// step 1: transform to base_link before selection). `valid` carries the result
// of any pre-filtering the caller could perform (confidence, workspace, pose
// validity, IK, detection age); an already-invalid candidate is counted but
// never selected.
struct WoodCandidate
{
  int wood_id{-1};
  double x{0.0};  // base_link, metres
  double y{0.0};
  double z{0.0};
  float confidence{0.0f};
  bool valid{true};
  std::string invalid_reason;  // filled only when valid == false

  // codex.md Phase 5: yaw of the detection about the base/pick Z axis, when the
  // vision message carries a real (non-identity) orientation. `has_yaw` is false
  // for a plain axis-aligned bbox detection (identity orientation) -> the caller
  // reports NO_TARGET_YAW_AVAILABLE and keeps the default downward orientation.
  bool has_yaw{false};
  double yaw_rad{0.0};
};

struct PickPlaceResolveInput
{
  PickPlaceMode mode{PickPlaceMode::MANUAL};

  bool has_gui_pick{false};
  geometry_msgs::msg::Pose gui_pick_pose;
  // codex.md Phase 5: true when the user actually typed a pick yaw in the GUI, so
  // a GUI pick can be tagged pick_orientation_source=GUI_POSE vs DEFAULT_ORIENTATION.
  bool gui_pick_has_yaw{false};

  bool has_gui_place{false};
  geometry_msgs::msg::Pose gui_place_pose;

  geometry_msgs::msg::Pose default_pick_pose;  // MANUAL last-resort pick
  geometry_msgs::msg::Pose fixed_place_pose;   // default place when GUI empty

  std::vector<WoodCandidate> wood_candidates;  // already in base_link
};

struct PickPlaceResolution
{
  bool ok{false};

  geometry_msgs::msg::Pose pick_pose;
  geometry_msgs::msg::Pose place_pose;

  std::string pick_pose_source;   // "GUI" | "VISION" | "DEFAULT"
  std::string place_pose_source;  // "GUI" | "DEFAULT"

  int number_of_wood_detected{0};
  int number_of_valid_wood{0};
  int selected_wood_id{-1};
  std::string selected_reason;        // "nearest_to_place" when a wood is chosen
  double distance_to_place_xy{-1.0};  // metres; -1 when no wood selected

  // codex.md Phase 5: yaw provenance of the resolved pick orientation.
  bool target_has_yaw{false};
  double target_yaw_rad{0.0};
  // One of: VISION_QUATERNION | VISION_YAW | GUI_POSE | DEFAULT_ORIENTATION |
  // NO_TARGET_YAW_AVAILABLE.
  std::string pick_orientation_source;

  std::string fail_reason;  // "" on success; NO_WOOD_DETECTED / NO_VALID_WOOD
};

// codex.md (RL trained XY gate): the DrlPickPlace policy was trained only on
// wood spawned inside a bounded XY region on the table (robot_drl/config.py
// DEFAULT_WORKSPACE_MIN / DEFAULT_WORKSPACE_MAX -> X in [0.250, 0.500] m, Y in
// [-0.150, 0.150] m in base_link). Handing the RL pick a wood outside that XY
// box is out-of-distribution for the policy, so the PickPlaceRL path must gate
// candidates on X/Y. Z is deliberately NOT part of the gate: training randomized
// only the wood XY on the table surface, so any Z with a valid XY is still inside
// the trained regime (codex.md: "Chỉ kiểm tra X/Y, không reject theo Z").
struct RlTrainedXYBounds
{
  double x_min{0.250};
  double x_max{0.500};
  double y_min{-0.150};
  double y_max{0.150};
};

// True when (x, y) lies inside the RL-trained XY box. Pure, ROS-free, and
// independent of Z so the resolver self-test can exercise it directly.
inline bool woodInsideRlTrainedXY(double x, double y, const RlTrainedXYBounds & b)
{
  return x >= b.x_min && x <= b.x_max && y >= b.y_min && y <= b.y_max;
}

// Selects the valid wood whose XY distance to (place_x, place_y) is smallest
// (codex.md Phase 3 step 3). Returns the index into `candidates` or -1 when no
// valid candidate exists. `out_valid_count` and `out_best_dist` are always set.
inline int selectWoodNearestPlace(
  const std::vector<WoodCandidate> & candidates,
  double place_x,
  double place_y,
  int & out_valid_count,
  double & out_best_dist)
{
  out_valid_count = 0;
  out_best_dist = -1.0;
  int best_index = -1;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const WoodCandidate & c = candidates[i];
    if (!c.valid) {
      continue;
    }
    ++out_valid_count;
    const double dx = c.x - place_x;
    const double dy = c.y - place_y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (best_index < 0 || dist < out_best_dist) {
      out_best_dist = dist;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

// codex.md Phase 5: set the resolved pick orientation from a selected wood's
// yaw. A downward orientation with yaw is always rebuilt (never a raw vision
// quaternion) so the tool Z stays down; only the Z-yaw is taken from vision.
// When the wood has no yaw (identity orientation / plain bbox) the default
// downward orientation seeded into out.pick_pose is kept and the source is
// NO_TARGET_YAW_AVAILABLE.
inline void applyVisionYaw(const WoodCandidate & c, PickPlaceResolution & out)
{
  if (c.has_yaw) {
    out.pick_pose.orientation = makeDownwardPickOrientationWithYaw(c.yaw_rad);
    out.target_has_yaw = true;
    out.target_yaw_rad = c.yaw_rad;
    out.pick_orientation_source = "VISION_YAW";
  } else {
    out.target_has_yaw = false;
    out.pick_orientation_source = "NO_TARGET_YAW_AVAILABLE";
  }
}

// Pure resolver shared by PickPlace and PickPlace RL. Never throws, never
// performs I/O. On failure ok == false and fail_reason is set; the place pose
// is still resolved so callers can log it.
inline PickPlaceResolution resolvePickPlaceTargets(const PickPlaceResolveInput & in)
{
  PickPlaceResolution out;
  out.number_of_wood_detected = static_cast<int>(in.wood_candidates.size());

  // ---- place_pose: GUI custom overrides the fixed default in both modes. ----
  if (in.has_gui_place) {
    out.place_pose = in.gui_place_pose;
    out.place_pose_source = "GUI";
  } else {
    out.place_pose = in.fixed_place_pose;
    out.place_pose_source = "DEFAULT";
  }

  const double place_x = out.place_pose.position.x;
  const double place_y = out.place_pose.position.y;

  // Pre-compute wood selection once (used by MANUAL rule 2 and AUTO).
  int valid_count = 0;
  double best_dist = -1.0;
  const int sel = selectWoodNearestPlace(in.wood_candidates, place_x, place_y, valid_count, best_dist);
  out.number_of_valid_wood = valid_count;

  if (in.mode == PickPlaceMode::AUTO) {
    // AUTO pick_pose comes ONLY from a selected wood; no fallback to default.
    if (out.number_of_wood_detected == 0) {
      out.fail_reason = "NO_WOOD_DETECTED";
      return out;
    }
    if (sel < 0) {
      out.fail_reason = "NO_VALID_WOOD";
      return out;
    }
    const WoodCandidate & c = in.wood_candidates[static_cast<std::size_t>(sel)];
    out.pick_pose = in.default_pick_pose;  // seed orientation from default
    out.pick_pose.position.x = c.x;
    out.pick_pose.position.y = c.y;
    out.pick_pose.position.z = c.z;
    applyVisionYaw(c, out);  // codex.md Phase 5: orientation from wood yaw if any
    out.pick_pose_source = "VISION";
    out.selected_wood_id = c.wood_id;
    out.selected_reason = "nearest_to_place";
    out.distance_to_place_xy = best_dist;
    out.ok = true;
    return out;
  }

  // ---- MANUAL pick_pose: GUI -> VISION(valid wood) -> DEFAULT. ----
  if (in.has_gui_pick) {
    out.pick_pose = in.gui_pick_pose;
    out.pick_pose_source = "GUI";
    // Orientation comes from the GUI pose; tag it GUI_POSE only when the user
    // actually typed a yaw, otherwise the field was the default downward one.
    out.pick_orientation_source = in.gui_pick_has_yaw ? "GUI_POSE" : "DEFAULT_ORIENTATION";
    out.target_has_yaw = in.gui_pick_has_yaw;
  } else if (sel >= 0) {
    const WoodCandidate & c = in.wood_candidates[static_cast<std::size_t>(sel)];
    out.pick_pose = in.default_pick_pose;
    out.pick_pose.position.x = c.x;
    out.pick_pose.position.y = c.y;
    out.pick_pose.position.z = c.z;
    applyVisionYaw(c, out);
    out.pick_pose_source = "VISION";
    out.selected_wood_id = c.wood_id;
    out.selected_reason = "nearest_to_place";
    out.distance_to_place_xy = best_dist;
  } else {
    out.pick_pose = in.default_pick_pose;
    out.pick_pose_source = "DEFAULT";
    out.pick_orientation_source = "DEFAULT_ORIENTATION";
  }
  out.ok = true;
  return out;
}

}  // namespace robot_gui
