// Standalone self-test for pick_place_target_resolver.hpp (codex.md Phase 6).
//
// Exercises every MANUAL and AUTO test case from codex.md section 6 as pure
// logic — no ROS runtime, no robot, no camera. Build target
// `pick_place_resolver_selftest`; a non-zero exit code means a case regressed.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot_gui/pick_place_target_resolver.hpp"

using robot_gui::PickPlaceMode;
using robot_gui::PickPlaceResolveInput;
using robot_gui::PickPlaceResolution;
using robot_gui::RlTrainedXYBounds;
using robot_gui::WoodCandidate;
using robot_gui::resolvePickPlaceTargets;
using robot_gui::woodInsideRlTrainedXY;

namespace
{
int g_failures = 0;

void check(bool cond, const char * name)
{
  std::printf("[%s] %s\n", cond ? "PASS" : "FAIL", name);
  if (!cond) {
    ++g_failures;
  }
}

geometry_msgs::msg::Pose pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;
  return p;
}

WoodCandidate wood(int id, double x, double y, double z, float conf, bool valid = true)
{
  WoodCandidate c;
  c.wood_id = id;
  c.x = x;
  c.y = y;
  c.z = z;
  c.confidence = conf;
  c.valid = valid;
  return c;
}
}  // namespace

int main()
{
  const auto default_pick = pose(0.40, 0.10, 0.25);
  const auto fixed_place = pose(0.30, 0.0, 0.25);

  // ---- MANUAL 1: GUI pick + GUI place -> both GUI. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::MANUAL;
    in.has_gui_pick = true;
    in.gui_pick_pose = pose(0.42, 0.05, 0.20);
    in.has_gui_place = true;
    in.gui_place_pose = pose(0.31, -0.02, 0.24);
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.pick_pose_source == "GUI" && r.place_pose_source == "GUI",
      "MANUAL 1: GUI pick + GUI place");
    check(r.pick_pose.position.x == 0.42 && r.place_pose.position.x == 0.31,
      "MANUAL 1: values from GUI");
  }

  // ---- MANUAL 2: no pick, no place, no vision -> DEFAULT pick + DEFAULT place.
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::MANUAL;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.pick_pose_source == "DEFAULT" && r.place_pose_source == "DEFAULT",
      "MANUAL 2: defaults when nothing supplied");
    check(r.pick_pose.position.x == 0.40 && r.place_pose.position.x == 0.30,
      "MANUAL 2: default values");
  }

  // ---- MANUAL 3: no pick + one fake wood -> VISION pick. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::MANUAL;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    in.wood_candidates = {wood(7, 0.38, 0.02, 0.05, 0.9f)};
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.pick_pose_source == "VISION" && r.selected_wood_id == 7,
      "MANUAL 3: wood becomes pick");
    check(r.pick_pose.position.x == 0.38 && r.selected_reason == "nearest_to_place",
      "MANUAL 3: pick from wood pose");
  }

  // ---- MANUAL 4: no place -> fixed_place_pose (with GUI pick present). ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::MANUAL;
    in.has_gui_pick = true;
    in.gui_pick_pose = pose(0.41, 0.03, 0.2);
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.place_pose_source == "DEFAULT" && r.place_pose.position.x == 0.30,
      "MANUAL 4: place falls back to fixed default");
  }

  // ---- AUTO 2: no wood -> fail NO_WOOD_DETECTED, no crash. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(!r.ok && r.fail_reason == "NO_WOOD_DETECTED",
      "AUTO 2: no wood -> NO_WOOD_DETECTED");
    check(r.place_pose_source == "DEFAULT", "AUTO 2: place still resolved for logging");
  }

  // ---- AUTO 3: one fake wood -> selected as pick. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    in.wood_candidates = {wood(3, 0.45, -0.05, 0.06, 0.8f)};
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.pick_pose_source == "VISION" && r.selected_wood_id == 3,
      "AUTO 3: single wood selected");
  }

  // ---- AUTO 4: many woods -> valid one nearest (XY) to place is chosen. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;  // place at (0.30, 0.0)
    in.wood_candidates = {
      wood(1, 0.50, 0.10, 0.05, 0.9f),   // dist ~0.224
      wood(2, 0.33, 0.02, 0.05, 0.6f),   // dist ~0.036  <-- nearest
      wood(3, 0.40, -0.08, 0.05, 0.95f), // dist ~0.128
    };
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.selected_wood_id == 2 && r.number_of_valid_wood == 3,
      "AUTO 4: nearest-to-place wood chosen (not highest confidence)");
    check(r.distance_to_place_xy < 0.05, "AUTO 4: reported distance is the smallest");
  }

  // ---- AUTO 5: many woods but all invalid -> NO_VALID_WOOD. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    in.wood_candidates = {
      wood(1, 0.33, 0.02, 0.05, 0.2f, false),
      wood(2, 0.40, -0.08, 0.05, 0.1f, false),
    };
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(!r.ok && r.fail_reason == "NO_VALID_WOOD" && r.number_of_wood_detected == 2,
      "AUTO 5: all invalid -> NO_VALID_WOOD");
  }

  // ---- AUTO: mixed valid/invalid -> nearest *valid* wins even if an invalid
  //      one is closer. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    in.wood_candidates = {
      wood(1, 0.305, 0.0, 0.05, 0.9f, false),  // closest but invalid
      wood(2, 0.34, 0.0, 0.05, 0.7f, true),    // nearest valid
    };
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.selected_wood_id == 2,
      "AUTO mixed: nearest VALID wood chosen, invalid closer one skipped");
  }

  // ---- YAW 0: fake wood with yaw=0 -> pick orientation == legacy default
  //      downward (x,y,z,w)=(1,1,0,0) normalized, source VISION_YAW. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    WoodCandidate w = wood(9, 0.40, 0.0, 0.05, 0.9f);
    w.has_yaw = true;
    w.yaw_rad = 0.0;
    in.wood_candidates = {w};
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    const double s = std::sqrt(0.5);
    const bool downward =
      std::abs(r.pick_pose.orientation.x - s) < 1e-6 &&
      std::abs(r.pick_pose.orientation.y - s) < 1e-6 &&
      std::abs(r.pick_pose.orientation.z - 0.0) < 1e-6 &&
      std::abs(r.pick_pose.orientation.w - 0.0) < 1e-6;
    check(r.ok && r.target_has_yaw && r.pick_orientation_source == "VISION_YAW",
      "YAW 0: source VISION_YAW, target_has_yaw");
    check(downward && std::abs(r.target_yaw_rad - 0.0) < 1e-9,
      "YAW 0: orientation == legacy default downward (1,1,0,0)");
  }

  // ---- YAW 1.57: fake wood with yaw=pi/2 -> orientation must differ from the
  //      yaw=0 default, tool still Z-down. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    WoodCandidate w = wood(10, 0.40, 0.0, 0.05, 0.9f);
    w.has_yaw = true;
    w.yaw_rad = 1.57;
    in.wood_candidates = {w};
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    const double s = std::sqrt(0.5);
    const bool changed_from_default =
      std::abs(r.pick_pose.orientation.x - s) > 1e-3 ||
      std::abs(r.pick_pose.orientation.y - s) > 1e-3;
    check(r.ok && r.pick_orientation_source == "VISION_YAW" &&
      std::abs(r.target_yaw_rad - 1.57) < 1e-9,
      "YAW 1.57: source VISION_YAW, yaw carried through");
    check(changed_from_default, "YAW 1.57: orientation differs from yaw=0 default");
  }

  // ---- YAW none: wood without yaw -> NO_TARGET_YAW_AVAILABLE, default kept. ----
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    in.wood_candidates = {wood(11, 0.40, 0.0, 0.05, 0.9f)};  // has_yaw=false
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && !r.target_has_yaw &&
      r.pick_orientation_source == "NO_TARGET_YAW_AVAILABLE",
      "YAW none: NO_TARGET_YAW_AVAILABLE when wood has no yaw");
  }

  // ---- RL trained XY gate (codex.md fake candidate resolver): the DrlPickPlace
  //      path only picks wood inside the trained XY box; Z is never checked. ----
  const RlTrainedXYBounds rl_xy;  // defaults = robot_drl/config.py workspace XY

  // RL 1: wood inside the trained XY box -> valid (inside).
  {
    check(woodInsideRlTrainedXY(0.35, 0.00, rl_xy),
      "RL 1: wood inside trained XY -> valid");
    check(woodInsideRlTrainedXY(rl_xy.x_min, rl_xy.y_min, rl_xy) &&
      woodInsideRlTrainedXY(rl_xy.x_max, rl_xy.y_max, rl_xy),
      "RL 1b: trained XY box is inclusive at the boundary");
  }

  // RL 2: wood outside X min/max -> invalid.
  {
    check(!woodInsideRlTrainedXY(rl_xy.x_min - 0.001, 0.00, rl_xy),
      "RL 2a: wood below X min -> invalid");
    check(!woodInsideRlTrainedXY(rl_xy.x_max + 0.001, 0.00, rl_xy),
      "RL 2b: wood above X max -> invalid");
  }

  // RL 3: wood outside Y min/max -> invalid.
  {
    check(!woodInsideRlTrainedXY(0.35, rl_xy.y_min - 0.001, rl_xy),
      "RL 3a: wood below Y min -> invalid");
    check(!woodInsideRlTrainedXY(0.35, rl_xy.y_max + 0.001, rl_xy),
      "RL 3b: wood above Y max -> invalid");
  }

  // RL 4: wood with any Z but valid X/Y -> still valid (Z never rejected).
  {
    check(woodInsideRlTrainedXY(0.35, 0.05, rl_xy),  // XY only, Z absent from API
      "RL 4a: valid X/Y stays valid regardless of Z (Z not part of the gate)");
    // Drive it through the resolver the way collectWoodCandidates does: mark
    // validity from the XY gate only, then a wood with an extreme Z but valid XY
    // must still be selected for the RL pick.
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    WoodCandidate high_z = wood(21, 0.34, 0.01, 999.0, 0.9f);  // absurd Z, valid XY
    high_z.valid = woodInsideRlTrainedXY(high_z.x, high_z.y, rl_xy);
    WoodCandidate out_xy = wood(22, 0.90, 0.40, 0.05, 0.95f);  // best conf, out of XY
    out_xy.valid = woodInsideRlTrainedXY(out_xy.x, out_xy.y, rl_xy);
    if (!out_xy.valid) {
      out_xy.invalid_reason = "OUT_OF_RL_TRAINED_XY";
    }
    in.wood_candidates = {high_z, out_xy};
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(r.ok && r.selected_wood_id == 21 && r.number_of_valid_wood == 1,
      "RL 4b: high-Z valid-XY wood selected; out-of-XY wood rejected");
  }

  // RL 5: only wood is out of the trained XY box -> NO_VALID_WOOD (never
  //       dispatched to the RL pick), even though a wood was detected.
  {
    PickPlaceResolveInput in;
    in.mode = PickPlaceMode::AUTO;
    in.default_pick_pose = default_pick;
    in.fixed_place_pose = fixed_place;
    WoodCandidate out_xy = wood(23, 0.20, 0.00, 0.05, 0.9f);  // X below trained min
    out_xy.valid = woodInsideRlTrainedXY(out_xy.x, out_xy.y, rl_xy);
    if (!out_xy.valid) {
      out_xy.invalid_reason = "OUT_OF_RL_TRAINED_XY";
    }
    in.wood_candidates = {out_xy};
    const PickPlaceResolution r = resolvePickPlaceTargets(in);
    check(!r.ok && r.fail_reason == "NO_VALID_WOOD" && r.number_of_wood_detected == 1,
      "RL 5: out-of-XY wood detected but not dispatched (NO_VALID_WOOD)");
  }

  if (g_failures == 0) {
    std::printf("\nALL PICK/PLACE RESOLVER SELF-TESTS PASSED\n");
    return 0;
  }
  std::printf("\n%d SELF-TEST(S) FAILED\n", g_failures);
  return 1;
}
