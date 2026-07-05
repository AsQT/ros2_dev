#pragma once

// Shared "resolve RL obstacle input" helper for the DRL action servers.
//
// MovePoseRL and PickPlaceRL both feed the DRL unified planner a 15D observation
// whose obstacle fields (indices 9..14: rel_obs_xyz + obs_size_xyz) come from a
// best-effort vision box. Historically MovePoseRL resolved a real box while
// PickPlaceRL cleared the obstacle to zero, so the two actions handed the SAME
// policy different observations for the same target (see
// Reports/pickplace_rl_compare_moveposerl_plan_to_prepick_report.md). This helper
// is the single source of truth for that resolution so both actions stay
// consistent.
//
// Selection rule (unchanged from MovePoseRL / MoveTargetRl): among boxes of the
// configured obstacle_class with a finite pose and strictly-positive full size,
// pick the one whose centre is closest to the current-TCP -> target segment.
// Transform to the planning frame is injected by the caller (each server owns its
// own TF buffer), so this header stays TF-agnostic and unit-testable.

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_vision_pipeline_msgs/msg/box_array.hpp"

namespace robot_task_manager
{

struct RlObstacleInput
{
  bool has_obstacle = false;
  std::string source = "none";  // "vision" or "none"
  geometry_msgs::msg::Point center_base;
  geometry_msgs::msg::Vector3 size;
};

// Fills `out` (planning frame) from `in` expressed in `frame_in`. Returns false
// if the transform is unavailable (that box is then skipped).
using PointTransformFn = std::function<bool(
  const geometry_msgs::msg::Point & in,
  const std::string & frame_in,
  geometry_msgs::msg::Point & out)>;

namespace rl_obstacle_detail
{

inline bool finitePoint(const geometry_msgs::msg::Point & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

// "Full size" obstacle: every axis must be a real, strictly positive extent.
inline bool positiveSize(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
    v.x > 1e-6 && v.y > 1e-6 && v.z > 1e-6;
}

inline double pointToSegmentDistance(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Point & seg_a,
  const geometry_msgs::msg::Point & seg_b)
{
  const double abx = seg_b.x - seg_a.x;
  const double aby = seg_b.y - seg_a.y;
  const double abz = seg_b.z - seg_a.z;
  const double apx = point.x - seg_a.x;
  const double apy = point.y - seg_a.y;
  const double apz = point.z - seg_a.z;
  const double ab_len2 = abx * abx + aby * aby + abz * abz;
  double t = 0.0;
  if (ab_len2 > 1e-12) {
    t = (apx * abx + apy * aby + apz * abz) / ab_len2;
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  }
  const double cx = seg_a.x + t * abx;
  const double cy = seg_a.y + t * aby;
  const double cz = seg_a.z + t * abz;
  const double dx = point.x - cx;
  const double dy = point.y - cy;
  const double dz = point.z - cz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace rl_obstacle_detail

// Resolves the single obstacle box to hand the DRL policy. `boxes` should be a
// snapshot the caller has already confirmed is fresh enough (empty boxes -> no
// obstacle). `to_planning_frame` transforms a box centre from
// `boxes.header.frame_id` into the planning frame.
inline RlObstacleInput resolveRlObstacleInput(
  const robot_vision_pipeline_msgs::msg::BoxArray & boxes,
  const std::string & obstacle_class,
  const geometry_msgs::msg::Point & current_tcp_base,
  const geometry_msgs::msg::Point & target_base,
  const PointTransformFn & to_planning_frame)
{
  RlObstacleInput result;

  struct Candidate
  {
    geometry_msgs::msg::Point center_base;
    geometry_msgs::msg::Vector3 size;
  };
  std::vector<Candidate> candidates;
  for (const auto & box : boxes.boxes) {
    if (box.class_name != obstacle_class) {
      continue;
    }
    if (!rl_obstacle_detail::finitePoint(box.pose.position) ||
      !rl_obstacle_detail::positiveSize(box.size))
    {
      continue;
    }
    geometry_msgs::msg::Point center_base;
    if (!to_planning_frame(box.pose.position, boxes.header.frame_id, center_base)) {
      continue;
    }
    candidates.push_back({center_base, box.size});
  }

  if (candidates.empty()) {
    result.has_obstacle = false;
    result.source = "none";
    return result;
  }

  const Candidate * chosen = &candidates.front();
  if (candidates.size() > 1) {
    double best_dist = rl_obstacle_detail::pointToSegmentDistance(
      candidates.front().center_base, current_tcp_base, target_base);
    for (size_t i = 1; i < candidates.size(); ++i) {
      const double d = rl_obstacle_detail::pointToSegmentDistance(
        candidates[i].center_base, current_tcp_base, target_base);
      if (d < best_dist) {
        best_dist = d;
        chosen = &candidates[i];
      }
    }
  }

  result.has_obstacle = true;
  result.source = "vision";
  result.center_base = chosen->center_base;
  result.size = chosen->size;
  return result;
}

}  // namespace robot_task_manager
