#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_behavior_tree
{
// Capture once per navigation goal. Zero radians is a valid explicit heading.
inline void applyHeadingPolicy(
  geometry_msgs::msg::PoseStamped & goal, const std::string & policy,
  const std::optional<geometry_msgs::msg::PoseStamped> & current = std::nullopt)
{
  if (policy != "hold" && policy != "explicit") {
    throw std::invalid_argument("heading_policy must be hold or explicit");
  }
  if (policy == "hold" && (!current || current->header.frame_id != goal.header.frame_id)) {
    throw std::invalid_argument("hold heading requires current_pose in the goal frame");
  }
  auto q = policy == "hold" ? current->pose.orientation : goal.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("heading quaternion must be finite and nonzero");
  }
  q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  auto & result = goal.pose.orientation;
  result.x = result.y = 0.0;
  result.z = std::sin(yaw * 0.5);
  result.w = std::cos(yaw * 0.5);
}
}  // namespace rm_behavior_tree
