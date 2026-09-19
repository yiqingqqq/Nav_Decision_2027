#include "action/select_tunnel_direction.hpp"

#include <cmath>
#include <iostream>

namespace rm_behavior_tree
{

BT::PortsList SelectTunnelDirectionAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
    BT::InputPort<uint8_t>("robot_id"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("preparation_goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("entry_goal"),
    BT::OutputPort<nav_msgs::msg::Path>("entry_path"),
    BT::OutputPort<nav_msgs::msg::Path>("traverse_path"),
    BT::OutputPort<nav_msgs::msg::Path>("retreat_path")};
}

geometry_msgs::msg::PoseStamped SelectTunnelDirectionAction::makePose(
  double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

geometry_msgs::msg::PoseStamped SelectTunnelDirectionAction::worldToMap(
  double x, double y, double yaw)
{
  // Gazebo spawns the RMUL2027 red sentry at world (5, -3, pi), while
  // slam_toolbox defines map (0, 0, 0) at that pose. Keep the competition
  // route in world coordinates and perform the rigid transform at this
  // module boundary instead of mixing world and map coordinates in the BT.
  constexpr double kSpawnX = 5.0;
  constexpr double kSpawnY = -3.0;
  constexpr double kPi = 3.14159265358979323846;
  return makePose(kSpawnX - x, kSpawnY - y, yaw - kPi);
}

BT::NodeStatus SelectTunnelDirectionAction::tick()
{
  const auto location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
  const auto robot_id = getInput<uint8_t>("robot_id");
  if (!location || !robot_id || robot_id.value() > 1) {
    return BT::NodeStatus::FAILURE;
  }

  constexpr double kPi = 3.14159265358979323846;
  // The tunnel mode currently supports the red-side route only (robot_id=0).
  if (robot_id.value() != 0) {
    return BT::NodeStatus::FAILURE;
  }

  // Keep preparation poses outside the tunnel mouth. The ordered entry path
  // is handled by a dedicated low-clearance controller only after alignment.
  // The ordinary Nav2 controller handles both the open-space preparation pose
  // and the entrance pose, matching the manually validated RViz goal chain.
  const auto preparation_a = worldToMap(4.40, -3.20, kPi);
  // The measured handoff path ends at map (1.0, 0.4). The actual tunnel
  // centerline is 0.2 m farther inward at map y=0.6.
  const auto endpoint_a = worldToMap(4.0, -3.6, kPi);
  const auto midpoint = worldToMap(2.5, -3.6, kPi);
  const auto endpoint_b = worldToMap(0.5, -3.6, kPi);
  // Mirror the approach geometry for reverse traversal from the B side.
  const auto preparation_b = worldToMap(0.10, -3.20, 0.0);

  const double x = location->transform.translation.x;
  const double y = location->transform.translation.y;
  const auto squared_distance = [x, y](const geometry_msgs::msg::PoseStamped & pose) {
      const double dx = pose.pose.position.x - x;
      const double dy = pose.pose.position.y - y;
      return dx * dx + dy * dy;
    };

  const bool a_is_nearer = squared_distance(endpoint_a) <= squared_distance(endpoint_b);
  const auto & preparation = a_is_nearer ? preparation_a : preparation_b;
  const auto & entry = a_is_nearer ? endpoint_a : endpoint_b;
  const auto & exit = a_is_nearer ? endpoint_b : endpoint_a;
  const double tunnel_yaw = a_is_nearer ? 0.0 : kPi;

  const auto oriented_pose = [tunnel_yaw](const auto & pose) {
      return makePose(pose.pose.position.x, pose.pose.position.y, tunnel_yaw);
    };
  const auto oriented_entry = oriented_pose(entry);
  const auto oriented_preparation = oriented_pose(preparation);
  const auto oriented_midpoint = oriented_pose(midpoint);
  const auto oriented_exit = oriented_pose(exit);

  nav_msgs::msg::Path entry_path;
  entry_path.header.frame_id = "map";
  if (a_is_nearer) {
    // Preparation is a controller handoff region. Follow the measured,
    // ordered approach instead of asking the global planner to rediscover it.
    entry_path.poses = {
      makePose(0.60, 0.20, 0.0),
      makePose(0.72, 0.24, 0.0),
      makePose(0.84, 0.31, 0.0),
      makePose(1.00, 0.40, 0.0)};
  } else {
    // Mirror the approach for a traversal starting at the far endpoint.
    entry_path.poses = {
      makePose(4.90, 0.20, kPi),
      makePose(4.78, 0.24, kPi),
      makePose(4.66, 0.31, kPi),
      makePose(4.50, 0.40, kPi)};
  }

  nav_msgs::msg::Path traverse_path;
  traverse_path.header.frame_id = "map";
  traverse_path.poses = {oriented_entry, oriented_midpoint, oriented_exit};

  // Preserve the committed tunnel heading while backing out. TunnelController
  // is configured without oscillation recovery, so this cannot invoke Spin.
  nav_msgs::msg::Path retreat_path;
  retreat_path.header.frame_id = "map";
  retreat_path.poses = {oriented_exit, oriented_midpoint, oriented_entry, oriented_preparation};

  setOutput("preparation_goal", oriented_preparation);
  setOutput("entry_goal", oriented_entry);
  setOutput("entry_path", entry_path);
  setOutput("traverse_path", traverse_path);
  setOutput("retreat_path", retreat_path);
  std::cout << "Tunnel route selected from map pose (" << x << ", " << y
            << "): preparation=(" << oriented_preparation.pose.position.x << ", "
            << oriented_preparation.pose.position.y << "), entry=("
            << oriented_entry.pose.position.x << ", " << oriented_entry.pose.position.y
            << "), exit=(" << oriented_exit.pose.position.x << ", "
            << oriented_exit.pose.position.y << ")" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectTunnelDirectionAction>(
    "SelectTunnelDirection");
}
