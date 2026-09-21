#include "action/follow_tunnel_path.hpp"

namespace rm_behavior_tree
{

FollowTunnelPathAction::FollowTunnelPathAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::FollowPath>(name, conf, params)
{
}

BT::PortsList FollowTunnelPathAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<std::string>("controller_id", "TunnelController"),
    BT::InputPort<std::string>("goal_checker_id", "general_goal_checker")});
}

bool FollowTunnelPathAction::setGoal(nav2_msgs::action::FollowPath::Goal & goal)
{
  const auto path = getInput<nav_msgs::msg::Path>("path");
  const auto controller_id = getInput<std::string>("controller_id");
  const auto goal_checker_id = getInput<std::string>("goal_checker_id");
  if (!path || path.value().poses.size() < 2 || !controller_id || !goal_checker_id) {
    RCLCPP_ERROR(logger(), "Invalid committed tunnel path or controller settings");
    return false;
  }

  goal.path = path.value();
  goal.path.header.frame_id = "map";
  goal.path.header.stamp = now();
  for (auto & pose : goal.path.poses) {
    pose.header.frame_id = "map";
    pose.header.stamp = goal.path.header.stamp;
  }
  goal.controller_id = controller_id.value();
  goal.goal_checker_id = goal_checker_id.value();
  RCLCPP_INFO(
    logger(), "[%s] submit FollowPath: controller=%s, checker=%s, poses=%zu",
    name().c_str(), goal.controller_id.c_str(), goal.goal_checker_id.c_str(),
    goal.path.poses.size());
  return true;
}

BT::NodeStatus FollowTunnelPathAction::onResultReceived(const WrappedResult & result)
{
  RCLCPP_INFO(
    logger(), "[%s] FollowPath result: %s", name().c_str(),
    result.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded" : "failed");
  return result.code == rclcpp_action::ResultCode::SUCCEEDED ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowTunnelPathAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_WARN(logger(), "Committed tunnel path failed: %s", BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::FollowTunnelPathAction, "FollowTunnelPath");
