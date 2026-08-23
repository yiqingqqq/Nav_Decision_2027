#include "rm_navigation/tunnel_controller_selector.hpp"

namespace rm_navigation
{

BT::PortsList TunnelControllerSelector::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<std::string>("default_controller", "FollowPath"),
    BT::InputPort<std::string>("tunnel_controller", "TunnelController"),
    BT::OutputPort<std::string>("selected_controller")};
}

BT::NodeStatus TunnelControllerSelector::tick()
{
  const auto path = getInput<nav_msgs::msg::Path>("path");
  const auto default_controller = getInput<std::string>("default_controller");
  const auto tunnel_controller = getInput<std::string>("tunnel_controller");
  if (!path || !default_controller || !tunnel_controller) {
    return BT::NodeStatus::FAILURE;
  }

  bool intersects_tunnel = false;
  for (const auto & stamped_pose : path->poses) {
    const double x = stamped_pose.pose.position.x;
    const double y = stamped_pose.pose.position.y;
    const bool red_tunnel = x >= 1.3 && x <= 3.7 && y >= 0.2 && y <= 1.0;
    const bool blue_tunnel = x >= -3.7 && x <= -1.3 && y >= 3.2 && y <= 4.0;
    if (red_tunnel || blue_tunnel) {
      intersects_tunnel = true;
      break;
    }
  }

  setOutput(
    "selected_controller",
    intersects_tunnel ? tunnel_controller.value() : default_controller.value());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_navigation::TunnelControllerSelector>(
    "TunnelControllerSelector");
}
