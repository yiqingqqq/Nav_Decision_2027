#include "action/next_patrol_waypoint.hpp"

#include <stdexcept>
#include <utility>

namespace rm_behavior_tree
{

NextPatrolWaypointAction::NextPatrolWaypointAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(std::move(node))
{
  if (!node_) {
    throw std::invalid_argument("NextPatrolWaypoint requires a ROS node");
  }
}

BT::NodeStatus NextPatrolWaypointAction::tick()
{
  const auto preset_input = getInput<std::string>("preset");
  if (!preset_input || preset_input->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "NextPatrolWaypoint: missing preset");
    return BT::NodeStatus::FAILURE;
  }

  const auto & preset = preset_input.value();
  if (preset != last_preset_) {
    last_preset_ = preset;
    next_index_ = 0;
  }

  const std::string parameter =
    "strategy.presets." + preset + ".patrol_route";
  if (!node_->has_parameter(parameter)) {
    RCLCPP_ERROR(
      node_->get_logger(), "NextPatrolWaypoint: missing route '%s'",
      parameter.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto route = node_->get_parameter(parameter).as_string_array();
  if (route.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "NextPatrolWaypoint: route '%s' is empty",
      parameter.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const std::size_t selected = next_index_ % route.size();
  setOutput("waypoint", route[selected]);
  setOutput("route_index", static_cast<unsigned>(selected));
  next_index_ = (selected + 1) % route.size();

  RCLCPP_INFO(
    node_->get_logger(), "Preset '%s' selected patrol[%zu]=%s",
    preset.c_str(), selected, route[selected].c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree
