#include "action/load_strategy_preset.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

namespace rm_behavior_tree
{

LoadStrategyPresetAction::LoadStrategyPresetAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(std::move(node))
{
  if (!node_) {
    throw std::invalid_argument("LoadStrategyPreset requires a ROS node");
  }
}

bool LoadStrategyPresetAction::readWaypoint(
  const std::string & prefix, const std::string & key,
  const std::string & output_port)
{
  const std::string parameter = prefix + "." + key;
  if (!node_->has_parameter(parameter)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Strategy preset is missing parameter '%s'",
      parameter.c_str());
    return false;
  }
  const auto waypoint = node_->get_parameter(parameter).as_string();
  if (waypoint.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Strategy preset parameter '%s' is empty",
      parameter.c_str());
    return false;
  }
  setOutput(output_port, waypoint);
  return true;
}

BT::NodeStatus LoadStrategyPresetAction::tick()
{
  const auto preset = node_->get_parameter("strategy.active_preset").as_string();
  const std::string prefix = "strategy.presets." + preset;
  const std::string route_parameter = prefix + ".patrol_route";

  if (!node_->has_parameter(route_parameter)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Unknown strategy preset '%s'", preset.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto route = node_->get_parameter(route_parameter).as_string_array();
  if (route.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Strategy preset '%s' has an empty patrol route",
      preset.c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("active_preset", preset);
  if (!readWaypoint(prefix, "supply_waypoint", "supply_waypoint") ||
    !readWaypoint(prefix, "fallback_waypoint", "fallback_waypoint") ||
    !readWaypoint(prefix, "hold_waypoint", "hold_waypoint") ||
    !readWaypoint(prefix, "spin_near_waypoint", "spin_waypoint"))
  {
    return BT::NodeStatus::FAILURE;
  }

  const int hp_retreat =
    node_->get_parameter("strategy.safety.hp_retreat").as_int();
  const int hp_recover =
    node_->get_parameter("strategy.safety.hp_recover").as_int();
  const double damage_memory_s =
    node_->get_parameter("strategy.safety.damage_memory_s").as_double();
  const double navigation_timeout_s =
    node_->get_parameter("strategy.safety.navigation_timeout_s").as_double();
  const int navigation_retries =
    node_->get_parameter("strategy.safety.navigation_retries").as_int();
  if (hp_retreat < 0 || hp_recover <= hp_retreat ||
    damage_memory_s <= 0.0 || navigation_timeout_s <= 0.0 ||
    navigation_retries < 1)
  {
    RCLCPP_ERROR(node_->get_logger(), "Strategy safety parameters are invalid");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("hp_retreat", hp_retreat);
  setOutput("hp_recover", hp_recover);
  setOutput("damage_memory_s", damage_memory_s);
  setOutput(
    "navigation_timeout_ms",
    static_cast<unsigned>(navigation_timeout_s * 1000.0));
  setOutput("navigation_retries", navigation_retries);

  RCLCPP_INFO(
    node_->get_logger(), "Loaded strategy preset '%s' with %zu patrol waypoints",
    preset.c_str(), route.size());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree
