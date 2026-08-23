#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__LOAD_STRATEGY_PRESET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__LOAD_STRATEGY_PRESET_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class LoadStrategyPresetAction : public BT::SyncActionNode
{
public:
  LoadStrategyPresetAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("active_preset"),
      BT::OutputPort<std::string>("supply_waypoint"),
      BT::OutputPort<std::string>("fallback_waypoint"),
      BT::OutputPort<std::string>("hold_waypoint"),
      BT::OutputPort<std::string>("spin_waypoint"),
      BT::OutputPort<int>("hp_retreat"),
      BT::OutputPort<int>("hp_recover"),
      BT::OutputPort<double>("damage_memory_s"),
      BT::OutputPort<unsigned>("navigation_timeout_ms"),
      BT::OutputPort<unsigned>("navigation_retries")};
  }

  BT::NodeStatus tick() override;

private:
  bool readWaypoint(
    const std::string & prefix, const std::string & key,
    const std::string & output_port);

  rclcpp::Node::SharedPtr node_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__LOAD_STRATEGY_PRESET_HPP_
