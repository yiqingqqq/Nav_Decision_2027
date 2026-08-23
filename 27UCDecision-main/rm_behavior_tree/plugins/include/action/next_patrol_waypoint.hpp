#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__NEXT_PATROL_WAYPOINT_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__NEXT_PATROL_WAYPOINT_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class NextPatrolWaypointAction : public BT::SyncActionNode
{
public:
  NextPatrolWaypointAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("preset"),
      BT::OutputPort<std::string>("waypoint"),
      BT::OutputPort<unsigned>("route_index")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string last_preset_;
  std::size_t next_index_{0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__NEXT_PATROL_WAYPOINT_HPP_
