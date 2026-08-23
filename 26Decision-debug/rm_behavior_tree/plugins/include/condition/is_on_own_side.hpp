#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ON_OWN_SIDE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ON_OWN_SIDE_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace rm_behavior_tree {

class IsOnOwnSideCondition : public BT::ConditionNode {
public:
  IsOnOwnSideCondition(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
        BT::InputPort<uint8_t>("robot_id"),
        BT::InputPort<double>("red_exit_x", 3.5,
                              "Red-side tunnel exit x coordinate"),
        BT::InputPort<double>("blue_exit_x", -3.5,
                              "Blue-side tunnel exit x coordinate")};
  }

  BT::NodeStatus tick() override;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ON_OWN_SIDE_HPP_
