#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_ID_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_ID_HPP_

#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree {

class IsRobotIDCondition : public BT::ConditionNode {
public:
  IsRobotIDCondition(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<uint8_t>("robot_id"),
            BT::InputPort<uint8_t>("expected")};
  }

  BT::NodeStatus tick() override;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_ID_HPP_
