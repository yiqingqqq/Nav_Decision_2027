#include "condition/is_robot_id.hpp"

namespace rm_behavior_tree {

BT::NodeStatus IsRobotIDCondition::tick() {
  const auto robot_id = getInput<uint8_t>("robot_id");
  const auto expected = getInput<uint8_t>("expected");
  if (!robot_id || !expected) {
    return BT::NodeStatus::FAILURE;
  }
  return robot_id.value() == expected.value() ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior_tree::IsRobotIDCondition>("IsRobotID");
}
