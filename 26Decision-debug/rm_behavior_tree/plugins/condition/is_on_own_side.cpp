#include "condition/is_on_own_side.hpp"

namespace rm_behavior_tree {

BT::NodeStatus IsOnOwnSideCondition::tick() {
  const auto pose =
      getInput<geometry_msgs::msg::TransformStamped>("current_location");
  const auto robot_id = getInput<uint8_t>("robot_id");
  const auto red_exit_x = getInput<double>("red_exit_x");
  const auto blue_exit_x = getInput<double>("blue_exit_x");
  if (!pose || !robot_id || !red_exit_x || !blue_exit_x) {
    return BT::NodeStatus::FAILURE;
  }

  const double x = pose->transform.translation.x;
  if (robot_id.value() == 0) {
    return x < red_exit_x.value() ? BT::NodeStatus::SUCCESS
                                  : BT::NodeStatus::FAILURE;
  }
  if (robot_id.value() == 1) {
    return x > blue_exit_x.value() ? BT::NodeStatus::SUCCESS
                                   : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior_tree::IsOnOwnSideCondition>(
      "IsOnOwnSide");
}
