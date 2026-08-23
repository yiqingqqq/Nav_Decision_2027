#include "condition/is_HP_level.hpp"

namespace rm_behavior_tree
{

IsHPLevelCondition::IsHPLevelCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHPLevelCondition::checkIsHPLevel, this), config)
{
}

BT::NodeStatus IsHPLevelCondition::checkIsHPLevel()
{
  auto msg = getInput<uint16_t>("message");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  int hp_threshold = 200;  // 默认阈值
  getInput("hp_threshold", hp_threshold);

  uint16_t current_hp = msg.value();

  if (current_hp <= static_cast<uint16_t>(hp_threshold)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsHPLevelCondition>("IsHPLevel");
}
