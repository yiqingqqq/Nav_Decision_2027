#include "condition/is_retreat.hpp"

namespace rm_behavior_tree
{

IsRetreatCondition::IsRetreatCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRetreatCondition::checkIsRetreat, this), config)
{
}

BT::NodeStatus IsRetreatCondition::checkIsRetreat()
{
  auto msg = getInput<uint16_t>("message");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  int hp_threshold_retreat = 110;
  int hp_threshold_recover = 400;
  getInput("hp_threshold_retreat", hp_threshold_retreat);
  getInput("hp_threshold_recover", hp_threshold_recover);

  uint16_t current_hp = msg.value();

  // 双阈值迟滞比较逻辑
  if (is_retreating_) {
    if (current_hp >= hp_threshold_recover) {
      is_retreating_ = false;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  } else {
    if (current_hp < hp_threshold_retreat) {
      is_retreating_ = true;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsRetreatCondition>("IsRetreat");
}
