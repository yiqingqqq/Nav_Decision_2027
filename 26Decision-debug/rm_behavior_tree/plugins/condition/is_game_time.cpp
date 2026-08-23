#include "condition/is_game_time.hpp"

namespace rm_behavior_tree
{

IsGameTimeCondition::IsGameTimeCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameTimeCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameTimeCondition::checkGameStart()
{
  int game_progress;
  auto msg = getInput<uint8_t>("message");
  getInput("game_progress", game_progress);

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  if (msg.value() == game_progress) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsGameTimeCondition>("IsGameTime");
}
