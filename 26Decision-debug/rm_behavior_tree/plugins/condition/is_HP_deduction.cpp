#include "condition/is_HP_deduction.hpp"

namespace rm_behavior_tree
{

IsHPdeductionCondition::IsHPdeductionCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHPdeductionCondition::checkHPdeduction, this), config),
  first_tick_(true),
  last_hp_(0),
  last_hit_time_(0, 0, RCL_ROS_TIME) 
{
}

BT::NodeStatus IsHPdeductionCondition::checkHPdeduction()
{
  auto current_hp_opt = getInput<uint16_t>("message");

  if (!current_hp_opt) {
    return BT::NodeStatus::FAILURE;
  }

  uint16_t current_hp = current_hp_opt.value();
  auto current_time = rclcpp::Clock(RCL_ROS_TIME).now(); 

  if (first_tick_) {
    last_hp_ = current_hp;
    first_tick_ = false;
    last_hit_time_ = current_time - rclcpp::Duration::from_seconds(10.0);
    return BT::NodeStatus::FAILURE;
  }

  if (last_hp_ > current_hp) {
    last_hit_time_ = current_time;
  }

  last_hp_ = current_hp;

  if ((current_time - last_hit_time_).seconds() < 3.0) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsHPdeductionCondition>("IsHPdeduction");
}