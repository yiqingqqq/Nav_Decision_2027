#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_DEDUCTION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_DEDUCTION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp" 

namespace rm_behavior_tree
{

/**
 * @brief Condition节点，用于判断血量是否下降
 *
 * 读取黑板中的当前血量，与上次记录的血量进行对比。
 * 如果上一次血量 > 当前血量，说明发生了扣血，记录当前时间。
 * 在受击后的 3 秒内，持续返回 SUCCESS。
 * 否则返回 FAILURE。
 *
 * @param[in] message 当前血量 (uint16_t)
 */
class IsHPdeductionCondition : public BT::SimpleConditionNode
{
public:
  IsHPdeductionCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkHPdeduction();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint16_t>("message")};
  }

private:
  bool first_tick_;
  uint16_t last_hp_;
  rclcpp::Time last_hit_time_;  // 新增：记录最后一次受击的时间戳
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_DEDUCTION_HPP_