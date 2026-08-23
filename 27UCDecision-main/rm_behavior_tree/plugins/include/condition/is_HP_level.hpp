#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_LEVEL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_LEVEL_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "std_msgs/msg/u_int16.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，判断机器人血量是否低于指定阈值
 *
 * 当血量 < hp_threshold 时返回 SUCCESS，否则返回 FAILURE。
 *
 * @param[in] message    机器人当前血量
 * @param[in] hp_threshold 血量阈值，低于此值时触发 SUCCESS
 */
class IsHPLevelCondition : public BT::SimpleConditionNode
{
public:
  IsHPLevelCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkIsHPLevel();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint16_t>("message"),
      BT::InputPort<int>("hp_threshold")};
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_LEVEL_HPP_
