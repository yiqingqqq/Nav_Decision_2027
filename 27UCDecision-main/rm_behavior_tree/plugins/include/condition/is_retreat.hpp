#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RETREAT_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RETREAT_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "std_msgs/msg/u_int16.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点，用于判断机器人是否需要撤退
 *
 * 该节点从输入端口获取机器人血量，采用双阈值迟滞比较逻辑：
 * 1. 当血量 < hp_threshold_retreat 时，触发撤退状态，返回 SUCCESS。
 * 2. 在撤退状态下，只有当血量 >= hp_threshold_recover 时，才解除撤退状态，返回 FAILURE。
 * @param[in] message 机器人当前血量
 * @param[in] hp_threshold_retreat 触发撤退的极低血量阈值
 * @param[in] hp_threshold_recover 解除撤退的健康血量阈值
 */
class IsRetreatCondition : public BT::SimpleConditionNode
{
public:
  IsRetreatCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkIsRetreat();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint16_t>("message"),
      BT::InputPort<int>("hp_threshold_retreat"),
      BT::InputPort<int>("hp_threshold_recover")};
  }

private:
  bool is_retreating_ = false;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RETREAT_HPP_