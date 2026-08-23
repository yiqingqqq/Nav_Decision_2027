#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace rm_behavior_tree
{

/**
 * @brief Condition节点，用于判断机器人是否接近目标点
 *
 * 该节点计算机器人当前位置与目标点之间的欧氏距离，
 * 如果距离小于阈值则返回成功；否则返回失败。
 *
 * @param[in] current_location 机器人当前位置 (TransformStamped)
 * @param[in] goal_pose 目标点 (PoseStamped)
 * @param[in] distance_threshold 距离阈值 (米)
 * @return BT::NodeStatus 是否接近目标点
 */
class IsNearGoalCondition : public BT::SimpleConditionNode
{
public:
  IsNearGoalCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkIsNearGoal();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
      BT::InputPort<double>("distance_threshold")};
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_HPP_
