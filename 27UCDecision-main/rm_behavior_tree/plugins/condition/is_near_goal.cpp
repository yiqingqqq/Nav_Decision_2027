#include "condition/is_near_goal.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

namespace rm_behavior_tree
{

IsNearGoalCondition::IsNearGoalCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsNearGoalCondition::checkIsNearGoal, this), config)
{
}

BT::NodeStatus IsNearGoalCondition::checkIsNearGoal()
{
  // 获取输入端口数据
  auto current_location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  auto distance_threshold_res = getInput<double>("distance_threshold");

  // 检查是否获取成功
  if (!current_location || !goal_pose) {
    return BT::NodeStatus::FAILURE;
  }

  double distance_threshold = 2.0; 
  if (distance_threshold_res) {
    distance_threshold = distance_threshold_res.value();
  }

  // 计算欧氏距离 (只比较 x, y 坐标，忽略 z)
  const auto & curr_trans = current_location->transform.translation;
  const auto & goal_pos = goal_pose->pose.position;

  double dx = curr_trans.x - goal_pos.x;
  double dy = curr_trans.y - goal_pos.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 判断是否在阈值范围内
  if (distance < distance_threshold) {
    //RCLCPP_INFO(rclcpp::get_logger("is_near_goal"), "Near goal! distance=%.3f, threshold=%.3f", distance, distance_threshold);
    return BT::NodeStatus::SUCCESS;
  } else {
    //RCLCPP_INFO(rclcpp::get_logger("is_near_goal"), "Not near goal. distance=%.3f, threshold=%.3f", distance, distance_threshold);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsNearGoalCondition>("IsNearGoal");
}
