#include "action/sub_robot_status.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace rm_behavior_tree
{

SubRobotStatusAction::SubRobotStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, conf, params)
{
}

BT::NodeStatus SubRobotStatusAction::onTick(
  const std::shared_ptr<std_msgs::msg::UInt16> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, feedback_robot_hp: %u", name().c_str(), last_msg->data);
    setOutput("feedback_robot_hp", last_msg->data);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotStatusAction, "SubRobotStatus");