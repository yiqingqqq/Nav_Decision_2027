#include "action/sub_game_status.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace rm_behavior_tree
{

SubGameStatusAction::SubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::UInt8>(name, conf, params)
{
}

BT::NodeStatus SubGameStatusAction::onTick(
  const std::shared_ptr<std_msgs::msg::UInt8> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, game_progress: %u", name().c_str(), last_msg->data);
    setOutput("game_progress", last_msg->data);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");