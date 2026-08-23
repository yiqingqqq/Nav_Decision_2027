#include "action/sub_deduction_reason.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace rm_behavior_tree
{

SubDeductionReasonAction::SubDeductionReasonAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<std_msgs::msg::UInt8>(name, conf, params)
{
}

BT::NodeStatus SubDeductionReasonAction::onTick(
  const std::shared_ptr<std_msgs::msg::UInt8> & last_msg)
{
  if (last_msg)
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, deduction_reason: %u", name().c_str(), last_msg->data);
    setOutput("deduction_reason", last_msg->data);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubDeductionReasonAction, "SubDeductionReason");