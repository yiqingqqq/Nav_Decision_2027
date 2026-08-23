#include "action/sub_robot_id.hpp"

namespace rm_behavior_tree {

SubRobotIDAction::SubRobotIDAction(const std::string &name,
                                   const BT::NodeConfig &conf,
                                   const BT::RosNodeParams &params)
    : BT::RosTopicSubNode<std_msgs::msg::UInt8>(name, conf, params) {}

BT::NodeStatus SubRobotIDAction::onTick(
    const std::shared_ptr<std_msgs::msg::UInt8> &last_msg) {
  if (!last_msg) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("robot_id", last_msg->data);
  return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotIDAction, "SubRobotID");
