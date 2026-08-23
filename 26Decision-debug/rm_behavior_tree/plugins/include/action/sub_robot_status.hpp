#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_STATUS_HPP_

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace rm_behavior_tree
{
class SubRobotStatusAction : public BT::RosTopicSubNode<std_msgs::msg::UInt16>
{
public:
  SubRobotStatusAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<uint16_t>("feedback_robot_hp")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<std_msgs::msg::UInt16> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_STATUS_HPP_