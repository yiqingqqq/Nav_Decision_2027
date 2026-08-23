#include "action/send_spin.hpp"
#include "std_msgs/msg/float32.hpp"

namespace rm_behavior_tree
{

SendSpinAction::SendSpinAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode(name, conf, params)
{
}

bool SendSpinAction::setMessage(std_msgs::msg::Float32 & msg)
{
  // 获取输入参数
  bool is_spin = false;      // 是否转动
  float spin_velocity = 0.5; // 转速 (rad/s)
  getInput("is_spin", is_spin);
  getInput("spin_velocity", spin_velocity);

  // 根据 is_spin 决定是否转动
  if (is_spin) {
    msg.data = spin_velocity;
  } else {
    msg.data = 0.0;
  }

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendSpinAction, "SendSpin");
