#include "action/send_spin.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace rm_behavior_tree
{

SendSpinAction::SendSpinAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode(name, conf, params)
{
}

bool SendSpinAction::setMessage(std_msgs::msg::Float32MultiArray & msg)
{
  bool is_spin = false;
  float spin_velocity = 0.5;
  getInput("is_spin", is_spin);
  getInput("spin_velocity", spin_velocity);

  // data[0] is explicit ownership. This distinguishes "hold wz=0" from
  // "release wz back to Nav2".
  msg.data = {is_spin ? 1.0F : 0.0F, is_spin ? spin_velocity : 0.0F};

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendSpinAction, "SendSpin");
