#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_SPIN_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_SPIN_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace rm_behavior_tree
{

class SendSpinAction : public BT::RosTopicPubNode<std_msgs::msg::Float32>
{
public:
  SendSpinAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(std_msgs::msg::Float32 & msg) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("is_spin"),        // 是否转动
      BT::InputPort<float>("spin_velocity") // 转速 (rad/s)
    });
  }

private:
  // 匀速转动，不需要保存时间状态
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_SPIN_HPP_
