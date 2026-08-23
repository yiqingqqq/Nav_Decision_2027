#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_CONTROLLER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_CONTROLLER_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace rm_behavior_tree {

class SelectControllerAction
    : public BT::RosTopicPubNode<std_msgs::msg::String> {
public:
  SelectControllerAction(const std::string &name, const BT::NodeConfig &conf,
                         const BT::RosNodeParams &params);

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<std::string>("controller_id")});
  }

  bool setMessage(std_msgs::msg::String &msg) override;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_CONTROLLER_HPP_
