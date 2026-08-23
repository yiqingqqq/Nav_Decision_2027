#include "action/select_controller.hpp"

namespace rm_behavior_tree {

SelectControllerAction::SelectControllerAction(const std::string &name,
                                               const BT::NodeConfig &conf,
                                               const BT::RosNodeParams &params)
    : RosTopicPubNode(name, conf, params) {}

bool SelectControllerAction::setMessage(std_msgs::msg::String &msg) {
  const auto controller_id = getInput<std::string>("controller_id");
  if (!controller_id || controller_id->empty()) {
    return false;
  }
  msg.data = controller_id.value();
  return true;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SelectControllerAction,
                    "SelectController");
