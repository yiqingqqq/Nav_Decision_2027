#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_TUNNEL_PATH_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_TUNNEL_PATH_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"

namespace rm_behavior_tree
{

class FollowTunnelPathAction : public BT::RosActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowTunnelPathAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & result) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__FOLLOW_TUNNEL_PATH_HPP_
