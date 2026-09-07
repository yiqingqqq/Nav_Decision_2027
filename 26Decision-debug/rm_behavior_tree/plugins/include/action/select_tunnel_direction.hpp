#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_TUNNEL_DIRECTION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_TUNNEL_DIRECTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace rm_behavior_tree
{

class SelectTunnelDirectionAction : public BT::SyncActionNode
{
public:
  SelectTunnelDirectionAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  static geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw);
  static geometry_msgs::msg::PoseStamped worldToMap(double x, double y, double yaw);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SELECT_TUNNEL_DIRECTION_HPP_
