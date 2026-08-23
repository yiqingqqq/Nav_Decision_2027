#ifndef RM_NAVIGATION__TUNNEL_CONTROLLER_SELECTOR_HPP_
#define RM_NAVIGATION__TUNNEL_CONTROLLER_SELECTOR_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/path.hpp"

namespace rm_navigation
{

class TunnelControllerSelector : public BT::SyncActionNode
{
public:
  TunnelControllerSelector(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace rm_navigation

#endif  // RM_NAVIGATION__TUNNEL_CONTROLLER_SELECTOR_HPP_
