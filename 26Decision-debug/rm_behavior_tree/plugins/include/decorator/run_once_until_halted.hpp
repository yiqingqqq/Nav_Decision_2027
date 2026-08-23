#ifndef RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUN_ONCE_UNTIL_HALTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUN_ONCE_UNTIL_HALTED_HPP_

#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace rm_behavior_tree
{

// Runs a mission subtree once, then keeps returning its result. A halt from a
// parent branch transition starts a new mission and allows the subtree to run
// again (and therefore select a new tunnel direction).
class RunOnceUntilHalted : public BT::DecoratorNode
{
public:
  RunOnceUntilHalted(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf) {}

  static BT::PortsList providedPorts() { return {}; }

private:
  BT::NodeStatus tick() override;
  void halt() override;

  bool completed_{false};
  BT::NodeStatus completed_status_{BT::NodeStatus::IDLE};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUN_ONCE_UNTIL_HALTED_HPP_
