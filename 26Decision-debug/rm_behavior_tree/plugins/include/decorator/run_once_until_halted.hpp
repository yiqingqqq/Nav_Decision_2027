#ifndef RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUN_ONCE_UNTIL_HALTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUN_ONCE_UNTIL_HALTED_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/decorator_node.h"

namespace rm_behavior_tree
{

// Latches successful missions and retries failures after a short backoff.
// A parent branch transition halts and unlocks the mission, allowing a fresh
// route selection (and therefore a new tunnel direction).
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
  std::chrono::steady_clock::time_point retry_after_{};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__RUN_ONCE_UNTIL_HALTED_HPP_
