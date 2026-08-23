#ifndef RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__NON_BLOCKING_RATE_CONTROLLER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__NON_BLOCKING_RATE_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace rm_behavior_tree
{

// Periodically ticks an immediate child without blocking later siblings in a
// ReactiveSequence while waiting for the next period.
class NonBlockingRateController : public BT::DecoratorNode
{
public:
  NonBlockingRateController(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("hz", 10.0, "Rate")};
  }

private:
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::steady_clock> last_tick_;
  std::chrono::duration<double> period_;
  bool first_tick_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__DECORATOR__NON_BLOCKING_RATE_CONTROLLER_HPP_
