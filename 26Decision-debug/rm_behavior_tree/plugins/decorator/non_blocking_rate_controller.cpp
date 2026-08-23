#include "decorator/non_blocking_rate_controller.hpp"

#include <stdexcept>

namespace rm_behavior_tree
{

NonBlockingRateController::NonBlockingRateController(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf), first_tick_(true)
{
  double hz = 10.0;
  getInput("hz", hz);
  if (hz <= 0.0) {
    throw std::invalid_argument("NonBlockingRateController requires hz > 0");
  }
  period_ = std::chrono::duration<double>(1.0 / hz);
}

BT::NodeStatus NonBlockingRateController::tick()
{
  const auto now = std::chrono::steady_clock::now();
  if (!first_tick_ && now - last_tick_ < period_) {
    // The periodic publisher must never gate or halt the navigation sibling.
    return BT::NodeStatus::SUCCESS;
  }

  first_tick_ = false;
  last_tick_ = now;
  const auto child_status = child_node_->executeTick();
  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::NonBlockingRateController>(
    "NonBlockingRateController");
}
