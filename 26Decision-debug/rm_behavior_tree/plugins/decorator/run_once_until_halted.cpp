#include "decorator/run_once_until_halted.hpp"

namespace rm_behavior_tree
{

BT::NodeStatus RunOnceUntilHalted::tick()
{
  if (completed_) {
    return completed_status_;
  }

  setStatus(BT::NodeStatus::RUNNING);
  const auto child_status = child_node_->executeTick();
  if (BT::isStatusCompleted(child_status)) {
    completed_ = true;
    completed_status_ = child_status;
    resetChild();
  }
  return child_status;
}

void RunOnceUntilHalted::halt()
{
  completed_ = false;
  completed_status_ = BT::NodeStatus::IDLE;
  DecoratorNode::halt();
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RunOnceUntilHalted>("RunOnceUntilHalted");
}
