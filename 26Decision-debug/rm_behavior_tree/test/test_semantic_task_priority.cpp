#include <gtest/gtest.h>
#include <thread>
#include "behaviortree_cpp/bt_factory.h"
#include "decorator/run_once_until_halted.hpp"

TEST(SemanticTaskPriority, FailedRetreatRetriesWithoutStartingCenter)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<rm_behavior_tree::RunOnceUntilHalted>("Mission");
  bool low_hp = true;
  int returns = 0, centers = 0;
  factory.registerSimpleCondition("LowHP", [&](BT::TreeNode &) {
    return low_hp ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  });
  factory.registerSimpleAction("Return", [&](BT::TreeNode &) {
    ++returns; return BT::NodeStatus::FAILURE;
  });
  factory.registerSimpleAction("Center", [&](BT::TreeNode &) {
    ++centers; return BT::NodeStatus::SUCCESS;
  });
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4"><BehaviorTree ID="Main">
      <WhileDoElse><LowHP/><Mission><Return/></Mission><Center/></WhileDoElse>
    </BehaviorTree></root>)");
  for (int i = 0; i < 5; ++i) EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(returns, 1);
  EXPECT_EQ(centers, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(returns, 2);
  EXPECT_EQ(centers, 0);
  low_hp = false;
  EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(centers, 1);
}

TEST(SemanticTaskPriority, SuccessfulMissionRemainsLatched)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<rm_behavior_tree::RunOnceUntilHalted>("Mission");
  int calls = 0;
  factory.registerSimpleAction("Return", [&](BT::TreeNode &) {
    ++calls; return BT::NodeStatus::SUCCESS;
  });
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4"><BehaviorTree ID="Main"><Mission><Return/></Mission></BehaviorTree></root>)");
  for (int i = 0; i < 5; ++i) EXPECT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, 1);
}
