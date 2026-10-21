#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using Action = nav2_msgs::action::FollowPath;
using namespace std::chrono_literals;

class MockPathAction : public BT::RosActionNode<Action>
{
public:
  using RosActionNode::RosActionNode;
  bool setGoal(Goal & goal) override {goal.path.poses.resize(2); return true;}
  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    return result.code == rclcpp_action::ResultCode::SUCCEEDED ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ActionCancellation : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server_node = std::make_shared<rclcpp::Node>("mock_path_server");
    client_node = std::make_shared<rclcpp::Node>("mock_path_client");
    server = rclcpp_action::create_server<Action>(
      server_node, "/test_heading_cancel/follow_path",
      [this](const auto &, auto) {
        return reject ? rclcpp_action::GoalResponse::REJECT :
          rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto) {++cancel_count; return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        ++accepted_count;
        if(finish_immediately) handle->succeed(std::make_shared<Action::Result>());
        else active_handle = handle;
      });
    timer = server_node->create_wall_timer(5ms, [this] {
      if(active_handle && active_handle->is_canceling()) {
        active_handle->canceled(std::make_shared<Action::Result>());
        active_handle.reset();
      }
    });
    executor.add_node(server_node);
    thread = std::thread([this] {executor.spin();});
  }

  void TearDown() override
  {
    executor.cancel();
    thread.join();
  }

  std::unique_ptr<MockPathAction> action()
  {
    BT::RosNodeParams params(client_node, "/test_heading_cancel/follow_path");
    params.wait_for_server_timeout = 2s;
    params.server_timeout = 1s;
    return std::make_unique<MockPathAction>("MockPath", BT::NodeConfig{}, params);
  }

  rclcpp::Node::SharedPtr server_node, client_node;
  rclcpp_action::Server<Action>::SharedPtr server;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> active_handle;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  std::atomic<bool> reject{false}, finish_immediately{true};
  std::atomic<int> accepted_count{0}, cancel_count{0};
};

TEST_F(ActionCancellation, CompletedGoalThenImmediateCancelOfNewGoal)
{
  auto node = action();
  auto state = node->executeTick();
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while(state == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
    state = node->executeTick();
  }
  ASSERT_EQ(state, BT::NodeStatus::SUCCESS);
  node->halt();
  ASSERT_EQ(node->status(), BT::NodeStatus::IDLE);

  finish_immediately = false;
  ASSERT_EQ(node->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_NO_THROW(node->halt());
  EXPECT_EQ(node->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(accepted_count, 2);
  EXPECT_EQ(cancel_count, 1);
  EXPECT_NO_THROW(node->halt());
}

TEST_F(ActionCancellation, RejectedGoalCanBeHaltedBeforeResponseIsProcessed)
{
  reject = true;
  auto node = action();
  ASSERT_EQ(node->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_NO_THROW(node->halt());
  EXPECT_EQ(node->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(cancel_count, 0);
}
