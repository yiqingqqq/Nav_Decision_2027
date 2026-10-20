#include "action/send_goal.hpp"
#include "action/heading_policy.hpp"
#include "bt_conversions.hpp"
#include <random>

namespace rm_behavior_tree
{

SendGoalAction::SendGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
  if (auto node = params.nh.lock()) {
    goal_checker_pub_ = node->create_publisher<std_msgs::msg::String>(
      "/goal_checker_selector", rclcpp::QoS(1).transient_local().reliable());
  }
}

bool SendGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (!res) {
    RCLCPP_ERROR(logger(), "error reading port [goal_pose]");
    return false;
  }
  goal.pose = res.value();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  const auto policy = getInput<std::string>("heading_policy");
  if (!policy) {
    RCLCPP_ERROR(logger(), "error reading port [heading_policy]");
    return false;
  }
  std::optional<geometry_msgs::msg::PoseStamped> current;
  if (policy.value() == "hold") {
    const auto pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
    if (pose) current = pose.value();
  }
  try {
    applyHeadingPolicy(goal.pose, policy.value(), current);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(logger(), "Invalid navigation heading: %s", e.what());
    return false;
  }

  std_msgs::msg::String checker;
  const auto checker_override = getInput<std::string>("goal_checker_id");
  checker.data = checker_override && !checker_override->empty() ? checker_override.value() :
    (policy.value() == "hold" ? "position_goal_checker" : "general_goal_checker");
  if (!goal_checker_pub_) return false;
  goal_checker_pub_->publish(checker);

  auto move_opt = getInput<bool>("move");
  double length = 1.0;
  if (auto length_opt = getInput<double>("length")) {
    length = length_opt.value();
  }
  if (move_opt && move_opt.value()) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-length / 2.0, length / 2.0);
    goal.pose.pose.position.x += dis(gen);
    goal.pose.pose.position.y += dis(gen);
  }

  RCLCPP_INFO(
    logger(), "[%s] submit NavigateToPose: (%.2f, %.2f), heading_policy=%s, yaw=%.3f",
    name().c_str(), goal.pose.pose.position.x, goal.pose.pose.position.y,
    policy.value().c_str(),
    2.0 * std::atan2(goal.pose.pose.orientation.z, goal.pose.pose.orientation.w));

  std::cout << "Goal_pose: [ "
    << std::fixed << std::setprecision(1)
    << goal.pose.pose.position.x << ", "
    << goal.pose.pose.position.y << ", "
    << goal.pose.pose.position.z << ", "
    << goal.pose.pose.orientation.x << ", "
    << goal.pose.pose.orientation.y << ", "
    << goal.pose.pose.orientation.z << ", "
    << goal.pose.pose.orientation.w << " ]\n";

  return true;
}

void SendGoalAction::halt()
{
  try {
    RosActionNode<nav2_msgs::action::NavigateToPose>::halt();
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger(), "Safe halt: goal handle was already invalid or finished.");
  }
    resetStatus();
}

void SendGoalAction::onHalt()
{
  RCLCPP_INFO(logger(), "SendGoalAction has been halted.");
}

BT::NodeStatus SendGoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Success!!!");
      return BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger(), "Goal was canceled");
      std::cout << "Goal was canceled" << '\n';
      return BT::NodeStatus::FAILURE;
      break;
    default:
      RCLCPP_INFO(logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
      break;
  }
}

BT::NodeStatus SendGoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/)
{
  // std::cout << "Distance remaining: " << feedback->distance_remaining << '\n';
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  //RCLCPP_ERROR(logger(), "SendGoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");
