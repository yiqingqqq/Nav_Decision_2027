#include "action/select_tunnel_direction.hpp"

#include <cmath>

namespace rm_behavior_tree
{

BT::PortsList SelectTunnelDirectionAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
    BT::InputPort<uint8_t>("robot_id"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("entry_goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("inside_goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("exit_goal")};
}

geometry_msgs::msg::PoseStamped SelectTunnelDirectionAction::makePose(
  double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

BT::NodeStatus SelectTunnelDirectionAction::tick()
{
  const auto location = getInput<geometry_msgs::msg::TransformStamped>("current_location");
  const auto robot_id = getInput<uint8_t>("robot_id");
  if (!location || !robot_id || robot_id.value() > 1) {
    return BT::NodeStatus::FAILURE;
  }

  // RMUL2027 red-side field coordinates are expressed in the serialized map
  // frame whose origin is the Gazebo spawn pose (5.0, -3.0, pi).
  constexpr double kPi = 3.14159265358979323846;
  const bool red_side = robot_id.value() == 0;
  const double yaw_a_to_b = kPi;
  const double yaw_b_to_a = 0.0;
  const auto endpoint_a = red_side ?
    makePose(4.5, 0.6, yaw_a_to_b) : makePose(-0.5, 3.6, yaw_a_to_b);
  const auto endpoint_b = red_side ?
    makePose(1.0, 0.6, yaw_b_to_a) : makePose(-4.0, 3.6, yaw_b_to_a);
  const double inside_x = red_side ? 2.5 : -2.5;
  const double inside_y = red_side ? 0.6 : 3.6;

  const double x = location->transform.translation.x;
  const double y = location->transform.translation.y;
  const auto squared_distance = [x, y](const geometry_msgs::msg::PoseStamped & pose) {
      const double dx = pose.pose.position.x - x;
      const double dy = pose.pose.position.y - y;
      return dx * dx + dy * dy;
    };

  const bool a_is_nearer = squared_distance(endpoint_a) <= squared_distance(endpoint_b);
  const double tunnel_yaw = a_is_nearer ? yaw_a_to_b : yaw_b_to_a;
  const auto & entry = a_is_nearer ? endpoint_a : endpoint_b;
  const auto & exit = a_is_nearer ? endpoint_b : endpoint_a;
  setOutput(
    "entry_goal",
    makePose(entry.pose.position.x, entry.pose.position.y, tunnel_yaw));
  setOutput("inside_goal", makePose(inside_x, inside_y, tunnel_yaw));
  setOutput(
    "exit_goal",
    makePose(exit.pose.position.x, exit.pose.position.y, tunnel_yaw));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectTunnelDirectionAction>(
    "SelectTunnelDirection");
}
