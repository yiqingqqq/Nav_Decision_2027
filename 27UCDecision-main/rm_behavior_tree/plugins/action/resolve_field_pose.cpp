#include "action/resolve_field_pose.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace rm_behavior_tree
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

ResolveFieldPoseAction::ResolveFieldPoseAction(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(std::move(node))
{
  if (!node_) {
    throw std::invalid_argument("ResolveFieldPose requires a ROS node");
  }
}

BT::NodeStatus ResolveFieldPoseAction::tick()
{
  const auto waypoint = getInput<std::string>("waypoint");
  if (!waypoint || waypoint->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ResolveFieldPose: missing waypoint name");
    return BT::NodeStatus::FAILURE;
  }

  const std::string prefix = "field_map.waypoints." + waypoint.value();
  const std::string x_name = prefix + ".x";
  const std::string y_name = prefix + ".y";
  if (!node_->has_parameter(x_name) || !node_->has_parameter(y_name)) {
    RCLCPP_ERROR(
      node_->get_logger(), "ResolveFieldPose: waypoint '%s' is not configured",
      waypoint->c_str());
    return BT::NodeStatus::FAILURE;
  }

  double field_x = node_->get_parameter(x_name).as_double();
  double field_y = node_->get_parameter(y_name).as_double();
  double field_yaw = parameterOr(prefix + ".yaw", 0.0);
  const double field_length = parameterOr("field_map.length", 28.0);
  const double field_width = parameterOr("field_map.width", 15.0);
  const std::string alliance = parameterOr<std::string>("field_map.alliance", "red");
  const bool mirror_for_blue = parameterOr("field_map.mirror_for_blue", true);

  if (field_x < 0.0 || field_x > field_length || field_y < 0.0 || field_y > field_width) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ResolveFieldPose: waypoint '%s' (%.3f, %.3f) is outside %.1f x %.1f m field",
      waypoint->c_str(), field_x, field_y, field_length, field_width);
    return BT::NodeStatus::FAILURE;
  }

  if (alliance == "blue" && mirror_for_blue) {
    field_x = field_length - field_x;
    field_y = field_width - field_y;
    field_yaw = normalizeAngle(field_yaw + kPi);
  } else if (alliance != "red") {
    RCLCPP_ERROR(
      node_->get_logger(), "ResolveFieldPose: alliance must be 'red' or 'blue', got '%s'",
      alliance.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double origin_x = parameterOr("field_map.field_to_map.x", 0.0);
  const double origin_y = parameterOr("field_map.field_to_map.y", 0.0);
  const double origin_yaw = parameterOr("field_map.field_to_map.yaw", 0.0);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double map_yaw = normalizeAngle(field_yaw + origin_yaw);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = parameterOr<std::string>("field_map.frame_id", "map");
  pose.header.stamp = node_->get_clock()->now();
  pose.pose.position.x = origin_x + cos_yaw * field_x - sin_yaw * field_y;
  pose.pose.position.y = origin_y + sin_yaw * field_x + cos_yaw * field_y;
  pose.pose.orientation.z = std::sin(map_yaw / 2.0);
  pose.pose.orientation.w = std::cos(map_yaw / 2.0);

  setOutput("goal_pose", pose);
  setOutput("frame_id", pose.header.frame_id);

  if (!parameterOr("field_map.calibrated", false)) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "RMUC field map is using nominal waypoints. Calibrate field_to_map and waypoints before a real match.");
  }
  RCLCPP_INFO(
    node_->get_logger(), "Resolved %s/%s -> %s (%.2f, %.2f, %.2f rad)",
    alliance.c_str(), waypoint->c_str(), pose.header.frame_id.c_str(),
    pose.pose.position.x, pose.pose.position.y, map_yaw);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree
