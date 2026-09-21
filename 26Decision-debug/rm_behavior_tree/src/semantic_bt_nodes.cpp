#include "semantic/semantic_bt_nodes.hpp"

#include <tf2/time.h>
#include <utility>

namespace rm_behavior_tree
{
GetCurrentPoseAction::GetCurrentPoseAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(std::move(node))
{
  buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // lookupTransform() below uses a bounded timeout. It therefore needs a
  // listener-owned executor thread that can keep filling the buffer while the
  // behavior-tree tick is waiting.
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, true);
}
BT::PortsList GetCurrentPoseAction::providedPorts() {return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose")};}
BT::NodeStatus GetCurrentPoseAction::tick()
{
  try {
    const auto tf = buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.2));
    geometry_msgs::msg::PoseStamped pose; pose.header = tf.header;
    pose.pose.position.x = tf.transform.translation.x; pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z; pose.pose.orientation = tf.transform.rotation;
    last_valid_pose_ = pose;
    setOutput("current_pose", pose); return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "map->base_link unavailable: %s", e.what());
    // A transient TF miss must not make the outer ReactiveSequence cancel an
    // active Nav2 action. Startup still fails until the first real pose exists.
    if (last_valid_pose_) {
      auto pose = *last_valid_pose_;
      pose.header.stamp = node_->now();
      setOutput("current_pose", pose);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
}

ResolveFieldPoseAction::ResolveFieldPoseAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node,
  std::shared_ptr<SemanticNavigation> semantic)
: BT::SyncActionNode(name, config), node_(std::move(node)), semantic_(std::move(semantic)) {}
BT::PortsList ResolveFieldPoseAction::providedPorts() {return {BT::InputPort<std::string>("semantic_pose"), BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")};}
BT::NodeStatus ResolveFieldPoseAction::tick()
{
  const auto id = getInput<std::string>("semantic_pose"); if (!id) return BT::NodeStatus::FAILURE;
  try {setOutput("goal_pose", semantic_->resolveFieldPose(*id, node_->now())); return BT::NodeStatus::SUCCESS;}
  catch (const std::exception & e) {RCLCPP_ERROR(node_->get_logger(), "%s", e.what()); return BT::NodeStatus::FAILURE;}
}

GetCurrentRegionAction::GetCurrentRegionAction(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<SemanticNavigation> semantic)
: BT::SyncActionNode(name, config), semantic_(std::move(semantic)) {}
BT::PortsList GetCurrentRegionAction::providedPorts() {return {BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"), BT::OutputPort<std::string>("region")};}
BT::NodeStatus GetCurrentRegionAction::tick()
{
  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose"); if (!pose) return BT::NodeStatus::FAILURE;
  const auto region = semantic_->getSemanticRegion(*pose); setOutput("region", region);
  // Region classification is information for route selection, not a safety
  // gate. Never cancel an already-running Nav2 action at a region boundary.
  return BT::NodeStatus::SUCCESS;
}

IsInRegionCondition::IsInRegionCondition(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<SemanticNavigation> semantic)
: BT::ConditionNode(name, config), semantic_(std::move(semantic)) {}
BT::PortsList IsInRegionCondition::providedPorts() {return {BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"), BT::InputPort<std::string>("region")};}
BT::NodeStatus IsInRegionCondition::tick()
{
  const auto pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose"); const auto region = getInput<std::string>("region");
  return pose && region && semantic_->isInRegion(*pose, *region) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

SelectSemanticRouteAction::SelectSemanticRouteAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node,
  std::shared_ptr<SemanticNavigation> semantic)
: BT::SyncActionNode(name, config), node_(std::move(node)), semantic_(std::move(semantic)) {}
BT::PortsList SelectSemanticRouteAction::providedPorts()
{
  return {BT::InputPort<std::string>("current_region"), BT::InputPort<std::string>("target_region"),
    BT::InputPort<std::string>("task_type"), BT::OutputPort<std::string>("route_id"),
    BT::OutputPort<std::string>("edge_list"), BT::OutputPort<std::string>("route_type"),
    BT::OutputPort<std::string>("controller"),
    BT::OutputPort<bool>("uses_tunnel"), BT::OutputPort<geometry_msgs::msg::PoseStamped>("preparation_goal"),
    BT::OutputPort<nav_msgs::msg::Path>("entry_path"), BT::OutputPort<nav_msgs::msg::Path>("tunnel_path")};
}
BT::NodeStatus SelectSemanticRouteAction::tick()
{
  const auto current = getInput<std::string>("current_region"); const auto target = getInput<std::string>("target_region");
  const auto task = getInput<std::string>("task_type"); if (!current || !target || !task) return BT::NodeStatus::FAILURE;
  const auto route = semantic_->selectRoute(*current, *target, *task, node_->now());
  setOutput("route_id", route.id); setOutput("uses_tunnel", route.uses_tunnel);
  std::string edge_list;
  for (const auto & edge : route.edge_ids) {if (!edge_list.empty()) edge_list += ","; edge_list += edge;}
  setOutput("edge_list", edge_list);
  setOutput("route_type", route.uses_tunnel ? "tunnel" : "normal");
  setOutput("controller", route.uses_tunnel ? "TunnelController" : "FollowPath");
  if (route.uses_tunnel) {setOutput("preparation_goal", route.preparation_goal); setOutput("entry_path", route.entry_path); setOutput("tunnel_path", route.tunnel_path);}
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Semantic route %s (%s -> %s, task=%s)", route.id.c_str(),
    current->c_str(), target->c_str(), task->c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList IsTunnelRouteCondition::providedPorts() {return {BT::InputPort<bool>("uses_tunnel")};}
BT::NodeStatus IsTunnelRouteCondition::tick()
{
  const auto value = getInput<bool>("uses_tunnel"); return value && *value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BuildTunnelRetreatPathAction::BuildTunnelRetreatPathAction(
  const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(std::move(node)) {}
BT::PortsList BuildTunnelRetreatPathAction::providedPorts()
{
  return {BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
    BT::InputPort<nav_msgs::msg::Path>("tunnel_path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("preparation_goal"),
    BT::OutputPort<nav_msgs::msg::Path>("retreat_path")};
}
BT::NodeStatus BuildTunnelRetreatPathAction::tick()
{
  const auto current = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  const auto line = getInput<nav_msgs::msg::Path>("tunnel_path");
  const auto prep = getInput<geometry_msgs::msg::PoseStamped>("preparation_goal");
  if (!current || !line || !prep) return BT::NodeStatus::FAILURE;
  try {setOutput("retreat_path", SemanticNavigation::buildRetreatPath(*current, *line, *prep, node_->now())); return BT::NodeStatus::SUCCESS;}
  catch (const std::exception & e) {RCLCPP_ERROR(node_->get_logger(), "Cannot build tunnel retreat: %s", e.what()); return BT::NodeStatus::FAILURE;}
}
}  // namespace rm_behavior_tree
