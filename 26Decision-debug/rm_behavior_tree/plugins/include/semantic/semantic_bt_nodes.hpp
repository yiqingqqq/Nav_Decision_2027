#ifndef RM_BEHAVIOR_TREE__SEMANTIC_BT_NODES_HPP_
#define RM_BEHAVIOR_TREE__SEMANTIC_BT_NODES_HPP_

#include "semantic/semantic_navigation.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

namespace rm_behavior_tree
{
class GetCurrentPoseAction : public BT::SyncActionNode
{
public:
  GetCurrentPoseAction(const std::string &, const BT::NodeConfig &, rclcpp::Node::SharedPtr);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::optional<geometry_msgs::msg::PoseStamped> last_valid_pose_;
};

class ResolveFieldPoseAction : public BT::SyncActionNode
{
public:
  ResolveFieldPoseAction(const std::string &, const BT::NodeConfig &, rclcpp::Node::SharedPtr, std::shared_ptr<SemanticNavigation>);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_; std::shared_ptr<SemanticNavigation> semantic_;
};

class GetCurrentRegionAction : public BT::SyncActionNode
{
public:
  GetCurrentRegionAction(const std::string &, const BT::NodeConfig &, std::shared_ptr<SemanticNavigation>);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private: std::shared_ptr<SemanticNavigation> semantic_;
};

class IsInRegionCondition : public BT::ConditionNode
{
public:
  IsInRegionCondition(const std::string &, const BT::NodeConfig &, std::shared_ptr<SemanticNavigation>);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private: std::shared_ptr<SemanticNavigation> semantic_;
};

class SelectSemanticRouteAction : public BT::SyncActionNode
{
public:
  SelectSemanticRouteAction(const std::string &, const BT::NodeConfig &, rclcpp::Node::SharedPtr, std::shared_ptr<SemanticNavigation>);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private: rclcpp::Node::SharedPtr node_; std::shared_ptr<SemanticNavigation> semantic_;
};

class IsTunnelRouteCondition : public BT::ConditionNode
{
public: using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class BuildTunnelRetreatPathAction : public BT::SyncActionNode
{
public:
  BuildTunnelRetreatPathAction(const std::string &, const BT::NodeConfig &, rclcpp::Node::SharedPtr);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private: rclcpp::Node::SharedPtr node_;
};
}  // namespace rm_behavior_tree
#endif
