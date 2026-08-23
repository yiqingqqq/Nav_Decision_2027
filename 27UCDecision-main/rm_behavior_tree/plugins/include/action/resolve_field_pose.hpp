#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__RESOLVE_FIELD_POSE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__RESOLVE_FIELD_POSE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class ResolveFieldPoseAction : public BT::SyncActionNode
{
public:
  ResolveFieldPoseAction(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("waypoint"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
      BT::OutputPort<std::string>("frame_id")};
  }

  BT::NodeStatus tick() override;

private:
  template<typename T>
  T parameterOr(const std::string & name, const T & fallback) const
  {
    T value = fallback;
    node_->get_parameter_or(name, value, fallback);
    return value;
  }

  rclcpp::Node::SharedPtr node_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__RESOLVE_FIELD_POSE_HPP_
