#include "action/get_current_location.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace rm_behavior_tree
{

GetCurrentLocationAction::GetCurrentLocationAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("get_current_location");
  if (!node_) {
    throw std::runtime_error("Failed to create node 'get_current_location'");
  }
  // Inherit use_sim_time so TF lookups work in simulation
  if (!node_->has_parameter("use_sim_time")) {
    node_->declare_parameter("use_sim_time", true);
  } else {
    node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  auto clock = node_->get_clock();
  tf2::Duration buffer_duration(tf2::durationFromSec(10.0));  // 10 seconds buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock, buffer_duration, node_);
  if (!tf_buffer_) {
    throw std::runtime_error("Failed to create tf2_ros::Buffer");
  }

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (!tf_listener_) {
    throw std::runtime_error("Failed to create tf2_ros::TransformListener");
  }
}

BT::NodeStatus GetCurrentLocationAction::tick()
{
  geometry_msgs::msg::TransformStamped t;
  const auto target_frame_input = getInput<std::string>("target_frame");
  const auto base_frame_input = getInput<std::string>("base_frame");
  const std::string target_frame = target_frame_input ? target_frame_input.value() : "map";
  const std::string base_frame = base_frame_input ? base_frame_input.value() : "base_link";

  try {
    // Use tf2::TimePointZero to request the latest available transform
    // In ROS2, TimePointZero means "latest" (equivalent to ROS1's Time(0))
    t = tf_buffer_->lookupTransform(
      target_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
    setOutput("current_location", t);

    RCLCPP_DEBUG(
      logger_,
      "Current Location:"
      "\nTranslation:"
      "\nx: %f"
      "\ny: %f"
      "\nz: %f"
      "\nRotation:"
      "\nx: %f"
      "\ny: %f"
      "\nz: %f"
      "\nw: %f",
      t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
      t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
      t.transform.rotation.w);

    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_, "Failed to transform %s to %s: %s",
      base_frame.c_str(), target_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::GetCurrentLocationAction>("GetCurrentLocation");
}
