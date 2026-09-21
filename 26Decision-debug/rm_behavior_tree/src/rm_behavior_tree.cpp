#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"
#include "semantic/semantic_bt_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;

  std::string bt_xml_path;
  auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");
  node->declare_parameter<std::string>(
    "style", "./rm_decision_ws/rm_behavior_tree/rm_behavior_tree.xml");
  node->get_parameter_or<std::string>(
    "style", bt_xml_path, "./rm_decision_ws/rm_behavior_tree/config/attack_left.xml");
  const auto default_semantic_config =
    ament_index_cpp::get_package_share_directory("rm_behavior_tree") +
    "/config/RMUL2027_semantic_map.yaml";
  const auto semantic_config = node->declare_parameter<std::string>(
    "semantic_config", default_semantic_config);
  auto semantic_navigation =
    std::make_shared<rm_behavior_tree::SemanticNavigation>(semantic_config);

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  auto update_msg_node = std::make_shared<rclcpp::Node>("update_msg");
  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = update_msg_node;

  auto send_goal_node = std::make_shared<rclcpp::Node>("send_goal");
  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = send_goal_node;
  params_send_goal.default_port_value = "goal_pose";
  params_send_goal.wait_for_server_timeout = std::chrono::milliseconds(5000);

  auto follow_tunnel_path_node = std::make_shared<rclcpp::Node>("follow_tunnel_path");
  BT::RosNodeParams params_follow_tunnel_path;
  params_follow_tunnel_path.nh = follow_tunnel_path_node;
  params_follow_tunnel_path.default_port_value = "/follow_path";
  params_follow_tunnel_path.wait_for_server_timeout = std::chrono::milliseconds(5000);

  auto send_spin_node = std::make_shared<rclcpp::Node>("send_spin");
  BT::RosNodeParams params_send_spin;
  params_send_spin.nh = send_spin_node;
  params_send_spin.default_port_value = "/spin_velocity";

  auto select_controller_node = std::make_shared<rclcpp::Node>("select_controller");
  BT::RosNodeParams params_select_controller;
  params_select_controller.nh = select_controller_node;
  params_select_controller.default_port_value = "/controller_selector";
  params_select_controller.publisher_qos =
    rclcpp::QoS(1).transient_local().reliable();

  // clang-format off
  const std::vector<std::string> msg_update_plugin_libs = {
    "sub_game_status",
    "sub_robot_status",
    "sub_robot_id",
    "sub_deduction_reason",
  };

  const std::vector<std::string> bt_plugin_libs = {
    "rate_controller",
    "non_blocking_rate_controller",
    "run_once_until_halted",
    "is_game_time",
    "is_retreat",
    "is_near_goal",
    "get_current_location",
    "is_HP_deduction",
    "is_HP_level",
    "is_robot_id",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  factory.registerNodeType<rm_behavior_tree::GetCurrentPoseAction>(
    "GetCurrentPose", node);
  factory.registerNodeType<rm_behavior_tree::ResolveFieldPoseAction>(
    "ResolveFieldPose", node, semantic_navigation);
  factory.registerNodeType<rm_behavior_tree::GetCurrentRegionAction>(
    "GetCurrentRegion", semantic_navigation);
  factory.registerNodeType<rm_behavior_tree::IsInRegionCondition>(
    "IsInRegion", semantic_navigation);
  factory.registerNodeType<rm_behavior_tree::SelectSemanticRouteAction>(
    "SelectSemanticRoute", node, semantic_navigation);
  factory.registerNodeType<rm_behavior_tree::IsTunnelRouteCondition>(
    "IsTunnelRoute");
  factory.registerNodeType<rm_behavior_tree::BuildTunnelRetreatPathAction>(
    "BuildTunnelRetreatPath", node);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(
    factory, BT::SharedLibrary::getOSName("follow_tunnel_path"), params_follow_tunnel_path);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_spin"), params_send_spin);

  RegisterRosNode(
    factory, BT::SharedLibrary::getOSName("select_controller"), params_select_controller);

  auto tree = factory.createTreeFromFile(bt_xml_path);

  // Groot is diagnostic-only: a stale monitor or behavior-tree process must
  // never abort the robot decision process.
  const bool enable_groot = node->declare_parameter<bool>("enable_groot", true);
  const unsigned groot_port = static_cast<unsigned>(
    node->declare_parameter<int>("groot_port", 1667));
  std::unique_ptr<BT::Groot2Publisher> groot_publisher;
  if (enable_groot) {
    try {
      groot_publisher = std::make_unique<BT::Groot2Publisher>(tree, groot_port);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Groot2 publisher could not bind port %u: %s. Continuing without Groot monitoring.",
        groot_port, e.what());
    }
  }

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
