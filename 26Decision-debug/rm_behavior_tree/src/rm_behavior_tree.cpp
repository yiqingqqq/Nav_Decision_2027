#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

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

  std::cout << "Start RM_Behavior_Tree" << '\n';
  RCLCPP_INFO(node->get_logger(), "Load bt_xml: \e[1;42m %s \e[0m", bt_xml_path.c_str());

  auto update_msg_node = std::make_shared<rclcpp::Node>("update_msg");
  BT::RosNodeParams params_update_msg;
  params_update_msg.nh = update_msg_node;

  auto send_goal_node = std::make_shared<rclcpp::Node>("send_goal");
  BT::RosNodeParams params_send_goal;
  params_send_goal.nh = send_goal_node;
  params_send_goal.default_port_value = "goal_pose";

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
    "is_on_own_side",
    "select_tunnel_direction",
  };
  // clang-format on

  for (const auto & p : msg_update_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_update_msg);
  }

  for (const auto & p : bt_plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);

  RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_spin"), params_send_spin);

  RegisterRosNode(
    factory, BT::SharedLibrary::getOSName("select_controller"), params_select_controller);

  auto tree = factory.createTreeFromFile(bt_xml_path);

  // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  while (rclcpp::ok()) {
    tree.tickWhileRunning(std::chrono::milliseconds(10));
  }

  rclcpp::shutdown();
  return 0;
}
