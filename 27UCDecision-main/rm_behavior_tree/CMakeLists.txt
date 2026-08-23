cmake_minimum_required(VERSION 3.8)
project(rm_behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2_ros
  behaviortree_cpp
  behaviortree_ros2
)

function(configure_bt_target target)
  target_include_directories(
    ${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins/include>
  )
  ament_target_dependencies(${target} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

add_executable(rm_behavior_tree
  src/rm_behavior_tree.cpp
  plugins/action/load_strategy_preset.cpp
  plugins/action/next_patrol_waypoint.cpp
  plugins/action/resolve_field_pose.cpp
)
configure_bt_target(rm_behavior_tree)

set(BT_PLUGIN_TARGETS
  send_goal
  sub_game_status
  sub_robot_status
  sub_deduction_reason
  send_spin
  get_current_location
  rate_controller
  is_game_time
  is_retreat
  is_near_goal
  is_HP_deduction
  is_HP_level
)

add_library(send_goal SHARED plugins/action/send_goal.cpp)
add_library(sub_game_status SHARED plugins/action/sub_game_status.cpp)
add_library(sub_robot_status SHARED plugins/action/sub_robot_status.cpp)
add_library(sub_deduction_reason SHARED plugins/action/sub_deduction_reason.cpp)
add_library(send_spin SHARED plugins/action/send_spin.cpp)
add_library(get_current_location SHARED plugins/action/get_current_location.cpp)
add_library(rate_controller SHARED plugins/decorator/rate_controller.cpp)
add_library(is_game_time SHARED plugins/condition/is_game_time.cpp)
add_library(is_retreat SHARED plugins/condition/is_retreat.cpp)
add_library(is_near_goal SHARED plugins/condition/is_near_goal.cpp)
add_library(is_HP_deduction SHARED plugins/condition/is_HP_deduction.cpp)
add_library(is_HP_level SHARED plugins/condition/is_HP_level.cpp)

foreach(target IN LISTS BT_PLUGIN_TARGETS)
  configure_bt_target(${target})
endforeach()

install(
  TARGETS rm_behavior_tree
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS ${BT_PLUGIN_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
