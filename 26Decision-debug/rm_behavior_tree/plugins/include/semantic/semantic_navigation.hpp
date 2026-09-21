#ifndef RM_BEHAVIOR_TREE__SEMANTIC_NAVIGATION_HPP_
#define RM_BEHAVIOR_TREE__SEMANTIC_NAVIGATION_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>

#include <map>
#include <string>
#include <vector>

namespace rm_behavior_tree
{

struct SemanticRoute
{
  std::string id;
  std::vector<std::string> edge_ids;
  bool uses_tunnel{false};
  geometry_msgs::msg::PoseStamped preparation_goal;
  nav_msgs::msg::Path entry_path;
  nav_msgs::msg::Path tunnel_path;
  geometry_msgs::msg::PoseStamped final_goal;
};

class SemanticNavigation
{
public:
  explicit SemanticNavigation(const std::string & yaml_path);

  geometry_msgs::msg::PoseStamped resolveFieldPose(
    const std::string & id, const rclcpp::Time & stamp) const;
  std::string getSemanticRegion(const geometry_msgs::msg::PoseStamped & map_pose) const;
  bool isInRegion(
    const geometry_msgs::msg::PoseStamped & map_pose, const std::string & region_id) const;
  SemanticRoute selectRoute(
    const std::string & current_region, const std::string & target_region,
    const std::string & task_type, const rclcpp::Time & stamp) const;

  static nav_msgs::msg::Path buildRetreatPath(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & ordered_centerline,
    const geometry_msgs::msg::PoseStamped & preparation_goal,
    const rclcpp::Time & stamp);

  const std::string & frameId() const {return frame_id_;}

private:
  struct Point {double x{0.0}; double y{0.0};};
  struct PoseDef {double x{0.0}; double y{0.0}; double yaw{0.0};};
  enum class Shape {CIRCLE, RECTANGLE, POLYGON};
  struct Region
  {
    std::string id;
    Shape shape;
    int priority{0};
    Point center;
    double radius{0.0};
    Point min;
    Point max;
    std::vector<Point> polygon;
  };
  struct Edge
  {
    std::string id;
    std::string from;
    std::string to;
    std::string type;
    bool bidirectional{false};
    double cost{1.0};
    std::string preparation_from;
    std::string preparation_to;
    std::vector<std::string> approach_from;
    std::vector<std::string> approach_to;
    std::vector<std::string> centerline;
    std::string exit_from;
    std::string exit_to;
  };

  Point fieldToMap(double x, double y) const;
  PoseDef fieldPoseToMap(const PoseDef & pose) const;
  geometry_msgs::msg::PoseStamped makePose(const PoseDef & pose, const rclcpp::Time & stamp) const;
  nav_msgs::msg::Path makePath(
    const std::vector<std::string> & pose_ids, bool reverse, const rclcpp::Time & stamp) const;
  std::string nodeForRegion(const std::string & region) const;

  std::string frame_id_{"map"};
  double transform_x_{0.0};
  double transform_y_{0.0};
  double transform_yaw_{0.0};
  std::map<std::string, PoseDef> poses_;
  std::vector<Region> regions_;
  std::map<std::string, std::string> region_nodes_;
  std::vector<Edge> edges_;
};

}  // namespace rm_behavior_tree

#endif
