#include "semantic/semantic_navigation.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace rm_behavior_tree
{
namespace
{
double normalize(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

double yawOf(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

SemanticNavigation::SemanticNavigation(const std::string & yaml_path)
{
  const auto root = YAML::LoadFile(yaml_path);
  frame_id_ = root["frame_id"].as<std::string>("map");
  const auto tf = root["map_T_field"];
  transform_x_ = tf["x"].as<double>();
  transform_y_ = tf["y"].as<double>();
  transform_yaw_ = tf["yaw"].as<double>();

  for (const auto & item : root["poses"]) {
    const auto id = item.first.as<std::string>();
    const auto value = item.second;
    poses_[id] = {value["x"].as<double>(), value["y"].as<double>(), value["yaw"].as<double>(0.0)};
  }
  for (const auto & item : root["regions"]) {
    Region region;
    region.id = item.first.as<std::string>();
    const auto value = item.second;
    region.priority = value["priority"].as<int>(0);
    const auto shape = value["shape"].as<std::string>();
    if (shape == "circle") {
      region.shape = Shape::CIRCLE;
      region.center = fieldToMap(value["center"][0].as<double>(), value["center"][1].as<double>());
      region.radius = value["radius"].as<double>();
    } else if (shape == "rectangle") {
      region.shape = Shape::RECTANGLE;
      const auto a = fieldToMap(value["min"][0].as<double>(), value["min"][1].as<double>());
      const auto b = fieldToMap(value["max"][0].as<double>(), value["max"][1].as<double>());
      region.min = {std::min(a.x, b.x), std::min(a.y, b.y)};
      region.max = {std::max(a.x, b.x), std::max(a.y, b.y)};
    } else if (shape == "polygon") {
      region.shape = Shape::POLYGON;
      for (const auto & p : value["points"]) {
        region.polygon.push_back(fieldToMap(p[0].as<double>(), p[1].as<double>()));
      }
    } else {
      throw std::runtime_error("Unknown semantic region shape: " + shape);
    }
    regions_.push_back(region);
  }
  std::stable_sort(regions_.begin(), regions_.end(), [](const Region & a, const Region & b) {
    return a.priority > b.priority;
  });
  for (const auto & item : root["region_nodes"]) {
    region_nodes_[item.first.as<std::string>()] = item.second.as<std::string>();
  }
  for (const auto & item : root["edges"]) {
    Edge edge;
    edge.id = item["id"].as<std::string>();
    edge.from = item["from"].as<std::string>();
    edge.to = item["to"].as<std::string>();
    edge.type = item["type"].as<std::string>();
    edge.bidirectional = item["bidirectional"].as<bool>(false);
    edge.cost = item["cost"].as<double>(1.0);
    edge.preparation_from = item["preparation_from"].as<std::string>("");
    edge.preparation_to = item["preparation_to"].as<std::string>("");
    if (item["approach_from"]) for (const auto & p : item["approach_from"]) edge.approach_from.push_back(p.as<std::string>());
    if (item["approach_to"]) for (const auto & p : item["approach_to"]) edge.approach_to.push_back(p.as<std::string>());
    if (item["centerline"]) for (const auto & p : item["centerline"]) edge.centerline.push_back(p.as<std::string>());
    edge.exit_from = item["exit_from"].as<std::string>("");
    edge.exit_to = item["exit_to"].as<std::string>("");
    edges_.push_back(edge);
  }
}

SemanticNavigation::Point SemanticNavigation::fieldToMap(double x, double y) const
{
  const double c = std::cos(transform_yaw_);
  const double s = std::sin(transform_yaw_);
  return {transform_x_ + c * x - s * y, transform_y_ + s * x + c * y};
}

SemanticNavigation::PoseDef SemanticNavigation::fieldPoseToMap(const PoseDef & pose) const
{
  const auto p = fieldToMap(pose.x, pose.y);
  return {p.x, p.y, normalize(pose.yaw + transform_yaw_)};
}

geometry_msgs::msg::PoseStamped SemanticNavigation::makePose(
  const PoseDef & pose, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame_id_;
  result.header.stamp = stamp;
  result.pose.position.x = pose.x;
  result.pose.position.y = pose.y;
  result.pose.orientation.z = std::sin(pose.yaw * 0.5);
  result.pose.orientation.w = std::cos(pose.yaw * 0.5);
  return result;
}

geometry_msgs::msg::PoseStamped SemanticNavigation::resolveFieldPose(
  const std::string & id, const rclcpp::Time & stamp) const
{
  const auto it = poses_.find(id);
  if (it == poses_.end()) throw std::out_of_range("Unknown semantic pose: " + id);
  return makePose(fieldPoseToMap(it->second), stamp);
}

bool SemanticNavigation::isInRegion(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & id) const
{
  if (pose.header.frame_id != frame_id_) return false;
  const auto it = std::find_if(regions_.begin(), regions_.end(), [&](const Region & r) {return r.id == id;});
  if (it == regions_.end()) return false;
  const Point p{pose.pose.position.x, pose.pose.position.y};
  if (it->shape == Shape::CIRCLE) {
    return std::hypot(p.x - it->center.x, p.y - it->center.y) <= it->radius;
  }
  if (it->shape == Shape::RECTANGLE) {
    return p.x >= it->min.x && p.x <= it->max.x && p.y >= it->min.y && p.y <= it->max.y;
  }
  bool inside = false;
  for (size_t i = 0, j = it->polygon.size() - 1; i < it->polygon.size(); j = i++) {
    const auto & a = it->polygon[i]; const auto & b = it->polygon[j];
    if (((a.y > p.y) != (b.y > p.y)) &&
      p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

std::string SemanticNavigation::getSemanticRegion(const geometry_msgs::msg::PoseStamped & pose) const
{
  for (const auto & region : regions_) if (isInRegion(pose, region.id)) return region.id;
  return "unknown";
}

std::string SemanticNavigation::nodeForRegion(const std::string & region) const
{
  const auto it = region_nodes_.find(region);
  return it == region_nodes_.end() ? "" : it->second;
}

nav_msgs::msg::Path SemanticNavigation::makePath(
  const std::vector<std::string> & ids, bool reverse, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path; path.header.frame_id = frame_id_; path.header.stamp = stamp;
  auto append = [&](const std::string & id) {path.poses.push_back(resolveFieldPose(id, stamp));};
  if (reverse) for (auto it = ids.rbegin(); it != ids.rend(); ++it) append(*it);
  else for (const auto & id : ids) append(id);
  return path;
}

SemanticRoute SemanticNavigation::selectRoute(
  const std::string & current_region, const std::string & target_region,
  const std::string & task_type, const rclcpp::Time & stamp) const
{
  SemanticRoute result;
  const auto start = nodeForRegion(current_region);
  const auto goal = nodeForRegion(target_region);
  if (start.empty() || goal.empty() || start == goal || task_type == "patrol") {
    result.id = "normal_direct";
    return result;
  }
  const bool tunnel_preferred =
    (target_region == "center_zone" && (current_region == "red_supply_zone" || current_region == "own_side")) ||
    (target_region == "red_supply_zone" && (current_region == "center_zone" || current_region == "center_side"));
  struct Step {double cost; std::string node; std::vector<std::pair<size_t, bool>> path;};
  auto greater = [](const Step & a, const Step & b) {return a.cost > b.cost;};
  std::priority_queue<Step, std::vector<Step>, decltype(greater)> queue(greater);
  std::map<std::string, double> best; queue.push({0.0, start, {}}); best[start] = 0.0;
  std::vector<std::pair<size_t, bool>> selected;
  while (!queue.empty()) {
    auto step = queue.top(); queue.pop();
    if (step.node == goal) {selected = step.path; break;}
    if (step.cost > best[step.node]) continue;
    for (size_t i = 0; i < edges_.size(); ++i) {
      const auto & edge = edges_[i];
      for (const bool reverse : {false, true}) {
        if (reverse && !edge.bidirectional) continue;
        const auto & from = reverse ? edge.to : edge.from;
        const auto & to = reverse ? edge.from : edge.to;
        if (from != step.node) continue;
        double edge_cost = edge.cost;
        if (tunnel_preferred && edge.type == "normal" &&
          ((from == "red_supply" && to == "center") || (from == "center" && to == "red_supply"))) edge_cost += 1000.0;
        const double next_cost = step.cost + edge_cost;
        if (!best.count(to) || next_cost < best[to]) {
          best[to] = next_cost; auto path = step.path; path.push_back({i, reverse});
          queue.push({next_cost, to, std::move(path)});
        }
      }
    }
  }
  if (selected.empty()) {result.id = "normal_direct"; return result;}
  result.id = start + "_to_" + goal;
  for (const auto & selected_edge : selected) {
    const auto & edge = edges_[selected_edge.first]; const bool reverse = selected_edge.second;
    result.edge_ids.push_back(edge.id + (reverse ? ":reverse" : ""));
    if (edge.type == "tunnel") {
      result.uses_tunnel = true;
      result.preparation_goal = resolveFieldPose(reverse ? edge.preparation_to : edge.preparation_from, stamp);
      result.entry_path = makePath(reverse ? edge.approach_to : edge.approach_from, false, stamp);
      result.tunnel_path = makePath(edge.centerline, reverse, stamp);
      const auto & exit_id = reverse ? edge.exit_from : edge.exit_to;
      if (!exit_id.empty()) {
        auto exit_pose = resolveFieldPose(exit_id, stamp);
        // Keep the validated tunnel heading through the short mouth-clearance
        // segment. The normal controller only takes over once fully outside.
        exit_pose.pose.orientation = result.tunnel_path.poses.back().pose.orientation;
        result.tunnel_path.poses.push_back(exit_pose);
      }
      if (reverse) {
        const double yaw = yawOf(result.tunnel_path.poses.front());
        for (auto & pose : result.tunnel_path.poses) {
          pose.pose.orientation.z = std::sin(normalize(yaw + M_PI) * 0.5);
          pose.pose.orientation.w = std::cos(normalize(yaw + M_PI) * 0.5);
        }
      }
    }
  }
  return result;
}

nav_msgs::msg::Path SemanticNavigation::buildRetreatPath(
  const geometry_msgs::msg::PoseStamped & current, const nav_msgs::msg::Path & line,
  const geometry_msgs::msg::PoseStamped & preparation, const rclcpp::Time & stamp)
{
  if (line.poses.size() < 2 || current.header.frame_id != line.header.frame_id) {
    throw std::invalid_argument("Retreat requires a map-frame centerline with at least two poses");
  }
  double best_distance = std::numeric_limits<double>::infinity(); size_t best_segment = 0; double best_t = 0.0;
  for (size_t i = 0; i + 1 < line.poses.size(); ++i) {
    const auto & a = line.poses[i].pose.position; const auto & b = line.poses[i + 1].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    const double t = length2 == 0.0 ? 0.0 : std::clamp(
      ((current.pose.position.x - a.x) * dx + (current.pose.position.y - a.y) * dy) / length2, 0.0, 1.0);
    const double px = a.x + t * dx, py = a.y + t * dy;
    const double distance = std::hypot(current.pose.position.x - px, current.pose.position.y - py);
    if (distance < best_distance) {best_distance = distance; best_segment = i; best_t = t;}
  }
  nav_msgs::msg::Path path; path.header.frame_id = line.header.frame_id; path.header.stamp = stamp;
  auto projected = line.poses[best_segment];
  const auto & a = line.poses[best_segment].pose.position; const auto & b = line.poses[best_segment + 1].pose.position;
  projected.header = path.header; projected.pose.position.x = a.x + best_t * (b.x - a.x); projected.pose.position.y = a.y + best_t * (b.y - a.y);
  path.poses.push_back(projected);
  for (size_t i = best_segment + 1; i > 0; --i) {auto pose = line.poses[i - 1]; pose.header = path.header; path.poses.push_back(pose);}
  auto prep = preparation; prep.header = path.header; prep.pose.orientation = line.poses.front().pose.orientation; path.poses.push_back(prep);
  return path;
}

}  // namespace rm_behavior_tree
