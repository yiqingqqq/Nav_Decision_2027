#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "teb_local_planner/optimal_planner.h"

using namespace teb_local_planner;

class HeadingRegression : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("teb_heading_regression");
    cfg.robot.max_vel_x = 1.4;
    cfg.robot.max_vel_x_backwards = 0.7;
    cfg.robot.max_vel_y = 0.8;
    cfg.robot.acc_lim_y = 0.9;
    cfg.optim.weight_kinematics_nh = 0.0;
    cfg.optim.weight_kinematics_forward_drive = 0.0;
    cfg.optim.no_inner_iterations = 20;
    cfg.optim.no_outer_iterations = 4;
    // Deliberately conflict with heading mode: heading must take priority.
    cfg.trajectory.global_plan_overwrite_orientation = true;
    cfg.trajectory.allow_init_with_backwards_motion = true;
  }

  void checkTrajectory(double dx, double dy, double heading)
  {
    std::vector<geometry_msgs::msg::PoseStamped> path(11);
    for (size_t i = 0; i < path.size(); ++i) {
      auto & pose = path[i].pose;
      const double t = static_cast<double>(i) / (path.size() - 1);
      pose.position.x = dx * t;
      pose.position.y = dy * t;
      pose.orientation.z = std::sin(heading * 0.5);
      pose.orientation.w = std::cos(heading * 0.5);
    }
    TebOptimalPlanner planner;
    ObstContainer obstacles;
    planner.initialize(node, cfg, &obstacles);
    ASSERT_TRUE(planner.plan(path));
    double max_error = 0.0;
    for (int i = 0; i < planner.teb().sizePoses(); ++i) {
      const double error = std::abs(g2o::normalize_theta(planner.teb().Pose(i).theta() - heading));
      max_error = std::max(max_error, error);
    }
    EXPECT_LT(max_error, 0.10);
    std::cout << "heading=" << heading << " move=(" << dx << "," << dy
              << ") max_yaw_error=" << max_error << " rad\n";
    double vx, vy, omega;
    ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 1));
    EXPECT_TRUE(std::isfinite(vx) && std::isfinite(vy) && std::isfinite(omega));
    EXPECT_LT(std::abs(omega), 0.10);
  }

  nav2_util::LifecycleNode::SharedPtr node;
  TebConfig cfg;
};

TEST_F(HeadingRegression, SidewaysMotionKeepsHeading) {checkTrajectory(0.0, 2.0, 0.0);}
TEST_F(HeadingRegression, BackwardMotionKeepsHeading) {checkTrajectory(-2.0, 0.0, 0.0);}
TEST_F(HeadingRegression, ExplicitTunnelHeadingIsPreserved) {checkTrajectory(-2.0, 0.0, M_PI);}

TEST_F(HeadingRegression, ReturnEntryWithLargeInitialHeadingError)
{
  cfg.robot.max_vel_x = 1.2;
  cfg.robot.max_vel_x_backwards = 0.3;
  cfg.robot.max_vel_theta = 0.9;
  cfg.robot.acc_lim_x = 1.2;
  cfg.robot.acc_lim_y = cfg.robot.acc_lim_theta = 0.9;
  cfg.optim.no_inner_iterations = 5;
  cfg.optim.no_outer_iterations = 4;
  cfg.optim.weight_viapoint = 20.0;
  cfg.optim.weight_shortest_path = 0.2;
  cfg.optim.weight_optimaltime = 1.0;
  cfg.trajectory.dt_ref = 0.15;
  cfg.trajectory.dt_hysteresis = 0.015;
  const double points[4][2] = {{4.922, 0.193}, {4.78, 0.24}, {4.66, 0.31}, {4.50, 0.40}};
  std::vector<geometry_msgs::msg::PoseStamped> path(4);
  for (size_t i = 0; i < path.size(); ++i) {
    path[i].pose.position.x = points[i][0];
    path[i].pose.position.y = points[i][1];
    const double yaw = i == 0 ? -0.656 : -M_PI;
    path[i].pose.orientation.z = std::sin(yaw / 2.0);
    path[i].pose.orientation.w = std::cos(yaw / 2.0);
  }
  ObstContainer obstacles;
  ViaPointContainer via_points{{4.66, 0.31}};
  TebOptimalPlanner planner;
  planner.initialize(node, cfg, &obstacles, {}, &via_points);
  ASSERT_TRUE(planner.plan(path));
  double vx, vy, omega;
  ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 1));
  std::cout << "return_entry vx=" << vx << " vy=" << vy << " omega=" << omega
            << " duration=" << planner.teb().getSumOfAllTimeDiffs() << " s\n";
  EXPECT_LT(omega, -0.01);
}

TEST_F(HeadingRegression, ParameterDefaultsAndValidation)
{
  EXPECT_DOUBLE_EQ(cfg.trajectory.goal_heading_weight, 100.0);
  cfg.declareParameters(node, "FollowPath");
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.goal_heading_weight").as_double(), 100.0);
  for (const double value : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(cfg.dynamicParametersCallback({
      rclcpp::Parameter("FollowPath.goal_heading_weight", value)}).successful);
    EXPECT_DOUBLE_EQ(cfg.trajectory.goal_heading_weight, 100.0);
  }
  EXPECT_TRUE(cfg.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.goal_heading_weight", 0.0)}).successful);
  EXPECT_DOUBLE_EQ(cfg.trajectory.goal_heading_weight, 0.0);
}
