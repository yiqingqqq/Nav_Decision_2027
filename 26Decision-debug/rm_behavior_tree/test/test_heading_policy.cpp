#include <gtest/gtest.h>
#include <limits>
#include "action/heading_policy.hpp"

using geometry_msgs::msg::PoseStamped;
using rm_behavior_tree::applyHeadingPolicy;

TEST(HeadingPolicy, HoldIgnoresDefaultGoalYaw)
{
  PoseStamped goal, current;
  goal.header.frame_id = current.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  current.pose.orientation.z = std::sin(1.2 / 2.0);
  current.pose.orientation.w = std::cos(1.2 / 2.0);
  applyHeadingPolicy(goal, "hold", current);
  EXPECT_NEAR(goal.pose.orientation.z, current.pose.orientation.z, 1e-12);
  EXPECT_NEAR(goal.pose.orientation.w, current.pose.orientation.w, 1e-12);
  current.pose.orientation.w = 0.0;
  EXPECT_NEAR(2.0 * std::atan2(goal.pose.orientation.z, goal.pose.orientation.w), 1.2, 1e-12);
}

TEST(HeadingPolicy, ExplicitZeroIsAnActualHeading)
{
  PoseStamped goal;
  goal.pose.orientation.w = 2.0;
  applyHeadingPolicy(goal, "explicit");
  EXPECT_DOUBLE_EQ(goal.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(goal.pose.orientation.w, 1.0);
}

TEST(HeadingPolicy, ExplicitReverseHeadingIsPreserved)
{
  PoseStamped goal;
  goal.pose.orientation.z = -1.0;
  goal.pose.orientation.w = 0.0;
  applyHeadingPolicy(goal, "explicit");
  EXPECT_NEAR(std::abs(goal.pose.orientation.z), 1.0, 1e-12);
  EXPECT_NEAR(goal.pose.orientation.w, 0.0, 1e-12);
}

TEST(HeadingPolicy, RejectsMissingOrWrongFramePose)
{
  PoseStamped goal, current;
  goal.header.frame_id = "map";
  current.header.frame_id = "odom";
  EXPECT_THROW(applyHeadingPolicy(goal, "hold"), std::invalid_argument);
  EXPECT_THROW(applyHeadingPolicy(goal, "hold", current), std::invalid_argument);
}

TEST(HeadingPolicy, RejectsInvalidPolicyAndQuaternion)
{
  PoseStamped goal;
  goal.pose.orientation.w = 0.0;
  EXPECT_THROW(applyHeadingPolicy(goal, "default"), std::invalid_argument);
  EXPECT_THROW(applyHeadingPolicy(goal, "explicit"), std::invalid_argument);
  goal.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(applyHeadingPolicy(goal, "explicit"), std::invalid_argument);
}
