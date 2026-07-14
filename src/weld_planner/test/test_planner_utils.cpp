#include "weld_planner/planner_utils.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace {

geometry_msgs::msg::Point Point(double x, double y, double z) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST(PlannerUtils, ComputesPathLength) {
  const std::vector<geometry_msgs::msg::Point> points{Point(0.0, 0.0, 0.0),
                                                      Point(3.0, 4.0, 0.0)};
  EXPECT_NEAR(weld_planner::PathLength(points), 5.0, 1e-9);
}

TEST(PlannerUtils, RejectsInsufficientPoints) {
  const std::vector<geometry_msgs::msg::Point> points{Point(0.0, 0.0, 0.0)};
  EXPECT_EQ(weld_planner::ValidatePath(points, 0.2),
            weld_planner::PlanFault::kInsufficientPoints);
}

TEST(PlannerUtils, RejectsExcessiveGap) {
  const std::vector<geometry_msgs::msg::Point> points{Point(0.0, 0.0, 0.0),
                                                      Point(1.0, 0.0, 0.0)};
  EXPECT_EQ(weld_planner::ValidatePath(points, 0.2), weld_planner::PlanFault::kExcessiveGap);
}

TEST(PlannerUtils, ResamplesPathAtFixedSpacing) {
  const std::vector<geometry_msgs::msg::Point> points{Point(0.0, 0.0, 0.0),
                                                      Point(0.3, 0.0, 0.0)};
  const auto samples = weld_planner::ResamplePath(points, 0.1);

  ASSERT_GE(samples.size(), 3U);
  EXPECT_NEAR(samples[1].x, 0.1, 1e-9);
  EXPECT_NEAR(samples[2].x, 0.2, 1e-9);
}

}  // namespace

