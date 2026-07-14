#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "seam_perception/perception_utils.hpp"

namespace {

geometry_msgs::msg::Point Point(double x, double y, double z) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST(PerceptionUtils, RejectsNonFinitePoints) {
  EXPECT_TRUE(seam_perception::IsFinitePoint(Point(1.0, 2.0, 3.0)));
  EXPECT_FALSE(
      seam_perception::IsFinitePoint(Point(std::numeric_limits<double>::quiet_NaN(), 2.0, 3.0)));
  EXPECT_FALSE(
      seam_perception::IsFinitePoint(Point(1.0, std::numeric_limits<double>::infinity(), 3.0)));
}

TEST(PerceptionUtils, SmoothsNeighboringPoints) {
  const std::vector<geometry_msgs::msg::Point> points{Point(0.0, 0.0, 0.0), Point(1.0, 3.0, 0.0),
                                                      Point(2.0, 0.0, 0.0)};

  const auto smoothed = seam_perception::SmoothPath(points, 1);

  ASSERT_EQ(smoothed.size(), 3U);
  EXPECT_NEAR(smoothed[1].x, 1.0, 1e-9);
  EXPECT_NEAR(smoothed[1].y, 1.0, 1e-9);
}

TEST(PerceptionUtils, DetectsNeighborJump) {
  const std::vector<geometry_msgs::msg::Point> points{Point(0.0, 0.0, 0.0), Point(0.1, 0.0, 0.0),
                                                      Point(1.0, 0.0, 0.0)};

  EXPECT_TRUE(seam_perception::HasExcessiveNeighborJump(points, 0.25));
  EXPECT_FALSE(seam_perception::HasExcessiveNeighborJump(points, 1.0));
}

}  // namespace
