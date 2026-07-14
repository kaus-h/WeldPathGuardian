#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace seam_perception {

inline bool IsFinitePoint(const geometry_msgs::msg::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

inline double Distance(const geometry_msgs::msg::Point& lhs, const geometry_msgs::msg::Point& rhs) {
  const auto dx = lhs.x - rhs.x;
  const auto dy = lhs.y - rhs.y;
  const auto dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline std::vector<geometry_msgs::msg::Point> SmoothPath(
    const std::vector<geometry_msgs::msg::Point>& points, std::size_t window_radius) {
  if (points.empty() || window_radius == 0) {
    return points;
  }

  std::vector<geometry_msgs::msg::Point> smoothed;
  smoothed.reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto begin = i > window_radius ? i - window_radius : 0;
    const auto end = std::min(points.size() - 1, i + window_radius);
    geometry_msgs::msg::Point point;
    double count = 0.0;
    for (auto j = begin; j <= end; ++j) {
      point.x += points[j].x;
      point.y += points[j].y;
      point.z += points[j].z;
      count += 1.0;
    }
    point.x /= count;
    point.y /= count;
    point.z /= count;
    smoothed.push_back(point);
  }

  return smoothed;
}

inline double MeanConfidence(const std::vector<float>& confidences) {
  if (confidences.empty()) {
    return 0.0;
  }
  const auto sum = std::accumulate(confidences.begin(), confidences.end(), 0.0);
  return sum / static_cast<double>(confidences.size());
}

inline bool HasExcessiveNeighborJump(const std::vector<geometry_msgs::msg::Point>& points,
                                     double max_neighbor_distance) {
  if (points.size() < 2) {
    return false;
  }
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (Distance(points[i - 1], points[i]) > max_neighbor_distance) {
      return true;
    }
  }
  return false;
}

}  // namespace seam_perception
