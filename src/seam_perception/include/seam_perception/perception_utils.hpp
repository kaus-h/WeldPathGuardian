#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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

inline double MaxNeighborDistance(const std::vector<geometry_msgs::msg::Point>& points) {
  double max_distance = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    max_distance = std::max(max_distance, Distance(points[i - 1], points[i]));
  }
  return max_distance;
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

inline geometry_msgs::msg::Point FitLineAtX(const std::vector<geometry_msgs::msg::Point>& points,
                                            std::size_t begin, std::size_t end, double x_value) {
  const auto count = static_cast<double>(end - begin + 1);
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_xz = 0.0;

  for (auto index = begin; index <= end; ++index) {
    sum_x += points[index].x;
    sum_y += points[index].y;
    sum_z += points[index].z;
    sum_xx += points[index].x * points[index].x;
    sum_xy += points[index].x * points[index].y;
    sum_xz += points[index].x * points[index].z;
  }

  geometry_msgs::msg::Point fitted;
  fitted.x = x_value;
  const auto denominator = (count * sum_xx) - (sum_x * sum_x);
  if (std::abs(denominator) <= std::numeric_limits<double>::epsilon()) {
    fitted.y = sum_y / count;
    fitted.z = sum_z / count;
    return fitted;
  }

  const auto slope_y = ((count * sum_xy) - (sum_x * sum_y)) / denominator;
  const auto intercept_y = (sum_y - (slope_y * sum_x)) / count;
  const auto slope_z = ((count * sum_xz) - (sum_x * sum_z)) / denominator;
  const auto intercept_z = (sum_z - (slope_z * sum_x)) / count;
  fitted.y = (slope_y * x_value) + intercept_y;
  fitted.z = (slope_z * x_value) + intercept_z;
  return fitted;
}

inline std::vector<geometry_msgs::msg::Point> FitLocalLeastSquaresPath(
    const std::vector<geometry_msgs::msg::Point>& points, std::size_t window_radius) {
  if (points.size() < 2) {
    return points;
  }

  std::vector<geometry_msgs::msg::Point> fitted;
  fitted.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto begin = i > window_radius ? i - window_radius : 0;
    const auto end = std::min(points.size() - 1, i + window_radius);
    fitted.push_back(FitLineAtX(points, begin, end, points[i].x));
  }
  return fitted;
}

inline double RootMeanSquareError(const std::vector<geometry_msgs::msg::Point>& reference,
                                  const std::vector<geometry_msgs::msg::Point>& fitted) {
  if (reference.empty() || reference.size() != fitted.size()) {
    return 0.0;
  }

  double squared_error = 0.0;
  for (std::size_t i = 0; i < reference.size(); ++i) {
    const auto distance = Distance(reference[i], fitted[i]);
    squared_error += distance * distance;
  }
  return std::sqrt(squared_error / static_cast<double>(reference.size()));
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
