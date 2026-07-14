#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace weld_planner {

enum class PlanFault {
  kNone,
  kInsufficientPoints,
  kStaleObservation,
  kLowConfidence,
  kExcessiveGap,
  kInvalidGeometry,
};

inline const char* ToString(PlanFault fault) {
  switch (fault) {
    case PlanFault::kNone:
      return "None";
    case PlanFault::kInsufficientPoints:
      return "InsufficientPoints";
    case PlanFault::kStaleObservation:
      return "StaleObservation";
    case PlanFault::kLowConfidence:
      return "LowConfidence";
    case PlanFault::kExcessiveGap:
      return "ExcessiveGap";
    case PlanFault::kInvalidGeometry:
      return "InvalidGeometry";
  }
  return "InvalidGeometry";
}

inline double Distance(const geometry_msgs::msg::Point& lhs, const geometry_msgs::msg::Point& rhs) {
  const auto dx = lhs.x - rhs.x;
  const auto dy = lhs.y - rhs.y;
  const auto dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double PathLength(const std::vector<geometry_msgs::msg::Point>& points) {
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += Distance(points[i - 1], points[i]);
  }
  return length;
}

inline bool HasExcessiveGap(const std::vector<geometry_msgs::msg::Point>& points,
                            double max_gap_meters) {
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (Distance(points[i - 1], points[i]) > max_gap_meters) {
      return true;
    }
  }
  return false;
}

inline geometry_msgs::msg::Point Interpolate(const geometry_msgs::msg::Point& start,
                                             const geometry_msgs::msg::Point& end, double ratio) {
  geometry_msgs::msg::Point point;
  point.x = start.x + (end.x - start.x) * ratio;
  point.y = start.y + (end.y - start.y) * ratio;
  point.z = start.z + (end.z - start.z) * ratio;
  return point;
}

inline std::vector<geometry_msgs::msg::Point> ResamplePath(
    const std::vector<geometry_msgs::msg::Point>& points, double spacing_meters) {
  if (points.size() < 2 || spacing_meters <= 0.0) {
    return {};
  }

  std::vector<geometry_msgs::msg::Point> samples;
  samples.push_back(points.front());

  double distance_since_sample = 0.0;
  auto segment_start = points.front();

  for (std::size_t i = 1; i < points.size(); ++i) {
    auto segment_end = points[i];
    auto segment_length = Distance(segment_start, segment_end);

    while (distance_since_sample + segment_length >= spacing_meters && segment_length > 1e-9) {
      const auto remaining = spacing_meters - distance_since_sample;
      const auto ratio = remaining / segment_length;
      auto sample = Interpolate(segment_start, segment_end, ratio);
      samples.push_back(sample);
      segment_start = sample;
      segment_length = Distance(segment_start, segment_end);
      distance_since_sample = 0.0;
    }

    distance_since_sample += segment_length;
    segment_start = segment_end;
  }

  if (Distance(samples.back(), points.back()) > spacing_meters * 0.5) {
    samples.push_back(points.back());
  }

  return samples;
}

inline PlanFault ValidatePath(const std::vector<geometry_msgs::msg::Point>& points,
                              double max_gap_meters) {
  if (points.size() < 2) {
    return PlanFault::kInsufficientPoints;
  }
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return PlanFault::kInvalidGeometry;
    }
  }
  if (HasExcessiveGap(points, max_gap_meters)) {
    return PlanFault::kExcessiveGap;
  }
  return PlanFault::kNone;
}

}  // namespace weld_planner
