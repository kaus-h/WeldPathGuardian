#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include "fault_codes.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace weld_planner {

enum class PlanFault {
  kNone,
  kInsufficientPoints,
  kStaleObservation,
  kLowConfidence,
  kExcessiveGap,
  kExcessiveCurvature,
  kInvalidGeometry,
};

inline std::string_view ToString(PlanFault fault) {
  namespace fault_codes = weld_interfaces::fault_codes;

  switch (fault) {
    case PlanFault::kNone:
      return fault_codes::ToString(fault_codes::FaultCode::kNone);
    case PlanFault::kInsufficientPoints:
      return fault_codes::ToString(fault_codes::FaultCode::kInsufficientPoints);
    case PlanFault::kStaleObservation:
      return fault_codes::ToString(fault_codes::FaultCode::kStaleObservation);
    case PlanFault::kLowConfidence:
      return fault_codes::ToString(fault_codes::FaultCode::kLowConfidence);
    case PlanFault::kExcessiveGap:
      return fault_codes::ToString(fault_codes::FaultCode::kExcessiveGap);
    case PlanFault::kExcessiveCurvature:
      return fault_codes::ToString(fault_codes::FaultCode::kExcessiveCurvature);
    case PlanFault::kInvalidGeometry:
      return fault_codes::ToString(fault_codes::FaultCode::kInvalidGeometry);
  }
  return fault_codes::ToString(fault_codes::FaultCode::kInvalidGeometry);
}

inline double Distance(const geometry_msgs::msg::Point& lhs, const geometry_msgs::msg::Point& rhs) {
  const auto dx = lhs.x - rhs.x;
  const auto dy = lhs.y - rhs.y;
  const auto dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline tf2::Vector3 ToVector(const geometry_msgs::msg::Point& point) {
  return {point.x, point.y, point.z};
}

inline double PathLength(const std::vector<geometry_msgs::msg::Point>& points) {
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += Distance(points[i - 1], points[i]);
  }
  return length;
}

inline double SegmentAngle(const tf2::Vector3& first, const tf2::Vector3& second) {
  const auto first_length = first.length();
  const auto second_length = second.length();
  if (first_length <= 1e-9 || second_length <= 1e-9) {
    return 0.0;
  }
  const auto cosine = std::clamp(first.dot(second) / (first_length * second_length), -1.0, 1.0);
  return std::acos(cosine);
}

inline double MaxCurvature(const std::vector<geometry_msgs::msg::Point>& points) {
  double max_curvature = 0.0;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const auto previous = ToVector(points[i]) - ToVector(points[i - 1]);
    const auto next = ToVector(points[i + 1]) - ToVector(points[i]);
    const auto average_length = (previous.length() + next.length()) * 0.5;
    if (average_length <= 1e-9) {
      continue;
    }
    max_curvature = std::max(max_curvature, SegmentAngle(previous, next) / average_length);
  }
  return max_curvature;
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

inline geometry_msgs::msg::Quaternion MakeToolOrientation(
    const geometry_msgs::msg::Point& previous, const geometry_msgs::msg::Point& next,
    const geometry_msgs::msg::Point& surface_normal) {
  auto tangent = ToVector(next) - ToVector(previous);
  if (tangent.length() <= 1e-9) {
    tangent = {1.0, 0.0, 0.0};
  }
  tangent.normalize();

  auto normal = ToVector(surface_normal);
  if (normal.length() <= 1e-9) {
    normal = {0.0, 0.0, 1.0};
  }
  normal.normalize();

  auto lateral = normal.cross(tangent);
  if (lateral.length() <= 1e-9) {
    lateral = tf2::Vector3{0.0, 1.0, 0.0};
  }
  lateral.normalize();
  normal = tangent.cross(lateral);
  normal.normalize();

  tf2::Matrix3x3 rotation(tangent.x(), lateral.x(), normal.x(), tangent.y(), lateral.y(),
                          normal.y(), tangent.z(), lateral.z(), normal.z());
  tf2::Quaternion quaternion;
  rotation.getRotation(quaternion);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
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
                              double max_gap_meters, double max_curvature_rad_per_meter) {
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
  if (MaxCurvature(points) > max_curvature_rad_per_meter) {
    return PlanFault::kExcessiveCurvature;
  }
  return PlanFault::kNone;
}

}  // namespace weld_planner
