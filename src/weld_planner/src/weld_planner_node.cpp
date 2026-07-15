#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "weld_interfaces/msg/filtered_seam.hpp"
#include "weld_interfaces/msg/weld_plan.hpp"
#include "weld_planner/planner_utils.hpp"

namespace {

std_msgs::msg::ColorRGBA Color(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace

class WeldPlannerNode final : public rclcpp::Node {
 public:
  WeldPlannerNode() : Node("weld_planner") {
    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 0.05);
    max_gap_ = declare_parameter<double>("max_gap", 0.18);
    max_curvature_rad_per_meter_ = declare_parameter<double>("max_curvature_rad_per_meter", 45.0);
    tool_speed_ = declare_parameter<double>("tool_speed", 0.04);
    surface_normal_.x = declare_parameter<double>("surface_normal_x", 0.0);
    surface_normal_.y = declare_parameter<double>("surface_normal_y", 0.0);
    surface_normal_.z = declare_parameter<double>("surface_normal_z", 1.0);

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    plan_pub_ = create_publisher<weld_interfaces::msg::WeldPlan>("/weld/plan", reliable_qos);
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/weld/plan_markers", reliable_qos);
    subscription_ = create_subscription<weld_interfaces::msg::FilteredSeam>(
        "/seam/filtered", reliable_qos,
        [this](weld_interfaces::msg::FilteredSeam::SharedPtr msg) { BuildPlan(*msg); });
  }

 private:
  void BuildPlan(const weld_interfaces::msg::FilteredSeam& seam) {
    const auto processing_start = std::chrono::steady_clock::now();
    weld_interfaces::msg::WeldPlan plan;
    plan.header = seam.header;
    plan.valid = false;
    plan.fault_code =
        seam.valid
            ? weld_interfaces::fault_codes::ToString(weld_interfaces::fault_codes::FaultCode::kNone)
            : seam.fault_code;

    if (!seam.valid) {
      SetProcessingLatency(processing_start, plan);
      Publish(plan);
      return;
    }

    const auto validation_fault =
        weld_planner::ValidatePath(seam.points, max_gap_, max_curvature_rad_per_meter_);
    if (validation_fault != weld_planner::PlanFault::kNone) {
      plan.fault_code = weld_planner::ToString(validation_fault);
      SetProcessingLatency(processing_start, plan);
      Publish(plan);
      return;
    }

    const auto samples = weld_planner::ResamplePath(seam.points, waypoint_spacing_);
    if (samples.size() < 2) {
      plan.fault_code = weld_interfaces::fault_codes::ToString(
          weld_interfaces::fault_codes::FaultCode::kInsufficientPoints);
      SetProcessingLatency(processing_start, plan);
      Publish(plan);
      return;
    }

    plan.valid = true;
    plan.path_length = static_cast<float>(weld_planner::PathLength(samples));
    plan.max_curvature_rad_per_meter = static_cast<float>(weld_planner::MaxCurvature(samples));

    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto next_index = std::min(i + 1, samples.size() - 1);
      const auto prev_index = i == 0 ? 0 : i - 1;

      weld_interfaces::msg::WeldWaypoint waypoint;
      waypoint.pose.position = samples[i];
      waypoint.pose.orientation = weld_planner::MakeToolOrientation(
          samples[prev_index], samples[next_index], surface_normal_);
      waypoint.speed = static_cast<float>(tool_speed_);
      plan.waypoints.push_back(waypoint);
    }

    SetProcessingLatency(processing_start, plan);
    Publish(plan);
  }

  void SetProcessingLatency(std::chrono::steady_clock::time_point start,
                            weld_interfaces::msg::WeldPlan& plan) const {
    const auto processing_elapsed = std::chrono::steady_clock::now() - start;
    plan.processing_latency_ms =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(processing_elapsed).count()) /
        1000.0;
  }

  void Publish(const weld_interfaces::msg::WeldPlan& plan) {
    PublishMarkers(plan);
    plan_pub_->publish(plan);
  }

  void PublishMarkers(const weld_interfaces::msg::WeldPlan& plan) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker arrows;
    arrows.header = plan.header;
    arrows.ns = "weld_waypoints";
    arrows.id = 1;
    arrows.type = visualization_msgs::msg::Marker::LINE_LIST;
    arrows.action = visualization_msgs::msg::Marker::ADD;
    arrows.scale.x = 0.008;
    arrows.color = plan.valid ? Color(1.0F, 0.85F, 0.05F, 0.95F) : Color(1.0F, 0.05F, 0.05F, 0.95F);

    for (const auto& waypoint : plan.waypoints) {
      const auto& start = waypoint.pose.position;
      auto end = start;
      end.z += 0.04;
      arrows.points.push_back(start);
      arrows.points.push_back(end);
    }

    markers.markers.push_back(arrows);
    marker_pub_->publish(markers);
  }

  double waypoint_spacing_{0.05};
  double max_gap_{0.18};
  double max_curvature_rad_per_meter_{45.0};
  double tool_speed_{0.04};
  geometry_msgs::msg::Point surface_normal_;
  rclcpp::Publisher<weld_interfaces::msg::WeldPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<weld_interfaces::msg::FilteredSeam>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeldPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
