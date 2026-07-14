#include <algorithm>
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
    tool_speed_ = declare_parameter<double>("tool_speed", 0.04);

    plan_pub_ = create_publisher<weld_interfaces::msg::WeldPlan>("/weld/plan", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/weld/plan_markers", 10);
    subscription_ = create_subscription<weld_interfaces::msg::FilteredSeam>(
        "/seam/filtered", 10,
        [this](weld_interfaces::msg::FilteredSeam::SharedPtr msg) { BuildPlan(*msg); });
  }

 private:
  void BuildPlan(const weld_interfaces::msg::FilteredSeam& seam) {
    weld_interfaces::msg::WeldPlan plan;
    plan.header = seam.header;
    plan.valid = false;
    plan.fault_code = seam.valid ? "None" : seam.fault_code;

    if (!seam.valid) {
      Publish(plan);
      return;
    }

    const auto validation_fault = weld_planner::ValidatePath(seam.points, max_gap_);
    if (validation_fault != weld_planner::PlanFault::kNone) {
      plan.fault_code = weld_planner::ToString(validation_fault);
      Publish(plan);
      return;
    }

    const auto samples = weld_planner::ResamplePath(seam.points, waypoint_spacing_);
    if (samples.size() < 2) {
      plan.fault_code = "InsufficientPoints";
      Publish(plan);
      return;
    }

    plan.valid = true;
    plan.path_length = static_cast<float>(weld_planner::PathLength(samples));

    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto next_index = std::min(i + 1, samples.size() - 1);
      const auto prev_index = i == 0 ? 0 : i - 1;
      const auto dx = samples[next_index].x - samples[prev_index].x;
      const auto dy = samples[next_index].y - samples[prev_index].y;
      const auto yaw = std::atan2(dy, dx);

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);

      weld_interfaces::msg::WeldWaypoint waypoint;
      waypoint.pose.position = samples[i];
      waypoint.pose.orientation = tf2::toMsg(orientation);
      waypoint.speed = static_cast<float>(tool_speed_);
      plan.waypoints.push_back(waypoint);
    }

    Publish(plan);
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
  double tool_speed_{0.04};
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
