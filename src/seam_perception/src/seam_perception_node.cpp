#include <algorithm>
#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "seam_perception/perception_utils.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "weld_interfaces/msg/filtered_seam.hpp"
#include "weld_interfaces/msg/seam_observation.hpp"

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

class SeamPerceptionNode final : public rclcpp::Node {
 public:
  SeamPerceptionNode() : Node("seam_perception") {
    min_confidence_ = declare_parameter<double>("min_confidence", 0.55);
    stale_threshold_ms_ = declare_parameter<int>("stale_threshold_ms", 350);
    smoothing_window_ = declare_parameter<int>("smoothing_window", 1);
    min_points_ = declare_parameter<int>("min_points", 4);
    max_neighbor_distance_ = declare_parameter<double>("max_neighbor_distance", 0.20);

    filtered_pub_ = create_publisher<weld_interfaces::msg::FilteredSeam>("/seam/filtered", 10);
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/seam/filtered_markers", 10);
    subscription_ = create_subscription<weld_interfaces::msg::SeamObservation>(
        "/seam/raw", 10,
        [this](weld_interfaces::msg::SeamObservation::SharedPtr msg) { ProcessObservation(*msg); });
  }

 private:
  void ProcessObservation(const weld_interfaces::msg::SeamObservation& observation) {
    weld_interfaces::msg::FilteredSeam filtered;
    filtered.header = observation.header;
    filtered.valid = true;
    filtered.fault_code = "None";

    const auto age =
        (get_clock()->now() - rclcpp::Time(observation.header.stamp)).nanoseconds() / 1000000.0;
    if (age > static_cast<double>(stale_threshold_ms_)) {
      filtered.valid = false;
      filtered.fault_code = "StaleObservation";
    }

    std::vector<float> accepted_confidences;
    for (std::size_t i = 0; i < observation.points.size(); ++i) {
      const auto valid_flag = i < observation.valid.size() && observation.valid[i];
      const auto confidence =
          i < observation.confidences.size() ? observation.confidences[i] : 0.0F;

      if (!valid_flag || confidence < min_confidence_ ||
          !seam_perception::IsFinitePoint(observation.points[i])) {
        ++filtered.rejected_points;
        continue;
      }

      filtered.points.push_back(observation.points[i]);
      accepted_confidences.push_back(confidence);
    }

    filtered.mean_confidence =
        static_cast<float>(seam_perception::MeanConfidence(accepted_confidences));

    if (filtered.points.size() < static_cast<std::size_t>(min_points_)) {
      filtered.valid = false;
      if (filtered.fault_code == "None") {
        filtered.fault_code = "InsufficientPoints";
      }
    }

    if (filtered.valid &&
        seam_perception::HasExcessiveNeighborJump(filtered.points, max_neighbor_distance_)) {
      filtered.valid = false;
      filtered.fault_code = "ExcessiveGap";
    }

    if (filtered.valid) {
      filtered.points =
          seam_perception::SmoothPath(filtered.points, static_cast<std::size_t>(smoothing_window_));
    }

    PublishMarkers(filtered);
    filtered_pub_->publish(filtered);
  }

  void PublishMarkers(const weld_interfaces::msg::FilteredSeam& filtered) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker line;
    line.header = filtered.header;
    line.ns = "filtered_seam";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.012;
    line.color =
        filtered.valid ? Color(0.1F, 0.35F, 1.0F, 0.95F) : Color(1.0F, 0.05F, 0.05F, 0.95F);
    line.points = filtered.points;

    markers.markers.push_back(line);
    marker_pub_->publish(markers);
  }

  double min_confidence_{0.55};
  int stale_threshold_ms_{350};
  int smoothing_window_{1};
  int min_points_{4};
  double max_neighbor_distance_{0.20};
  rclcpp::Publisher<weld_interfaces::msg::FilteredSeam>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<weld_interfaces::msg::SeamObservation>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeamPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
