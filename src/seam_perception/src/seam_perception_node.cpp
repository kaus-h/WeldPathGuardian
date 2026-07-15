#include <algorithm>
#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "fault_codes.hpp"
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
    fitting_window_ = declare_parameter<int>("fitting_window", 3);
    min_points_ = declare_parameter<int>("min_points", 4);
    max_neighbor_distance_ = declare_parameter<double>("max_neighbor_distance", 0.20);

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    filtered_pub_ =
        create_publisher<weld_interfaces::msg::FilteredSeam>("/seam/filtered", reliable_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/seam/filtered_markers",
                                                                         reliable_qos);
    subscription_ = create_subscription<weld_interfaces::msg::SeamObservation>(
        "/seam/raw", rclcpp::SensorDataQoS(),
        [this](weld_interfaces::msg::SeamObservation::SharedPtr msg) { ProcessObservation(*msg); });
  }

 private:
  void ProcessObservation(const weld_interfaces::msg::SeamObservation& observation) {
    namespace fault_codes = weld_interfaces::fault_codes;

    const auto processing_start = std::chrono::steady_clock::now();
    weld_interfaces::msg::FilteredSeam filtered;
    filtered.header = observation.header;
    filtered.valid = true;
    filtered.fault_code = fault_codes::ToString(fault_codes::FaultCode::kNone);

    const auto age =
        (get_clock()->now() - rclcpp::Time(observation.header.stamp)).nanoseconds() / 1000000.0;
    if (age > static_cast<double>(stale_threshold_ms_)) {
      filtered.valid = false;
      filtered.fault_code = fault_codes::ToString(fault_codes::FaultCode::kStaleObservation);
    }

    std::vector<float> accepted_confidences;
    std::size_t confidence_rejections = 0;
    for (std::size_t i = 0; i < observation.points.size(); ++i) {
      const auto valid_flag = i < observation.valid.size() && observation.valid[i];
      const auto confidence =
          i < observation.confidences.size() ? observation.confidences[i] : 0.0F;

      if (!valid_flag || !seam_perception::IsFinitePoint(observation.points[i])) {
        ++filtered.rejected_points;
        continue;
      }

      if (confidence < min_confidence_) {
        ++filtered.rejected_points;
        ++confidence_rejections;
        continue;
      }

      filtered.points.push_back(observation.points[i]);
      accepted_confidences.push_back(confidence);
    }

    filtered.mean_confidence =
        static_cast<float>(seam_perception::MeanConfidence(accepted_confidences));

    if (filtered.points.size() < static_cast<std::size_t>(min_points_)) {
      filtered.valid = false;
      if (filtered.fault_code == fault_codes::kNone) {
        filtered.fault_code =
            confidence_rejections > 0
                ? fault_codes::ToString(fault_codes::FaultCode::kLowConfidence)
                : fault_codes::ToString(fault_codes::FaultCode::kInsufficientPoints);
      }
    }

    filtered.max_gap = static_cast<float>(seam_perception::MaxNeighborDistance(filtered.points));
    if (filtered.valid &&
        seam_perception::HasExcessiveNeighborJump(filtered.points, max_neighbor_distance_)) {
      filtered.valid = false;
      filtered.fault_code = fault_codes::ToString(fault_codes::FaultCode::kExcessiveGap);
    }

    if (filtered.valid) {
      const auto smoothed =
          seam_perception::SmoothPath(filtered.points, static_cast<std::size_t>(smoothing_window_));
      filtered.points = seam_perception::FitLocalLeastSquaresPath(
          smoothed, static_cast<std::size_t>(fitting_window_));
      filtered.fit_error =
          static_cast<float>(seam_perception::RootMeanSquareError(smoothed, filtered.points));
    }

    const auto processing_elapsed = std::chrono::steady_clock::now() - processing_start;
    filtered.processing_latency_ms =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(processing_elapsed).count()) /
        1000.0;

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
  int fitting_window_{3};
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
