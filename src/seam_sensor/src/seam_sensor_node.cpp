#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "weld_interfaces/msg/seam_observation.hpp"

namespace {

using namespace std::chrono_literals;

geometry_msgs::msg::Point MakePoint(double x, double y, double z) {
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA Color(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace

class SeamSensorNode final : public rclcpp::Node {
 public:
  SeamSensorNode() : Node("seam_sensor"), rng_(std::random_device{}()) {
    scenario_ = declare_parameter<std::string>("scenario", "gaussian_noise");
    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    point_count_ = declare_parameter<int>("point_count", 60);
    noise_stddev_ = declare_parameter<double>("noise_stddev", 0.004);
    dropout_ratio_ = declare_parameter<double>("dropout_ratio", 0.08);
    confidence_mean_ = declare_parameter<double>("confidence_mean", 0.92);
    stale_offset_ms_ = declare_parameter<int>("stale_offset_ms", 800);

    observation_pub_ = create_publisher<weld_interfaces::msg::SeamObservation>("/seam/raw", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/seam/raw_markers", 10);

    const auto period_ms = std::max(1, static_cast<int>(1000.0 / std::max(rate_hz_, 1.0)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this] { PublishSample(); });

    RCLCPP_INFO(get_logger(), "seam_sensor publishing scenario '%s' at %.1f Hz", scenario_.c_str(),
                rate_hz_);
  }

 private:
  void PublishSample() {
    const auto scenario = get_parameter("scenario").as_string();
    const auto now = get_clock()->now();
    const auto sample_index = sample_count_.fetch_add(1);

    weld_interfaces::msg::SeamObservation observation;
    observation.header.frame_id = "weld_cell";
    observation.header.stamp = now;
    observation.scenario = scenario;

    if (scenario == "stale_messages") {
      observation.header.stamp = now - rclcpp::Duration::from_nanoseconds(
                                           static_cast<int64_t>(stale_offset_ms_) * 1000000);
    }

    if (scenario == "complete_dropout") {
      PublishMarkers(observation);
      observation_pub_->publish(observation);
      return;
    }

    std::normal_distribution<double> noise(0.0, noise_stddev_);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> confidence_noise(0.0, 0.06);

    const auto count = std::max(point_count_, 2);
    observation.points.reserve(static_cast<std::size_t>(count));
    observation.confidences.reserve(static_cast<std::size_t>(count));
    observation.valid.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
      const auto t = static_cast<double>(i) / static_cast<double>(count - 1);
      const auto x = t * 1.2;
      auto y = 0.08 * std::sin(t * 2.2 * std::numbers::pi);
      auto z = 0.03 * std::cos(t * std::numbers::pi);

      if (scenario == "sudden_offset" && t > 0.55) {
        y += 0.09;
        z += 0.03;
      }

      auto point = MakePoint(x, y, z);
      if (scenario == "gaussian_noise" || scenario == "missing_segment" ||
          scenario == "sudden_offset" || scenario == "low_confidence_recovery") {
        point.x += noise(rng_);
        point.y += noise(rng_);
        point.z += noise(rng_) * 0.5;
      }

      auto valid = true;
      if (scenario == "missing_segment" && t > 0.42 && t < 0.58) {
        valid = false;
      }
      if (scenario == "gaussian_noise" && unit(rng_) < dropout_ratio_) {
        valid = false;
      }

      const auto in_confidence_dip =
          scenario == "low_confidence_recovery" && sample_index >= 10 && sample_index < 80;
      const auto confidence_baseline = in_confidence_dip ? 0.25 : confidence_mean_;
      const auto confidence =
          std::clamp(confidence_baseline + confidence_noise(rng_) - (valid ? 0.0 : 0.35), 0.0, 1.0);

      observation.points.push_back(point);
      observation.confidences.push_back(static_cast<float>(confidence));
      observation.valid.push_back(valid);
    }

    PublishMarkers(observation);
    observation_pub_->publish(observation);
  }

  void PublishMarkers(const weld_interfaces::msg::SeamObservation& observation) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker points;
    points.header = observation.header;
    points.ns = "raw_seam";
    points.id = 1;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = 0.018;
    points.scale.y = 0.018;
    points.scale.z = 0.018;
    points.color = Color(0.1F, 0.9F, 0.2F, 0.9F);

    visualization_msgs::msg::Marker rejected;
    rejected.header = observation.header;
    rejected.ns = "raw_rejected";
    rejected.id = 2;
    rejected.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    rejected.action = visualization_msgs::msg::Marker::ADD;
    rejected.scale.x = 0.025;
    rejected.scale.y = 0.025;
    rejected.scale.z = 0.025;
    rejected.color = Color(1.0F, 0.05F, 0.05F, 0.95F);

    for (std::size_t i = 0; i < observation.points.size(); ++i) {
      const auto is_valid = i < observation.valid.size() && observation.valid[i];
      if (is_valid) {
        points.points.push_back(observation.points[i]);
      } else {
        rejected.points.push_back(observation.points[i]);
      }
    }

    markers.markers.push_back(points);
    markers.markers.push_back(rejected);
    marker_pub_->publish(markers);
  }

  std::string scenario_;
  double rate_hz_{20.0};
  int point_count_{60};
  double noise_stddev_{0.004};
  double dropout_ratio_{0.08};
  double confidence_mean_{0.92};
  int stale_offset_ms_{800};
  std::atomic<uint64_t> sample_count_{0};
  std::mt19937 rng_;
  rclcpp::Publisher<weld_interfaces::msg::SeamObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeamSensorNode>());
  rclcpp::shutdown();
  return 0;
}
