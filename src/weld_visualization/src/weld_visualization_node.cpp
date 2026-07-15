#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "weld_interfaces/msg/system_status.hpp"

namespace {

std_msgs::msg::ColorRGBA ColorForState(const std::string& state) {
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0F;
  if (state == "EXECUTING" || state == "COMPLETED") {
    color.r = 0.1F;
    color.g = 0.9F;
    color.b = 0.2F;
  } else if (state == "FAULTED" || state == "CANCELLED") {
    color.r = 1.0F;
    color.g = 0.05F;
    color.b = 0.05F;
  } else {
    color.r = 1.0F;
    color.g = 0.85F;
    color.b = 0.1F;
  }
  return color;
}

}  // namespace

class WeldVisualizationNode final : public rclcpp::Node {
 public:
  WeldVisualizationNode() : Node("weld_visualization") {
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/weld/status_markers",
                                                                         reliable_qos);
    status_sub_ = create_subscription<weld_interfaces::msg::SystemStatus>(
        "/weld/status", reliable_qos,
        [this](weld_interfaces::msg::SystemStatus::SharedPtr msg) { PublishStatus(*msg); });
  }

 private:
  void PublishStatus(const weld_interfaces::msg::SystemStatus& status) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker text;
    text.header = status.header;
    text.ns = "weld_status";
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 0.2;
    text.pose.position.y = -0.35;
    text.pose.position.z = 0.25;
    text.scale.z = 0.055;
    text.color = ColorForState(status.state);
    text.text = "State: " + status.state + "\nFault: " + status.latest_fault;
    markers.markers.push_back(text);
    marker_pub_->publish(markers);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<weld_interfaces::msg::SystemStatus>::SharedPtr status_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeldVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
