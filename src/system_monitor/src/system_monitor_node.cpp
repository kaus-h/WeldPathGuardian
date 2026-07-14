#include <atomic>
#include <chrono>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "weld_interfaces/msg/filtered_seam.hpp"
#include "weld_interfaces/msg/seam_observation.hpp"
#include "weld_interfaces/msg/system_status.hpp"
#include "weld_interfaces/msg/weld_plan.hpp"

class SystemMonitorNode final : public rclcpp::Node {
 public:
  SystemMonitorNode() : Node("system_monitor") {
    raw_sub_ = create_subscription<weld_interfaces::msg::SeamObservation>(
        "/seam/raw", 10, [this](weld_interfaces::msg::SeamObservation::SharedPtr msg) {
          ++raw_observations_;
          latest_raw_latency_ms_.store(AgeMs(msg->header.stamp));
        });

    filtered_sub_ = create_subscription<weld_interfaces::msg::FilteredSeam>(
        "/seam/filtered", 10, [this](weld_interfaces::msg::FilteredSeam::SharedPtr msg) {
          ++filtered_observations_;
          rejected_observations_.fetch_add(msg->rejected_points);
          latest_filtered_latency_ms_.store(AgeMs(msg->header.stamp));
          if (!msg->valid) {
            latest_fault_ = msg->fault_code;
          }
        });

    plan_sub_ = create_subscription<weld_interfaces::msg::WeldPlan>(
        "/weld/plan", 10, [this](weld_interfaces::msg::WeldPlan::SharedPtr msg) {
          latest_plan_latency_ms_.store(AgeMs(msg->header.stamp));
          if (!msg->valid) {
            ++planning_failures_;
            latest_fault_ = msg->fault_code;
          }
        });

    status_sub_ = create_subscription<weld_interfaces::msg::SystemStatus>(
        "/weld/status", 10, [this](weld_interfaces::msg::SystemStatus::SharedPtr msg) {
          latest_state_ = msg->state;
          execution_faults_.store(msg->execution_faults);
          if (msg->latest_fault != "None") {
            latest_fault_ = msg->latest_fault;
          }
        });

    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { Report(); });
  }

 private:
  double AgeMs(const builtin_interfaces::msg::Time& stamp) const {
    return static_cast<double>((get_clock()->now() - rclcpp::Time(stamp)).nanoseconds()) /
           1000000.0;
  }

  void Report() {
    RCLCPP_INFO(get_logger(),
                "state=%s raw=%u filtered=%u rejected=%u plan_failures=%u exec_faults=%u "
                "latency_ms(raw=%.2f filtered=%.2f plan=%.2f) latest_fault=%s",
                latest_state_.c_str(), raw_observations_.load(), filtered_observations_.load(),
                rejected_observations_.load(), planning_failures_.load(), execution_faults_.load(),
                latest_raw_latency_ms_.load(), latest_filtered_latency_ms_.load(),
                latest_plan_latency_ms_.load(), latest_fault_.c_str());
  }

  std::atomic<uint32_t> raw_observations_{0};
  std::atomic<uint32_t> filtered_observations_{0};
  std::atomic<uint32_t> rejected_observations_{0};
  std::atomic<uint32_t> planning_failures_{0};
  std::atomic<uint32_t> execution_faults_{0};
  std::atomic<double> latest_raw_latency_ms_{0.0};
  std::atomic<double> latest_filtered_latency_ms_{0.0};
  std::atomic<double> latest_plan_latency_ms_{0.0};
  std::string latest_state_{"IDLE"};
  std::string latest_fault_{"None"};

  rclcpp::Subscription<weld_interfaces::msg::SeamObservation>::SharedPtr raw_sub_;
  rclcpp::Subscription<weld_interfaces::msg::FilteredSeam>::SharedPtr filtered_sub_;
  rclcpp::Subscription<weld_interfaces::msg::WeldPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<weld_interfaces::msg::SystemStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
