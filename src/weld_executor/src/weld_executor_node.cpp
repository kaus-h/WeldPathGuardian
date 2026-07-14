#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "weld_executor/state_machine.hpp"
#include "weld_interfaces/action/execute_weld.hpp"
#include "weld_interfaces/msg/system_status.hpp"
#include "weld_interfaces/msg/weld_plan.hpp"

class WeldExecutorNode final : public rclcpp::Node {
 public:
  using ExecuteWeld = weld_interfaces::action::ExecuteWeld;
  using GoalHandleExecuteWeld = rclcpp_action::ServerGoalHandle<ExecuteWeld>;

  WeldExecutorNode() : Node("weld_executor") {
    auto_execute_ = declare_parameter<bool>("auto_execute", true);
    waypoint_delay_ms_ = declare_parameter<int>("waypoint_delay_ms", 40);

    plan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    status_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    status_pub_ = create_publisher<weld_interfaces::msg::SystemStatus>("/weld/status", 10);

    rclcpp::SubscriptionOptions plan_options;
    plan_options.callback_group = plan_group_;
    plan_sub_ = create_subscription<weld_interfaces::msg::WeldPlan>(
        "/weld/plan", 10, [this](weld_interfaces::msg::WeldPlan::SharedPtr plan) { OnPlan(*plan); },
        plan_options);

    action_server_ = rclcpp_action::create_server<ExecuteWeld>(
        this, "execute_weld",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecuteWeld::Goal> goal) {
          return HandleGoal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
          return HandleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
          HandleAccepted(goal_handle);
        },
        rcl_action_server_get_default_options(), action_group_);

    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(250), [this] { PublishStatus(); }, status_group_);
  }

 private:
  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID&,
                                         const std::shared_ptr<const ExecuteWeld::Goal> goal) {
    if (!goal->plan.valid || goal->plan.waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting invalid ExecuteWeld goal: %s",
                  goal->plan.fault_code.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::scoped_lock lock(state_mutex_);
    if (state_ != weld_executor::ExecutionState::kIdle && !weld_executor::IsTerminal(state_)) {
      RCLCPP_WARN(get_logger(), "Rejecting ExecuteWeld goal while state is %s",
                  weld_executor::ToString(state_));
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleExecuteWeld>) {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
    std::thread([this, goal_handle] {
      ExecutePlan(goal_handle->get_goal()->plan, goal_handle);
    }).detach();
  }

  void OnPlan(const weld_interfaces::msg::WeldPlan& plan) {
    if (!plan.valid) {
      planning_failures_.fetch_add(1);
      SetLastFault(plan.fault_code);
      TrySetState(weld_executor::ExecutionState::kFaulted);
      PublishStatus();
      return;
    }

    if (!auto_execute_) {
      return;
    }

    std::thread([this, plan] { ExecutePlan(plan, nullptr); }).detach();
  }

  bool BeginExecution() {
    std::scoped_lock lock(state_mutex_);
    if (state_ != weld_executor::ExecutionState::kIdle && !weld_executor::IsTerminal(state_)) {
      return false;
    }
    state_ = weld_executor::ExecutionState::kValidating;
    last_fault_ = "None";
    cancel_requested_.store(false);
    completed_waypoints_.store(0);
    return true;
  }

  bool TrySetState(weld_executor::ExecutionState target) {
    std::scoped_lock lock(state_mutex_);
    if (!weld_executor::CanTransition(state_, target)) {
      state_ = weld_executor::ExecutionState::kFaulted;
      last_fault_ = "InvalidStateTransition";
      return false;
    }
    state_ = target;
    return true;
  }

  std::string CurrentState() const {
    std::scoped_lock lock(state_mutex_);
    return weld_executor::ToString(state_);
  }

  void SetLastFault(const std::string& fault) {
    std::scoped_lock lock(state_mutex_);
    last_fault_ = fault;
  }

  void ExecutePlan(const weld_interfaces::msg::WeldPlan& plan,
                   std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
    const auto start = std::chrono::steady_clock::now();
    if (!BeginExecution()) {
      if (goal_handle) {
        auto result = std::make_shared<ExecuteWeld::Result>();
        result->success = false;
        result->fault_code = "Busy";
        result->completed_waypoints = 0;
        result->execution_time_ms = 0.0;
        goal_handle->abort(result);
      }
      return;
    }
    PublishStatus();

    if (!plan.valid || plan.waypoints.empty()) {
      FinishWithFault(goal_handle, "InvalidPlan", start);
      return;
    }

    TrySetState(weld_executor::ExecutionState::kPlanning);
    PublishStatus();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    TrySetState(weld_executor::ExecutionState::kReady);
    PublishStatus();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    TrySetState(weld_executor::ExecutionState::kExecuting);
    PublishStatus();

    for (std::size_t i = 0; i < plan.waypoints.size(); ++i) {
      if (cancel_requested_.load() || (goal_handle && goal_handle->is_canceling())) {
        TrySetState(weld_executor::ExecutionState::kCancelled);
        SetLastFault("Cancelled");
        PublishStatus();
        if (goal_handle) {
          auto result = std::make_shared<ExecuteWeld::Result>();
          result->success = false;
          result->fault_code = "Cancelled";
          result->completed_waypoints = completed_waypoints_.load();
          result->execution_time_ms = ElapsedMs(start);
          goal_handle->canceled(result);
        }
        return;
      }

      completed_waypoints_.store(static_cast<uint32_t>(i + 1));
      if (goal_handle) {
        auto feedback = std::make_shared<ExecuteWeld::Feedback>();
        feedback->current_waypoint = static_cast<uint32_t>(i);
        feedback->completion_percent = static_cast<float>(
            (100.0 * static_cast<double>(i + 1)) / static_cast<double>(plan.waypoints.size()));
        feedback->current_state = CurrentState();
        goal_handle->publish_feedback(feedback);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(waypoint_delay_ms_));
    }

    TrySetState(weld_executor::ExecutionState::kCompleted);
    latest_execution_ms_.store(ElapsedMs(start));
    PublishStatus();

    if (goal_handle) {
      auto result = std::make_shared<ExecuteWeld::Result>();
      result->success = true;
      result->fault_code = "None";
      result->completed_waypoints = completed_waypoints_.load();
      result->execution_time_ms = latest_execution_ms_.load();
      goal_handle->succeed(result);
    }
  }

  void FinishWithFault(const std::shared_ptr<GoalHandleExecuteWeld>& goal_handle,
                       const std::string& fault, std::chrono::steady_clock::time_point start) {
    execution_faults_.fetch_add(1);
    SetLastFault(fault);
    TrySetState(weld_executor::ExecutionState::kFaulted);
    latest_execution_ms_.store(ElapsedMs(start));
    PublishStatus();

    if (goal_handle) {
      auto result = std::make_shared<ExecuteWeld::Result>();
      result->success = false;
      result->fault_code = fault;
      result->completed_waypoints = completed_waypoints_.load();
      result->execution_time_ms = latest_execution_ms_.load();
      goal_handle->abort(result);
    }
  }

  double ElapsedMs(std::chrono::steady_clock::time_point start) const {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    return static_cast<double>(
               std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()) /
           1000.0;
  }

  void PublishStatus() {
    weld_interfaces::msg::SystemStatus status;
    status.header.stamp = get_clock()->now();
    status.header.frame_id = "weld_cell";
    {
      std::scoped_lock lock(state_mutex_);
      status.state = weld_executor::ToString(state_);
      status.latest_fault = last_fault_.empty() ? "None" : last_fault_;
    }
    status.planning_failures = planning_failures_.load();
    status.execution_faults = execution_faults_.load();
    status.latest_latency_ms = latest_execution_ms_.load();
    status_pub_->publish(status);
  }

  bool auto_execute_{true};
  int waypoint_delay_ms_{40};
  mutable std::mutex state_mutex_;
  weld_executor::ExecutionState state_{weld_executor::ExecutionState::kIdle};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<uint32_t> completed_waypoints_{0};
  std::atomic<uint32_t> planning_failures_{0};
  std::atomic<uint32_t> execution_faults_{0};
  std::atomic<double> latest_execution_ms_{0.0};
  std::string last_fault_{"None"};

  rclcpp::CallbackGroup::SharedPtr plan_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  rclcpp::Subscription<weld_interfaces::msg::WeldPlan>::SharedPtr plan_sub_;
  rclcpp::Publisher<weld_interfaces::msg::SystemStatus>::SharedPtr status_pub_;
  rclcpp_action::Server<ExecuteWeld>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WeldExecutorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
