#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "fault_codes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "weld_executor/state_machine.hpp"
#include "weld_interfaces/action/execute_weld.hpp"
#include "weld_interfaces/msg/system_status.hpp"
#include "weld_interfaces/msg/weld_plan.hpp"

namespace fault_codes = weld_interfaces::fault_codes;

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

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    status_pub_ =
        create_publisher<weld_interfaces::msg::SystemStatus>("/weld/status", reliable_qos);

    rclcpp::SubscriptionOptions plan_options;
    plan_options.callback_group = plan_group_;
    plan_sub_ = create_subscription<weld_interfaces::msg::WeldPlan>(
        "/weld/plan", reliable_qos,
        [this](weld_interfaces::msg::WeldPlan::SharedPtr plan) { OnPlan(*plan); }, plan_options);

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

    worker_ = std::jthread([this](std::stop_token stop_token) { WorkerLoop(stop_token); });
  }

  ~WeldExecutorNode() override {
    if (worker_.joinable()) {
      worker_.request_stop();
      cancel_requested_.store(true);
      job_cv_.notify_all();
      worker_.join();
    }
  }

 private:
  struct ExecutionJob {
    weld_interfaces::msg::WeldPlan plan;
    std::shared_ptr<GoalHandleExecuteWeld> goal_handle;
  };

  rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID&,
                                         const std::shared_ptr<const ExecuteWeld::Goal> goal) {
    if (!goal->plan.valid || goal->plan.waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting invalid ExecuteWeld goal: %s",
                  goal->plan.fault_code.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto state = CurrentExecutionState();
    if (state != weld_executor::ExecutionState::kIdle && !weld_executor::IsTerminal(state)) {
      RCLCPP_WARN(get_logger(), "Rejecting ExecuteWeld goal while state is %s",
                  weld_executor::ToString(state));
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (WorkerHasQueuedOrRunningJob()) {
      RCLCPP_WARN(get_logger(), "Rejecting ExecuteWeld goal while worker is busy");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleExecuteWeld>) {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
    if (!EnqueueExecution(goal_handle->get_goal()->plan, goal_handle)) {
      CompleteAborted(goal_handle, "Busy", 0, 0.0);
    }
  }

  void OnPlan(const weld_interfaces::msg::WeldPlan& plan) {
    if (!plan.valid) {
      planning_failures_.fetch_add(1);
      if (plan.fault_code == fault_codes::kLowConfidence) {
        EnterPausedForLowConfidence();
      } else {
        SetLastFault(plan.fault_code);
        TrySetState(weld_executor::ExecutionState::kFaulted);
      }
      PublishStatus();
      return;
    }

    if (!auto_execute_) {
      return;
    }

    if (!EnqueueExecution(plan, nullptr)) {
      dropped_plans_.fetch_add(1);
    }
  }

  bool EnqueueExecution(const weld_interfaces::msg::WeldPlan& plan,
                        std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
    {
      std::scoped_lock lock(job_mutex_);
      if (worker_busy_ || has_pending_job_) {
        return false;
      }
      ExecutionJob job;
      job.plan = plan;
      job.goal_handle = std::move(goal_handle);
      pending_job_ = std::move(job);
      has_pending_job_ = true;
    }
    job_cv_.notify_one();
    return true;
  }

  bool WorkerHasQueuedOrRunningJob() const {
    std::scoped_lock lock(job_mutex_);
    return worker_busy_ || has_pending_job_;
  }

  void WorkerLoop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
      ExecutionJob job;
      {
        std::unique_lock lock(job_mutex_);
        job_cv_.wait(lock, stop_token, [this] { return has_pending_job_; });
        if (stop_token.stop_requested()) {
          return;
        }
        job = std::move(pending_job_);
        has_pending_job_ = false;
        worker_busy_ = true;
      }

      ExecutePlan(stop_token, job.plan, job.goal_handle);

      {
        std::scoped_lock lock(job_mutex_);
        worker_busy_ = false;
      }
      job_cv_.notify_all();
    }
  }

  bool BeginExecution() {
    std::scoped_lock lock(state_mutex_);
    if (weld_executor::IsTerminal(state_)) {
      if (!weld_executor::CanTransition(state_, weld_executor::ExecutionState::kIdle)) {
        return false;
      }
      state_ = weld_executor::ExecutionState::kIdle;
    }
    const auto can_resume = state_ == weld_executor::ExecutionState::kPaused;
    if (state_ != weld_executor::ExecutionState::kIdle && !can_resume) {
      return false;
    }
    if (can_resume && pause_active_) {
      latest_recovery_ms_.store(ElapsedMsUnlocked(pause_started_));
      pause_active_ = false;
    }
    if (!weld_executor::CanTransition(state_, weld_executor::ExecutionState::kValidating)) {
      return false;
    }
    state_ = weld_executor::ExecutionState::kValidating;
    last_fault_ = fault_codes::kNone;
    cancel_requested_.store(false);
    completed_waypoints_.store(0);
    return true;
  }

  bool TrySetState(weld_executor::ExecutionState target) {
    std::scoped_lock lock(state_mutex_);
    if (!weld_executor::CanTransition(state_, target)) {
      state_ = weld_executor::ExecutionState::kFaulted;
      last_fault_ = fault_codes::kInvalidStateTransition;
      return false;
    }
    state_ = target;
    return true;
  }

  std::string CurrentState() const {
    std::scoped_lock lock(state_mutex_);
    return weld_executor::ToString(state_);
  }

  weld_executor::ExecutionState CurrentExecutionState() const {
    std::scoped_lock lock(state_mutex_);
    return state_;
  }

  void SetLastFault(std::string_view fault) {
    std::scoped_lock lock(state_mutex_);
    last_fault_ = fault;
  }

  std::string LastFault() const {
    std::scoped_lock lock(state_mutex_);
    return last_fault_.empty() ? std::string{fault_codes::kNone} : last_fault_;
  }

  void EnterPausedForLowConfidence() {
    std::scoped_lock lock(state_mutex_);
    last_fault_ = fault_codes::kLowConfidence;
    if (state_ == weld_executor::ExecutionState::kPaused) {
      if (!pause_active_) {
        pause_started_ = std::chrono::steady_clock::now();
        pause_active_ = true;
      }
      return;
    }
    if (!weld_executor::CanTransition(state_, weld_executor::ExecutionState::kPaused)) {
      state_ = weld_executor::ExecutionState::kFaulted;
      last_fault_ = fault_codes::kInvalidStateTransition;
      return;
    }
    state_ = weld_executor::ExecutionState::kPaused;
    execution_pauses_.fetch_add(1);
    if (!pause_active_) {
      pause_started_ = std::chrono::steady_clock::now();
      pause_active_ = true;
    }
  }

  bool StopAwareSleep(std::stop_token stop_token, std::chrono::milliseconds duration) const {
    constexpr auto kPollInterval = std::chrono::milliseconds(10);
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (!stop_token.stop_requested() && !cancel_requested_.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return true;
      }
      std::this_thread::sleep_for(std::min(
          kPollInterval, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
    }
    return false;
  }

  void FinishInterrupted(const std::shared_ptr<GoalHandleExecuteWeld>& goal_handle,
                         std::chrono::steady_clock::time_point start) {
    SetLastFault(fault_codes::kInterrupted);
    TrySetState(weld_executor::ExecutionState::kCancelled);
    latest_execution_ms_.store(ElapsedMs(start));
    PublishStatus();
    CompleteAborted(goal_handle, fault_codes::kInterrupted, completed_waypoints_.load(),
                    latest_execution_ms_.load());
  }

  void ExecutePlan(std::stop_token stop_token, const weld_interfaces::msg::WeldPlan& plan,
                   std::shared_ptr<GoalHandleExecuteWeld> goal_handle) {
    const auto start = std::chrono::steady_clock::now();
    if (!BeginExecution()) {
      CompleteAborted(goal_handle, fault_codes::kBusy, 0, 0.0);
      return;
    }
    PublishStatus();

    if (!plan.valid || plan.waypoints.empty()) {
      FinishWithFault(goal_handle, fault_codes::kInvalidPlan, start);
      return;
    }

    if (!TrySetState(weld_executor::ExecutionState::kPlanning)) {
      FinishWithFault(goal_handle, LastFault(), start);
      return;
    }
    PublishStatus();
    if (!StopAwareSleep(stop_token, std::chrono::milliseconds(20))) {
      FinishInterrupted(goal_handle, start);
      return;
    }

    if (!TrySetState(weld_executor::ExecutionState::kReady)) {
      FinishWithFault(goal_handle, LastFault(), start);
      return;
    }
    PublishStatus();
    if (!StopAwareSleep(stop_token, std::chrono::milliseconds(20))) {
      FinishInterrupted(goal_handle, start);
      return;
    }

    if (!TrySetState(weld_executor::ExecutionState::kExecuting)) {
      FinishWithFault(goal_handle, LastFault(), start);
      return;
    }
    PublishStatus();

    for (std::size_t i = 0; i < plan.waypoints.size(); ++i) {
      if (stop_token.stop_requested()) {
        FinishInterrupted(goal_handle, start);
        return;
      }
      if (cancel_requested_.load() || (goal_handle && goal_handle->is_canceling())) {
        TrySetState(weld_executor::ExecutionState::kCancelled);
        SetLastFault(fault_codes::kCancelled);
        latest_execution_ms_.store(ElapsedMs(start));
        PublishStatus();
        CompleteCanceled(goal_handle, fault_codes::kCancelled, completed_waypoints_.load(),
                         latest_execution_ms_.load());
        return;
      }

      const auto current_state = CurrentExecutionState();
      if (current_state == weld_executor::ExecutionState::kPaused) {
        latest_execution_ms_.store(ElapsedMs(start));
        PublishStatus();
        CompleteAborted(goal_handle, fault_codes::kPaused, completed_waypoints_.load(),
                        latest_execution_ms_.load());
        return;
      }
      if (current_state == weld_executor::ExecutionState::kFaulted) {
        latest_execution_ms_.store(ElapsedMs(start));
        PublishStatus();
        CompleteAborted(goal_handle, LastFault(), completed_waypoints_.load(),
                        latest_execution_ms_.load());
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

      if (!StopAwareSleep(stop_token, std::chrono::milliseconds(waypoint_delay_ms_))) {
        if (stop_token.stop_requested()) {
          FinishInterrupted(goal_handle, start);
          return;
        }
      }
    }

    if (CurrentExecutionState() != weld_executor::ExecutionState::kExecuting ||
        !TrySetState(weld_executor::ExecutionState::kCompleted)) {
      const auto current_fault = LastFault();
      const auto fault = current_fault == fault_codes::kNone
                             ? std::string{fault_codes::kInterrupted}
                             : current_fault;
      CompleteAborted(goal_handle, fault, completed_waypoints_.load(), latest_execution_ms_.load());
      return;
    }
    latest_execution_ms_.store(ElapsedMs(start));
    PublishStatus();

    CompleteSucceeded(goal_handle, completed_waypoints_.load(), latest_execution_ms_.load());
  }

  void FinishWithFault(const std::shared_ptr<GoalHandleExecuteWeld>& goal_handle,
                       std::string_view fault, std::chrono::steady_clock::time_point start) {
    execution_faults_.fetch_add(1);
    SetLastFault(fault);
    TrySetState(weld_executor::ExecutionState::kFaulted);
    latest_execution_ms_.store(ElapsedMs(start));
    PublishStatus();

    CompleteAborted(goal_handle, fault, completed_waypoints_.load(), latest_execution_ms_.load());
  }

  void CompleteSucceeded(const std::shared_ptr<GoalHandleExecuteWeld>& goal_handle,
                         uint32_t completed_waypoints, double execution_time_ms) const {
    if (!goal_handle) {
      return;
    }
    auto result = std::make_shared<ExecuteWeld::Result>();
    result->success = true;
    result->fault_code = fault_codes::kNone;
    result->completed_waypoints = completed_waypoints;
    result->execution_time_ms = execution_time_ms;
    goal_handle->succeed(result);
  }

  void CompleteAborted(const std::shared_ptr<GoalHandleExecuteWeld>& goal_handle,
                       std::string_view fault, uint32_t completed_waypoints,
                       double execution_time_ms) const {
    if (!goal_handle) {
      return;
    }
    auto result = std::make_shared<ExecuteWeld::Result>();
    result->success = false;
    result->fault_code = fault;
    result->completed_waypoints = completed_waypoints;
    result->execution_time_ms = execution_time_ms;
    goal_handle->abort(result);
  }

  void CompleteCanceled(const std::shared_ptr<GoalHandleExecuteWeld>& goal_handle,
                        std::string_view fault, uint32_t completed_waypoints,
                        double execution_time_ms) const {
    if (!goal_handle) {
      return;
    }
    auto result = std::make_shared<ExecuteWeld::Result>();
    result->success = false;
    result->fault_code = fault;
    result->completed_waypoints = completed_waypoints;
    result->execution_time_ms = execution_time_ms;
    goal_handle->canceled(result);
  }

  double ElapsedMs(std::chrono::steady_clock::time_point start) const {
    std::scoped_lock lock(state_mutex_);
    return ElapsedMsUnlocked(start);
  }

  double ElapsedMsUnlocked(std::chrono::steady_clock::time_point start) const {
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
      status.latest_fault = last_fault_.empty() ? fault_codes::kNone : last_fault_;
    }
    status.planning_failures = planning_failures_.load();
    status.execution_faults = execution_faults_.load();
    status.execution_pauses = execution_pauses_.load();
    status.dropped_plans = dropped_plans_.load();
    status.latest_latency_ms = latest_execution_ms_.load();
    status.recovery_time_ms = latest_recovery_ms_.load();
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
  std::atomic<uint32_t> execution_pauses_{0};
  std::atomic<uint32_t> dropped_plans_{0};
  std::atomic<double> latest_execution_ms_{0.0};
  std::atomic<double> latest_recovery_ms_{0.0};
  std::string last_fault_{fault_codes::kNone};
  std::chrono::steady_clock::time_point pause_started_;
  bool pause_active_{false};
  mutable std::mutex job_mutex_;
  std::condition_variable_any job_cv_;
  ExecutionJob pending_job_;
  bool has_pending_job_{false};
  bool worker_busy_{false};
  std::jthread worker_;

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
