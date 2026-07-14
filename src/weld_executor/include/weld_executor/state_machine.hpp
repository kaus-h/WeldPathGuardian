#pragma once

#include <string_view>

namespace weld_executor {

enum class ExecutionState {
  kIdle,
  kValidating,
  kPlanning,
  kReady,
  kExecuting,
  kPaused,
  kCompleted,
  kFaulted,
  kCancelled,
};

inline const char* ToString(ExecutionState state) {
  switch (state) {
    case ExecutionState::kIdle:
      return "IDLE";
    case ExecutionState::kValidating:
      return "VALIDATING";
    case ExecutionState::kPlanning:
      return "PLANNING";
    case ExecutionState::kReady:
      return "READY";
    case ExecutionState::kExecuting:
      return "EXECUTING";
    case ExecutionState::kPaused:
      return "PAUSED";
    case ExecutionState::kCompleted:
      return "COMPLETED";
    case ExecutionState::kFaulted:
      return "FAULTED";
    case ExecutionState::kCancelled:
      return "CANCELLED";
  }
  return "FAULTED";
}

inline bool IsTerminal(ExecutionState state) {
  return state == ExecutionState::kCompleted || state == ExecutionState::kFaulted ||
         state == ExecutionState::kCancelled;
}

inline bool CanTransition(ExecutionState from, ExecutionState to) {
  if (from == to) {
    return true;
  }
  if (to == ExecutionState::kFaulted || to == ExecutionState::kCancelled) {
    return true;
  }
  if (IsTerminal(from)) {
    return to == ExecutionState::kIdle;
  }

  switch (from) {
    case ExecutionState::kIdle:
      return to == ExecutionState::kValidating;
    case ExecutionState::kValidating:
      return to == ExecutionState::kPlanning;
    case ExecutionState::kPlanning:
      return to == ExecutionState::kReady;
    case ExecutionState::kReady:
      return to == ExecutionState::kExecuting;
    case ExecutionState::kExecuting:
      return to == ExecutionState::kPaused || to == ExecutionState::kCompleted;
    case ExecutionState::kPaused:
      return to == ExecutionState::kExecuting;
    case ExecutionState::kCompleted:
    case ExecutionState::kFaulted:
    case ExecutionState::kCancelled:
      return to == ExecutionState::kIdle;
  }
  return false;
}

}  // namespace weld_executor

