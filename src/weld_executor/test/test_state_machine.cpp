#include "gtest/gtest.h"
#include "weld_executor/state_machine.hpp"

TEST(StateMachine, AllowsNominalExecutionPath) {
  using weld_executor::ExecutionState;

  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kIdle, ExecutionState::kValidating));
  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kValidating, ExecutionState::kPlanning));
  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kPlanning, ExecutionState::kReady));
  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kReady, ExecutionState::kExecuting));
  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kExecuting, ExecutionState::kCompleted));
}

TEST(StateMachine, AllowsFaultFromAnyNonterminalState) {
  using weld_executor::ExecutionState;

  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kIdle, ExecutionState::kFaulted));
  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kExecuting, ExecutionState::kFaulted));
  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kPaused, ExecutionState::kFaulted));
}

TEST(StateMachine, TerminalStatesReturnOnlyToIdle) {
  using weld_executor::ExecutionState;

  EXPECT_TRUE(weld_executor::CanTransition(ExecutionState::kCompleted, ExecutionState::kIdle));
  EXPECT_FALSE(
      weld_executor::CanTransition(ExecutionState::kCompleted, ExecutionState::kExecuting));
}
