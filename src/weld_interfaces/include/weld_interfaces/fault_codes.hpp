#pragma once

#include <string_view>

namespace weld_interfaces::fault_codes {

enum class FaultCode {
  kNone,
  kInsufficientPoints,
  kStaleObservation,
  kLowConfidence,
  kExcessiveGap,
  kExcessiveCurvature,
  kInvalidGeometry,
  kInvalidPlan,
  kBusy,
  kCancelled,
  kPaused,
  kExecutionFault,
  kInvalidStateTransition,
  kInterrupted,
};

inline constexpr std::string_view kNone{"None"};
inline constexpr std::string_view kInsufficientPoints{"InsufficientPoints"};
inline constexpr std::string_view kStaleObservation{"StaleObservation"};
inline constexpr std::string_view kLowConfidence{"LowConfidence"};
inline constexpr std::string_view kExcessiveGap{"ExcessiveGap"};
inline constexpr std::string_view kExcessiveCurvature{"ExcessiveCurvature"};
inline constexpr std::string_view kInvalidGeometry{"InvalidGeometry"};
inline constexpr std::string_view kInvalidPlan{"InvalidPlan"};
inline constexpr std::string_view kBusy{"Busy"};
inline constexpr std::string_view kCancelled{"Cancelled"};
inline constexpr std::string_view kPaused{"Paused"};
inline constexpr std::string_view kExecutionFault{"ExecutionFault"};
inline constexpr std::string_view kInvalidStateTransition{"InvalidStateTransition"};
inline constexpr std::string_view kInterrupted{"Interrupted"};

inline constexpr std::string_view ToString(FaultCode fault) {
  switch (fault) {
    case FaultCode::kNone:
      return kNone;
    case FaultCode::kInsufficientPoints:
      return kInsufficientPoints;
    case FaultCode::kStaleObservation:
      return kStaleObservation;
    case FaultCode::kLowConfidence:
      return kLowConfidence;
    case FaultCode::kExcessiveGap:
      return kExcessiveGap;
    case FaultCode::kExcessiveCurvature:
      return kExcessiveCurvature;
    case FaultCode::kInvalidGeometry:
      return kInvalidGeometry;
    case FaultCode::kInvalidPlan:
      return kInvalidPlan;
    case FaultCode::kBusy:
      return kBusy;
    case FaultCode::kCancelled:
      return kCancelled;
    case FaultCode::kPaused:
      return kPaused;
    case FaultCode::kExecutionFault:
      return kExecutionFault;
    case FaultCode::kInvalidStateTransition:
      return kInvalidStateTransition;
    case FaultCode::kInterrupted:
      return kInterrupted;
  }
  return kInvalidGeometry;
}

}  // namespace weld_interfaces::fault_codes
