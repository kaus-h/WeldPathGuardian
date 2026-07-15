# Architecture

WeldPath Guardian is organized as a small ROS 2 graph that mirrors a perception-to-execution robotics workflow.

## Data Flow

1. `seam_sensor` publishes `weld_interfaces/msg/SeamObservation` on `/seam/raw`.
2. `seam_perception` validates and smooths observations, then publishes `FilteredSeam` on `/seam/filtered`.
3. `weld_planner` converts the filtered seam into `WeldPlan` waypoints on `/weld/plan`.
4. `weld_executor` accepts plans through the `ExecuteWeld` action and can auto-execute `/weld/plan` for the demo.
5. `system_monitor` subscribes to the graph and reports health/rate/latency metrics.
6. `weld_visualization` publishes RViz status text.

```text
/seam/raw
  seam_sensor
      |
      v
  seam_perception  -> /seam/filtered
      |
      v
  weld_planner     -> /weld/plan
      |
      v
  weld_executor    -> /weld/status + ExecuteWeld action
      |
      v
  system_monitor
```

## Concurrency

The executor node uses separate callback groups for plan input, action handling, and status publishing. Shared state is protected with a mutex, while cancellation and fault flags use atomics. Execution work runs outside the state lock.

Plans are serialized through one managed `std::jthread` worker and a condition-variable mailbox. The node destructor requests worker shutdown, wakes the worker, and joins it while ROS publishers/action resources are still alive.

The launch files use ROS 2 processes, so callbacks across nodes are naturally distributed. The executor itself is written to run correctly under a `MultiThreadedExecutor`.

The concurrency tests exercise simultaneous action goals, cancellation while execution is advancing through waypoints, and external plan faults while an accepted action is active. Every accepted action path resolves through `succeed`, `abort`, or `canceled`. Runtime launch tests exercise clean data, missing-segment faulting, and low-confidence pause/recovery behavior.

## Fault Model

Faults are represented as strings at message boundaries to keep RViz, logging, and CLI inspection simple. C++ nodes use the shared `weld_interfaces::fault_codes::FaultCode` enum and `ToString()` helpers so the string values have one typed source of truth:

- `None`
- `InsufficientPoints`
- `StaleObservation`
- `LowConfidence`
- `ExcessiveGap`
- `ExcessiveCurvature`
- `InvalidGeometry`
- `Cancelled`
- `ExecutionFault`

## Metrics

`FilteredSeam` and `WeldPlan` carry component `processing_latency_ms` values.
`SystemStatus` reports planning failures, execution faults, execution pauses,
recovery time, execution latency, and dropped auto-execution plans.

The planner does not know simulator ground truth. Path error against the clean
simulator reference curve is computed by `system_monitor` and the performance
collector for evaluation only.

The performance collector subscribes to the graph and records per-message
perception latency, planning latency, end-to-end message age, path error,
sample counts, p95/p99/max/stddev, input jitter, mean input rate, maximum
messages in an inclusive one-second window, seed, git/build metadata, CPU count,
and memory.

The demo auto-execution policy is first-plan-wins while the worker is busy:
new `/weld/plan` messages are dropped rather than queued or used to supersede
the active job, and the executor reports that count as `dropped_plans`.

## QoS And Units

Raw seam observations use sensor-data QoS. Filtered seams, weld plans, execution
status, and RViz marker topics use reliable keep-last QoS because those outputs
represent decisions or operator-facing state.

Curvature thresholds are expressed as radians per meter and configured through
`max_curvature_rad_per_meter`.
