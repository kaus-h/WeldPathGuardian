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

The launch files use ROS 2 processes, so callbacks across nodes are naturally distributed. The executor itself is written to run correctly under a `MultiThreadedExecutor`.

The concurrency tests exercise simultaneous action goals and cancellation while execution is advancing through waypoints. Runtime launch tests exercise clean data, missing-segment faulting, and low-confidence pause/recovery behavior.

## Fault Model

Faults are represented as strings at message boundaries to keep RViz, logging, and CLI inspection simple:

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
recovery time, execution latency, and path error against the clean reference
seam model used by the simulator.
