# Architecture

WeldPath Guardian is organized as a small ROS 2 graph that mirrors a perception-to-execution robotics workflow.

## Data Flow

1. `seam_sensor` publishes `weld_interfaces/msg/SeamObservation` on `/seam/raw`.
2. `seam_perception` validates and smooths observations, then publishes `FilteredSeam` on `/seam/filtered`.
3. `weld_planner` converts the filtered seam into `WeldPlan` waypoints on `/weld/plan`.
4. `weld_executor` accepts plans through the `ExecuteWeld` action and can auto-execute `/weld/plan` for the demo.
5. `system_monitor` subscribes to the graph and reports health/rate/latency metrics.
6. `weld_visualization` publishes RViz status text.

## Concurrency

The executor node uses separate callback groups for plan input, action handling, and status publishing. Shared state is protected with a mutex, while cancellation and fault flags use atomics. Execution work runs outside the state lock.

The launch files use ROS 2 processes, so callbacks across nodes are naturally distributed. The executor itself is written to run correctly under a `MultiThreadedExecutor`.

## Fault Model

Faults are represented as strings at message boundaries to keep RViz, logging, and CLI inspection simple:

- `None`
- `InsufficientPoints`
- `StaleObservation`
- `LowConfidence`
- `ExcessiveGap`
- `InvalidGeometry`
- `Cancelled`
- `ExecutionFault`

