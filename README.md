# WeldPath Guardian

WeldPath Guardian is a C++20/ROS 2 simulation of a fault-aware robotic welding pipeline. It converts noisy 3D seam observations into validated tool paths, executes them through a cancellable state machine, and monitors latency, data quality, and fault recovery. The project focuses on modular robotics architecture, deterministic behavior, concurrency, testing, and reliability under imperfect sensor conditions.

## Architecture
RViz markers are published for raw observations, filtered seam estimates, planned waypoints, rejected observations, and current execution status.

![WeldPath Guardian demo](docs/media/weldpath_guardian_demo.gif)
```text
seam_sensor
     |
     | /seam/raw
     v
seam_perception
     |
     | /seam/filtered
     v
weld_planner
     |
     | /weld/plan
     v
weld_executor  <------ ExecuteWeld action
     |
     | /weld/status
     v
system_monitor
```



## Packages

| Package | Purpose |
| --- | --- |
| `weld_interfaces` | Custom messages and the `ExecuteWeld` action. |
| `seam_sensor` | Simulates clean, noisy, stale, missing, offset, and dropout seam observations. |
| `seam_perception` | Rejects invalid data, filters low-confidence points, smooths the seam, and publishes quality metrics. |
| `weld_planner` | Validates geometry, resamples the seam into fixed-spacing tool waypoints, and publishes a structured plan. |
| `weld_executor` | Runs a cancellable, fault-aware execution state machine through a ROS 2 action server. |
| `system_monitor` | Tracks rates, rejected observations, planning failures, execution faults, and latency. |
| `weld_visualization` | Publishes RViz text/status markers for the current system state. |

## Build

Target environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- C++20
- `colcon`

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

## Run The Demo

```bash
ros2 launch ./launch/demo.launch.py
```

Run a fault scenario:

```bash
ros2 launch ./launch/fault_demo.launch.py scenario:=complete_dropout
```

Common scenarios:

- `clean`
- `gaussian_noise`
- `missing_segment`
- `stale_messages`
- `sudden_offset`
- `complete_dropout`
- `low_confidence_recovery`

The demo launch accepts parameter overrides for measured runs:

```bash
ros2 launch ./launch/demo.launch.py scenario:=gaussian_noise noise_stddev:=0.010 dropout_ratio:=0.10
```

Open RViz and add:

- `MarkerArray` on `/seam/raw_markers`
- `MarkerArray` on `/seam/filtered_markers`
- `MarkerArray` on `/weld/plan_markers`
- `MarkerArray` on `/weld/status_markers`

## Test

```bash
source /opt/ros/jazzy/setup.bash
colcon test --event-handlers console_direct+
colcon test-result
```

The tests cover finite-point rejection, local least-squares path fitting, insufficient geometry, waypoint spacing, curvature/gap validation, surface-normal orientation, state-machine transitions, action cancellation/concurrency, and runtime launch behavior for clean, missing-segment, and low-confidence recovery scenarios.

## Performance Snapshot

Measured on 2026-07-14 in WSL Ubuntu 24.04 / ROS 2 Jazzy. Full results and raw samples are in [docs/performance-results.md](docs/performance-results.md) and `docs/performance-latest.json`.

| Scenario | Median perception ms | Median planning ms | Median end-to-end ms | Recovery ms | Max path error m | Final state |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| clean | 0.007 | 0.0115 | 0.87 | 0.00 | 0.00131 | EXECUTING |
| gaussian_noise_0.006 | 0.006 | 0.011 | 0.75 | 0.00 | 0.00425 | EXECUTING |
| missing_segment | 0.004 | 0.000 | 0.83 | 0.00 | 0.00000 | FAULTED |
| low_confidence_recovery | 0.006 | 0.011 | 0.92 | 3499.97 | 0.00223 | COMPLETED |

## Development Standards

This repository is intentionally kept reviewable:

- Work on a feature branch such as `codex/weldpath-guardian`.
- Keep commits scoped to one logical change.
- Run `git diff --check` before committing.
- Run launch-file parsing checks after editing launch files:

```bash
python3 -m py_compile launch/demo.launch.py launch/fault_demo.launch.py
```

- Run the full ROS build and test suite from Ubuntu 24.04 / ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
bash scripts/run_static_checks.sh
colcon test --event-handlers console_direct+
colcon test-result
```

## Design Tradeoffs

- The perception stage uses deterministic filtering and smoothing rather than ML so failures are explainable.
- Local least-squares fitting is used after smoothing to stabilize the published seam path without hiding large gaps.
- Gap checks run before smoothing so missing seam segments cannot be averaged into apparently valid geometry.
- The executor is soft real-time and latency-monitored; it does not claim hard real-time guarantees.
- The planner rejects suspicious geometry early instead of attempting to repair every malformed path.
- Tool orientation is generated from path tangent plus configurable surface normal parameters.
- The default demo auto-executes new plans for a compact visual loop, while the `ExecuteWeld` action remains available for explicit long-running execution requests.

## Known Limitations

- No physical robot or industrial welding physics are modeled.
- MoveIt 2 integration is intentionally left as a stretch feature.
- Performance numbers are from the local WSL ROS 2 Jazzy environment and should be remeasured on any different target machine before publishing final resume numbers.
