# WeldPath Guardian

WeldPath Guardian is a C++20/ROS 2 simulation of a fault-aware robotic welding pipeline. It converts noisy 3D seam observations into validated tool paths, executes them through a cancellable state machine, and monitors latency, data quality, and fault recovery. The project focuses on modular robotics architecture, deterministic behavior, concurrency, testing, and reliability under imperfect sensor conditions.

## Architecture

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

RViz markers are published for raw observations, filtered seam estimates, planned waypoints, rejected observations, and current execution status.

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

The tests cover finite-point rejection, smoothing, insufficient geometry, waypoint spacing, curvature/gap validation, and state-machine transitions.

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
- Gap checks run before smoothing so missing seam segments cannot be averaged into apparently valid geometry.
- The executor is soft real-time and latency-monitored; it does not claim hard real-time guarantees.
- The planner rejects suspicious geometry early instead of attempting to repair every malformed path.
- The default demo auto-executes new plans for a compact visual loop, while the `ExecuteWeld` action remains available for explicit long-running execution requests.

## Known Limitations

- No physical robot or industrial welding physics are modeled.
- MoveIt 2 integration is intentionally left as a stretch feature.
- Performance numbers should be generated on the target Ubuntu/ROS 2 machine before using them in a resume or demo.
