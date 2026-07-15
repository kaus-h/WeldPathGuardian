# Performance Results

Measured on 2026-07-14 in WSL Ubuntu 24.04 with ROS 2 Jazzy tooling from the
`ros2-jazzy` micromamba environment. The simulator seed was fixed at `42`.
Results were generated with:

```bash
source install/setup.bash
python scripts/collect_performance.py --output docs/performance-latest.json
```

The collector subscribes to ROS topics and records per-message samples rather
than parsing one monitor log line per second. `perception` and `planning` are
component processing durations emitted by the nodes. `end-to-end` is the
`/weld/plan` message age measured from the original seam observation timestamp.
Path error is computed in the monitor/collector against the simulator reference
curve, not inside planner or executor logic. Raw samples, launch logs,
git/environment metadata, p95, p99, max, standard deviation, jitter, input rate,
maximum messages in an inclusive one-second window, and scenario parameters are
stored in `docs/performance-latest.json`.

| Scenario | Plan samples | Median perception ms | p95/p99 perception ms | Median planning ms | p95/p99 planning ms | Median end-to-end ms | p95/p99 end-to-end ms | Input rate Hz | Max msgs/1s | Jitter ms | Recovery ms | p95 path error m | Last observed state | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| clean | pending refresh | | | | | | | | | | | | | |
| gaussian_noise_0.002 | pending refresh | | | | | | | | | | | | | |
| gaussian_noise_0.006 | pending refresh | | | | | | | | | | | | | |
| gaussian_noise_0.010 | pending refresh | | | | | | | | | | | | | |
| missing_segment | pending refresh | | | | | | | | | | | | | |
| low_confidence_recovery | pending refresh | | | | | | | | | | | | | |

These are soft real-time ROS graph measurements, not hard real-time guarantees.
