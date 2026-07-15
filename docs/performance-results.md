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
curve, not inside planner logic. Raw samples, launch logs, git/environment
metadata, p95, p99, max, standard deviation, jitter, and scenario parameters are
stored in `docs/performance-latest.json`.

| Scenario | Plan samples | Median perception ms | p95/p99 perception ms | Median planning ms | p95/p99 planning ms | Median end-to-end ms | p95/p99 end-to-end ms | Sustained input Hz | Jitter ms | Recovery ms | p95 path error m | Final state | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| clean | 147 | 0.008 | 0.012 / 0.040 | 0.009 | 0.011 / 0.045 | 1.469 | 1.830 / 1.991 | 21 | 0.166 | 0.00 | 0.00131 | EXECUTING | No faults. |
| gaussian_noise_0.002 | 149 | 0.008 | 0.011 / 0.027 | 0.009 | 0.011 / 0.068 | 1.443 | 2.292 / 6.847 | 21 | 1.787 | 0.00 | 0.00200 | EXECUTING | Low noise, no faults. |
| gaussian_noise_0.006 | 147 | 0.008 | 0.020 / 0.048 | 0.009 | 0.012 / 0.071 | 1.491 | 1.884 / 2.242 | 21 | 0.220 | 0.00 | 0.00365 | EXECUTING | Medium noise, no faults. |
| gaussian_noise_0.010 | 148 | 0.007 | 0.010 / 0.026 | 0.008 | 0.010 / 0.090 | 1.453 | 2.048 / 120.066 | 21 | 1.096 | 0.00 | 0.00548 | EXECUTING | High-noise run retained valid plans with larger path error and one end-to-end latency outlier. |
| missing_segment | 73 | 0.005 | 0.008 / 0.030 | 0.000 | 0.001 / 0.001 | 1.149 | 1.744 / 3.862 | 13 | 0.284 | 0.00 | 0.00000 | FAULTED | Missing seam segment rejected with `ExcessiveGap`. |
| low_confidence_recovery | 186 | 0.007 | 0.010 / 0.023 | 0.008 | 0.010 / 0.048 | 1.368 | 1.709 / 2.091 | 21 | 0.144 | 3499.97 | 0.00263 | EXECUTING | Confidence dip paused execution, then valid data resumed. |

These are soft real-time ROS graph measurements, not hard real-time guarantees.
