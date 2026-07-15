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
| clean | 92 | 0.008 | 0.021 / 0.082 | 0.008 | 0.010 / 0.036 | 1.530 | 55.242 / 246.110 | 21 | 0.742 | 0.00 | 0.00131 | READY | No faults; one startup tail-latency outlier. |
| gaussian_noise_0.002 | 86 | 0.007 | 0.020 / 0.022 | 0.008 | 0.010 / 0.049 | 1.443 | 1.989 / 3.170 | 21 | 0.375 | 0.00 | 0.00207 | EXECUTING | Low noise, no faults. |
| gaussian_noise_0.006 | 117 | 0.007 | 0.021 / 0.028 | 0.008 | 0.011 / 0.059 | 1.463 | 2.062 / 13.269 | 21 | 0.783 | 0.00 | 0.00394 | EXECUTING | Medium noise, no faults. |
| gaussian_noise_0.010 | 86 | 0.007 | 0.009 / 0.049 | 0.008 | 0.010 / 0.051 | 1.495 | 2.019 / 2.618 | 21 | 1.317 | 0.00 | 0.00619 | EXECUTING | High-noise run retained valid plans with larger path error. |
| missing_segment | 40 | 0.004 | 0.006 / 0.008 | 0.000 | 0.000 / 0.000 | 1.224 | 1.686 / 2.028 | 13 | 0.675 | 0.00 | 0.00000 | FAULTED | Missing seam segment rejected with `ExcessiveGap`. |
| low_confidence_recovery | 164 | 0.007 | 0.010 / 0.023 | 0.007 | 0.010 / 0.011 | 1.318 | 2.151 / 9.405 | 21 | 0.483 | 3499.68 | 0.00263 | EXECUTING | Confidence dip paused execution, then valid data resumed. |

These are soft real-time ROS graph measurements, not hard real-time guarantees.
