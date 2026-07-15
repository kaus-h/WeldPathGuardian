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
| clean | 131 | 0.007 | 0.017 / 0.026 | 0.008 | 0.010 / 0.021 | 1.424 | 1.892 / 2.149 | 20.00 | 21 | 0.103 | 0.00 | 0.00131 | EXECUTING | No faults; input-rate metric is no longer inflated by the inclusive window count. |
| gaussian_noise_0.002 | 129 | 0.007 | 0.008 / 0.009 | 0.008 | 0.010 / 0.022 | 1.484 | 2.383 / 10.658 | 20.03 | 21 | 1.268 | 0.00 | 0.00200 | EXECUTING | Low noise, no faults. |
| gaussian_noise_0.006 | 122 | 0.008 | 0.018 / 0.065 | 0.008 | 0.011 / 0.137 | 1.426 | 2.142 / 15.460 | 20.00 | 21 | 1.453 | 0.00 | 0.00363 | EXECUTING | Medium noise, no faults. |
| gaussian_noise_0.010 | 122 | 0.007 | 0.020 / 0.028 | 0.008 | 0.010 / 0.026 | 1.403 | 2.014 / 2.803 | 20.27 | 22 | 5.364 | 0.00 | 0.00548 | EXECUTING | High-noise run retained valid plans with larger path error. |
| missing_segment | 63 | 0.004 | 0.006 / 0.007 | 0.000 | 0.000 / 0.001 | 1.155 | 77.634 / 325.682 | 12.05 | 13 | 0.113 | 0.00 | 0.00000 | FAULTED | Missing seam segment rejected with `ExcessiveGap`. |
| low_confidence_recovery | 121 | 0.006 | 0.011 / 0.026 | 0.007 | 0.009 / 0.061 | 1.296 | 2.026 / 5.661 | 20.28 | 22 | 5.322 | 3750.07 | 0.00263 | EXECUTING | Confidence dip counted as one pause episode, then valid data resumed. |

These are soft real-time ROS graph measurements, not hard real-time guarantees.
