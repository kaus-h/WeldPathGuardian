# Performance Results

Measured on 2026-07-14 in WSL Ubuntu 24.04 with ROS 2 Jazzy tooling from the
`ros2-jazzy` micromamba environment. Results were generated with:

```bash
source install/setup.bash
python scripts/collect_performance.py --output docs/performance-latest.json
```

`perception` and `planning` are component processing durations emitted by the
nodes. `end-to-end` is the `/weld/plan` message age observed by
`system_monitor`, using the original seam observation timestamp. Raw samples and
logs are stored in `docs/performance-latest.json`.

| Scenario | Median perception ms | p95 perception ms | Median planning ms | p95 planning ms | Median end-to-end ms | p95 end-to-end ms | Sustained input Hz | Recovery ms | Max path error m | Final state | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| clean | 0.007 | 0.008 | 0.0115 | 0.014 | 0.87 | 0.96 | 20 | 0.00 | 0.00131 | EXECUTING | No faults. |
| gaussian_noise_0.002 | 0.007 | 0.008 | 0.010 | 0.170 | 0.80 | 0.95 | 20 | 0.00 | 0.00208 | EXECUTING | Low noise, no faults. |
| gaussian_noise_0.006 | 0.006 | 0.026 | 0.011 | 0.012 | 0.75 | 0.99 | 20 | 0.00 | 0.00425 | EXECUTING | Medium noise, no faults. |
| gaussian_noise_0.010 | 0.006 | 0.008 | 0.009 | 0.011 | 0.75 | 0.83 | 20 | 0.00 | 0.00616 | EXECUTING | High noise caused 12 curvature planning rejections while later valid plans continued. |
| missing_segment | 0.004 | 0.005 | 0.000 | 0.000 | 0.83 | 0.87 | 13 | 0.00 | 0.00000 | FAULTED | Missing seam segment rejected with `ExcessiveGap`. |
| low_confidence_recovery | 0.006 | 0.008 | 0.011 | 0.014 | 0.92 | 1.01 | 20 | 3499.97 | 0.00223 | COMPLETED | Confidence dip paused execution, then valid data resumed and completed. |

These are soft real-time ROS graph measurements, not hard real-time guarantees.
