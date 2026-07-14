# Performance Results

Record results from the target ROS 2 machine after running the demo scenarios.

| Scenario | Median perception latency ms | p95 perception latency ms | Median planning latency ms | p95 planning latency ms | Recovery time ms | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| clean | TBD | TBD | TBD | TBD | TBD | Measure on Ubuntu 24.04 / ROS 2 Jazzy. |
| gaussian_noise | TBD | TBD | TBD | TBD | TBD | Increase `noise_stddev` between runs. |
| missing_segment | TBD | TBD | TBD | TBD | TBD | Expect planning rejection if gap exceeds threshold. |
| complete_dropout | TBD | TBD | TBD | TBD | TBD | Expect safe fault/no execution. |

Do not use performance numbers on a resume until they have been measured from an actual run.

