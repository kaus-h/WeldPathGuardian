#!/usr/bin/env bash
set -euo pipefail

find src -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0 |
  xargs -0 clang-format --dry-run --Werror

clang-tidy -p build/seam_sensor src/seam_sensor/src/seam_sensor_node.cpp --quiet
clang-tidy -p build/seam_perception \
  src/seam_perception/src/seam_perception_node.cpp \
  src/seam_perception/test/test_perception_utils.cpp \
  --quiet
clang-tidy -p build/weld_planner \
  src/weld_planner/src/weld_planner_node.cpp \
  src/weld_planner/test/test_planner_utils.cpp \
  --quiet
clang-tidy -p build/weld_executor \
  src/weld_executor/src/weld_executor_node.cpp \
  src/weld_executor/test/test_state_machine.cpp \
  --quiet
clang-tidy -p build/system_monitor src/system_monitor/src/system_monitor_node.cpp --quiet
clang-tidy -p build/weld_visualization src/weld_visualization/src/weld_visualization_node.cpp --quiet

