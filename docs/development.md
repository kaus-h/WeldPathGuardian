# Development Guide

## Git Etiquette

- Use a feature branch for implementation work.
- Keep commits small enough to review independently.
- Prefer imperative commit messages, for example `Add weld planner geometry tests`.
- Do not commit generated `build/`, `install/`, or `log/` directories.
- Run `git status --short --branch` before and after each commit.

## Local Checks

These checks do not require ROS 2:

```bash
git diff --check
python3 -m py_compile launch/demo.launch.py launch/fault_demo.launch.py
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
for path in Path("src").glob("*/package.xml"):
    ET.parse(path)
    print(path)
PY
```

## ROS 2 Checks

Run these on Ubuntu 24.04 with ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## Demo Checklist

1. Launch the default demo with `ros2 launch ./launch/demo.launch.py`.
2. Launch at least one fault scenario with `ros2 launch ./launch/fault_demo.launch.py scenario:=missing_segment`.
3. Confirm `/seam/raw`, `/seam/filtered`, `/weld/plan`, and `/weld/status` publish messages.
4. Confirm RViz marker topics render raw points, filtered line, waypoint arrows, rejected points, and status text.
5. Record measured performance in `docs/performance-results.md` before using claims externally.

