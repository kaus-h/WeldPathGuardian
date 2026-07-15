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
python3 -m py_compile scripts/collect_performance.py scripts/generate_demo_gif.py
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
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --event-handlers console_direct+
colcon test-result
```

Run static analysis after generating `compile_commands.json`:

```bash
bash scripts/run_static_checks.sh
```

## Demo Checklist

1. Launch the default demo with `ros2 launch ./launch/demo.launch.py`.
2. Launch at least one fault scenario with `ros2 launch ./launch/fault_demo.launch.py scenario:=missing_segment`.
3. Confirm `/seam/raw`, `/seam/filtered`, `/weld/plan`, and `/weld/status` publish messages.
4. Confirm RViz marker topics render raw points, filtered line, waypoint arrows, rejected points, and status text.
5. Record measured performance in `docs/performance-results.md` before using claims externally.

## Docker And RViz

The Dockerfile is intended to build and run the ROS graph reproducibly. RViz is installed in the image, but GUI use requires host display configuration.

Graph-only smoke run:

```bash
docker build -f docker/Dockerfile -t weldpath-guardian .
docker run --rm weldpath-guardian
```

Linux/X11 RViz run:

```bash
xhost +local:docker
docker run --rm -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  weldpath-guardian \
  bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && rviz2"
```

For Windows/WSL, prefer WSLg or a configured X server and test `rviz2` separately before recording.
