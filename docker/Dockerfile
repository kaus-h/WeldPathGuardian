FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    clang-format \
    clang-tidy \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-geometry-msgs \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /weldpath_guardian
COPY . .

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ./launch/demo.launch.py"]

