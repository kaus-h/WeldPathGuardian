FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    clang-format \
    clang-tidy \
    python3-colcon-common-extensions \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-rclcpp-action \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-visualization-msgs \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /weldpath_guardian
COPY . .

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch ./launch/demo.launch.py"]
