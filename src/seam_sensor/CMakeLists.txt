cmake_minimum_required(VERSION 3.16)
project(seam_sensor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(weld_interfaces REQUIRED)

add_executable(seam_sensor_node src/seam_sensor_node.cpp)
ament_target_dependencies(seam_sensor_node
  geometry_msgs
  rclcpp
  std_msgs
  visualization_msgs
  weld_interfaces
)

install(TARGETS seam_sensor_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
