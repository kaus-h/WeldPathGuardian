cmake_minimum_required(VERSION 3.16)
project(weld_visualization)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(weld_interfaces REQUIRED)

add_executable(weld_visualization_node src/weld_visualization_node.cpp)
ament_target_dependencies(weld_visualization_node
  rclcpp
  visualization_msgs
  weld_interfaces
)

install(TARGETS weld_visualization_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
