cmake_minimum_required(VERSION 3.16)
project(weld_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(weld_interfaces REQUIRED)

add_library(planner_utils INTERFACE)
target_include_directories(planner_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(weld_planner_node src/weld_planner_node.cpp)
target_link_libraries(weld_planner_node planner_utils)
ament_target_dependencies(weld_planner_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  weld_interfaces
)

install(TARGETS weld_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_planner_utils test/test_planner_utils.cpp)
  target_link_libraries(test_planner_utils planner_utils)
  ament_target_dependencies(test_planner_utils geometry_msgs tf2 tf2_geometry_msgs)
endif()

ament_package()
