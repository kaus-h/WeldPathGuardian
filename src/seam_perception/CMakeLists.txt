cmake_minimum_required(VERSION 3.16)
project(seam_perception)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(weld_interfaces REQUIRED)

add_library(perception_utils INTERFACE)
target_include_directories(perception_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(seam_perception_node src/seam_perception_node.cpp)
target_link_libraries(seam_perception_node perception_utils)
ament_target_dependencies(seam_perception_node
  geometry_msgs
  rclcpp
  std_msgs
  visualization_msgs
  weld_interfaces
)

install(TARGETS seam_perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_perception_utils test/test_perception_utils.cpp)
  target_link_libraries(test_perception_utils perception_utils)
  ament_target_dependencies(test_perception_utils geometry_msgs)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
