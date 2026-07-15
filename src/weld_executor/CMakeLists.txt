cmake_minimum_required(VERSION 3.16)
project(weld_executor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(weld_interfaces REQUIRED)

add_library(executor_state_machine INTERFACE)
target_include_directories(executor_state_machine INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(weld_executor_node src/weld_executor_node.cpp)
target_link_libraries(weld_executor_node executor_state_machine)
ament_target_dependencies(weld_executor_node
  rclcpp
  rclcpp_action
  weld_interfaces
)

install(TARGETS weld_executor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine executor_state_machine)
  ament_add_pytest_test(test_executor_action_concurrency
    test/test_executor_action_concurrency.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    TIMEOUT 45
  )
endif()

ament_package()
