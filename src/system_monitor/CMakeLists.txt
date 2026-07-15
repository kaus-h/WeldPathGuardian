cmake_minimum_required(VERSION 3.16)
project(system_monitor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(weld_interfaces REQUIRED)

add_executable(system_monitor_node src/system_monitor_node.cpp)
ament_target_dependencies(system_monitor_node builtin_interfaces geometry_msgs rclcpp weld_interfaces)

install(TARGETS system_monitor_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_pytest_test(test_runtime_integration
    test/test_runtime_integration.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    TIMEOUT 90
  )
endif()

ament_package()
