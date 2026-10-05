cmake_minimum_required(VERSION 3.8)
project(actions_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(action_servers src/action_servers.cpp)
ament_target_dependencies(action_servers custom_interfaces rclcpp rclcpp_action)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client custom_interfaces rclcpp rclcpp_action)

install(TARGETS action_servers action_client DESTINATION lib/${PROJECT_NAME})
ament_package()
