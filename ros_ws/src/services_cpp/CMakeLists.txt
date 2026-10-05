cmake_minimum_required(VERSION 3.8)
project(services_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(service_servers src/service_servers.cpp)
ament_target_dependencies(service_servers custom_interfaces geometry_msgs rclcpp)

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client custom_interfaces geometry_msgs rclcpp)

install(TARGETS service_servers service_client DESTINATION lib/${PROJECT_NAME})
ament_package()
