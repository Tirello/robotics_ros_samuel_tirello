cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "srv/SubtractTwoInts.srv"
  "srv/IsEven.srv"
  "srv/ConvertTemperature.srv"
  "srv/GetCircleArea.srv"
  "srv/ReverseString.srv"
  "srv/DotProduct.srv"
  "srv/ArrayStatistics.srv"
  "srv/LinearRegression.srv"
  "srv/ClosestPoint.srv"
  "srv/InverseKinematics2Link.srv"
  "action/CountUp.action"
  "action/Timer.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
