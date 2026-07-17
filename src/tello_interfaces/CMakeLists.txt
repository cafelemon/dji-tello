cmake_minimum_required(VERSION 3.8)
project(tello_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlightStatus.msg"
  "msg/TelloTelemetry.msg"
  "msg/TrackingStatus.msg"
  "srv/SetTarget.srv"
  "srv/TelloCommand.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
