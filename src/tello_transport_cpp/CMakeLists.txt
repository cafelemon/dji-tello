cmake_minimum_required(VERSION 3.8)
project(tello_transport_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_interfaces REQUIRED)

add_library(transport_core src/transport_core.cpp)
target_compile_features(transport_core PUBLIC cxx_std_17)
target_include_directories(transport_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(tello_transport_node src/tello_transport_node.cpp)
target_compile_features(tello_transport_node PUBLIC cxx_std_17)
target_link_libraries(tello_transport_node transport_core ${OpenCV_LIBS})
ament_target_dependencies(tello_transport_node
  cv_bridge diagnostic_msgs geometry_msgs rclcpp sensor_msgs std_msgs tello_interfaces)

install(TARGETS transport_core tello_transport_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transport_core test/test_transport_core.cpp)
  target_link_libraries(test_transport_core transport_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(transport_core)
ament_package()
