cmake_minimum_required(VERSION 3.8)
project(intuitive_hri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Handstatus.msg"
  "msg/JointGroupCommand.msg"
  "msg/JointGroupSingle.msg"
  "msg/Orientation.msg"
  "msg/Position.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include)

add_executable(leap_publisher src/LeapMotionPublisher.cpp)
ament_target_dependencies(leap_publisher
  rclcpp
  std_msgs
  geometry_msgs
)
install(TARGETS leap_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
