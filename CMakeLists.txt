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
  "msg/handstatus.msg"
  "msg/JointGroupCommand.msg"
  "msg/JointGroupSingle.msg"
  "msg/orientation.msg"
  "msg/up_or_down.msg"
)

ament_package()
