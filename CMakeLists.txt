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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(robotiq_driver REQUIRED)


include_directories(include ${CMAKE_HOME_DIRECTORY}/usr/include include/intuitive_hri)

set(SOURCES
    src/LeapMotionPublisher.cpp        
    src/LeapMotionListener.c
    src/SensorToRobotPosition.cpp     
)

add_executable(LeapMotionPublisher src/LeapMotionPublisher.cpp)
ament_target_dependencies(LeapMotionPublisher
  rclcpp
  std_msgs
  geometry_msgs
)
install(TARGETS LeapMotionPublisher
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(SensorToRobotPosition src/SensorToRobotPosition.cpp)
ament_target_dependencies(SensorToRobotPosition
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS SensorToRobotPosition
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(RobotControlBasic src/RobotControlBasic.cpp)
ament_target_dependencies(RobotControlBasic
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ${dependencies} robotiq_driver
  sensor_msgs
)

install(TARGETS RobotControlBasic
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(RobotControlAdvanced src/RobotControlAdvanced.cpp)
ament_target_dependencies(RobotControlAdvanced
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ${dependencies} robotiq_driver
  sensor_msgs
)

install(TARGETS RobotControlAdvanced
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
