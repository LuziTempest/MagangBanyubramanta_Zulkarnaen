cmake_minimum_required(VERSION 3.5)
project(my_video_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
