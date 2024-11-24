cmake_minimum_required(VERSION 3.12)
project(benchmarking)

# Menemukan dependensi
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)

# Menambahkan direktori header (untuk OpenCV dan OpenVINO)
include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${OpenVINO_INCLUDE_DIRS}
)

# Menambahkan executable
add_executable(benchmarker src/backup.cpp)  # Ubah ke file yang sesuai

# Menyertakan direktori header tambahan untuk OpenCV
target_include_directories(benchmarker
    PUBLIC ${OpenCV_INCLUDE_DIRS}
)

# Menyertakan library
target_link_libraries(benchmarker
    ${OpenCV_LIBS}
    openvino::runtime
)

# Menambahkan dependensi ROS 2 (rclcpp, sensor_msgs, dll.)
ament_target_dependencies(benchmarker
    rclcpp
    sensor_msgs
    cv_bridge
)

# Menambahkan target install
install(TARGETS benchmarker
  DESTINATION lib/${PROJECT_NAME}
)

# Menyelesaikan konfigurasi ROS 2
ament_package()
