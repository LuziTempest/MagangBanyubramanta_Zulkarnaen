# cmake_minimum_required(VERSION 3.8)
# project(pubsub)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(interfaces REQUIRED)

# add_executable(
#   node
#   src/node.cpp
# )
# ament_target_dependencies(node rclcpp sensor_msgs interfaces)

# install(
#   TARGETS node
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()


cmake_minimum_required(VERSION 3.8)
project(stmsubs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Temukan dependensi yang diperlukan
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stmface REQUIRED)

# Tambahkan executable
add_executable(
  node
  src/node.cpp
)

# Tentukan dependencies untuk executable
ament_target_dependencies(node rclcpp sensor_msgs stmface)

# Install executable
install(
  TARGETS node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

