cmake_minimum_required(VERSION 3.8)
project(pepper_omni)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(joy_to_cmd_vel src/joy_to_cmd_vel.cpp)
target_compile_features(joy_to_cmd_vel PUBLIC cxx_std_17)
ament_target_dependencies(joy_to_cmd_vel rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(omni_driver src/omni_driver.cpp)
target_compile_features(omni_driver PUBLIC cxx_std_17)
ament_target_dependencies(omni_driver rclcpp geometry_msgs std_msgs dynamixel_sdk)

install(TARGETS joy_to_cmd_vel omni_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
