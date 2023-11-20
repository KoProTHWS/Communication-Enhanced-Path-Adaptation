cmake_minimum_required(VERSION 3.8)
project(arc_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  control_msgs
  geometry_msgs
  rclcpp
)

add_executable(servo_keyboard_input src/keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    servo_keyboard_input
  DESTINATION
    lib/${PROJECT_NAME}
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
