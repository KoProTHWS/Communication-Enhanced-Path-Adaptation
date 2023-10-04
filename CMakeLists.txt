cmake_minimum_required(VERSION 3.8)
project(arc_hybrid_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(arc_interfaces REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_hybrid_planning
  moveit_servo
  moveit_task_constructor_core
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
  arc_interfaces
)

include_directories(include)

add_executable(hybrid_planning_demo_node src/hybrid_planning_demo_node.cpp)
ament_target_dependencies(hybrid_planning_demo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(hybrid_planning_interface_test src/main.cpp src/hybrid_planning_interface.cpp)
ament_target_dependencies(hybrid_planning_interface_test ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(panda_demo_node test/panda_demo_node.cpp)
ament_target_dependencies(panda_demo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    hybrid_planning_demo_node
    hybrid_planning_interface_test
    panda_demo_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
