cmake_minimum_required(VERSION 3.10)
project(kiwi_behaviour)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  behaviortree_cpp_v3
  control_msgs
  geometry_msgs
  kiwi_interfaces
  moveit_msgs
  moveit_ros_planning_interface
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## BUILD
add_library(ArmMoveNamedServer SHARED src/arm_move_named_server.cpp)
ament_target_dependencies(ArmMoveNamedServer ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(ArmMoveNamedServer PLUGIN "kiwi_behaviour::ArmMoveNamedServer" EXECUTABLE "arm_move_named_server")

add_library(TaskExecutor SHARED src/task_executor.cpp)
target_include_directories(TaskExecutor PRIVATE src)
ament_target_dependencies(TaskExecutor ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(TaskExecutor PLUGIN "kiwi_behaviour::TaskExecutor" EXECUTABLE "task_executor")

## INSTALL
install(TARGETS ArmMoveNamedServer TaskExecutor
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/startup_sequence.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
