cmake_minimum_required(VERSION 3.8)
project(xavbot_behaviour)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  behaviortree_cpp_v3
  geometry_msgs
  moveit_msgs
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  xavbot_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## BUILD
add_library(DrivePlatformServer SHARED src/drive_platform.cpp)
ament_target_dependencies(DrivePlatformServer ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(DrivePlatformServer PLUGIN "xavbot_behaviour::DrivePlatformServer" EXECUTABLE "drive_platform_server")

## INSTALL
install(TARGETS DrivePlatformServer
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME})

ament_package()
