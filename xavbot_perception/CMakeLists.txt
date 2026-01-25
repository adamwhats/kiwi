cmake_minimum_required(VERSION 3.10)
project(xavbot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  isaac_ros_apriltag_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  xavbot_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## BUILD
add_library(AprilTagManager SHARED src/apriltag_manager.cpp)
ament_target_dependencies(AprilTagManager ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(AprilTagManager PLUGIN "xavbot_perception::AprilTagManager" EXECUTABLE "apriltag_manager")

## INSTALL
install(TARGETS AprilTagManager
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
