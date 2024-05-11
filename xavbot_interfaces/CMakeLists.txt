cmake_minimum_required(VERSION 3.8)
project(xavbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  isaac_ros_apriltag_interfaces
  rosidl_default_generators
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## BUILD
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetectionStamped.msg"
  "action/QueryAprilTagLog.action"
  "action/DrivePlatform.action"
  DEPENDENCIES ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
