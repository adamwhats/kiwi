cmake_minimum_required(VERSION 3.8)
project(xavbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DETECT PLATFORM
execute_process(
  COMMAND uname -r
  RESULT_VARIABLE UNAME_RESULT
  OUTPUT_VARIABLE UNAME_OUTPUT
  ERROR_VARIABLE UNAME_ERROR
)
if(UNAME_RESULT EQUAL 0 AND UNAME_OUTPUT MATCHES "tegra")
  set(JETSON_PLATFORM TRUE)
else()
  set(JETSON_PLATFORM FALSE)
endif()

# DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  rosidl_default_generators
)

if(JETSON_PLATFORM)
  list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS 
    isaac_ros_apriltag_interfaces
  )
endif()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# BUILD
set(CUSTOM_INTERFACES
  "action/DrivePlatform.action"
)
if(JETSON_PLATFORM)
  list(APPEND CUSTOM_INTERFACES
    "msg/AprilTagDetectionStamped.msg"
    "action/QueryAprilTagLog.action"
  )
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_INTERFACES}
  DEPENDENCIES ${THIS_PACKAGE_INCLUDE_DEPENDS}
)





ament_export_dependencies(rosidl_default_runtime)

ament_package()
