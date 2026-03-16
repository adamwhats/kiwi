cmake_minimum_required(VERSION 3.10)
project(kiwi_interfaces)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  rosidl_default_generators
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# BUILD
set(CUSTOM_INTERFACES
  "action/ArmMoveNamed.action"
  "action/ExecuteTask.action"
  "action/SelectGraspTarget.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${CUSTOM_INTERFACES}
  DEPENDENCIES ${THIS_PACKAGE_INCLUDE_DEPENDS}
)





ament_export_dependencies(rosidl_default_runtime)

ament_package()
