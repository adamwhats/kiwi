cmake_minimum_required(VERSION 3.10)
project(xavbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  xavbot_hardware
  SHARED
  src/xavbot_hardware_interface.cpp
  src/motor2040_comms.cpp
)

target_include_directories(
    xavbot_hardware
    PRIVATE
    "include/xavbot_hardware"
)

target_link_libraries(xavbot_hardware PUBLIC serial)

ament_target_dependencies(
  xavbot_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

## INSTALL
install(
        TARGETS xavbot_hardware
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
pluginlib_export_plugin_description_file(hardware_interface xavbot_hardware.xml)

ament_export_include_directories(
  include
)
ament_export_libraries(
  xavbot_hardware
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  libserial-dev
  libserial
  serial
)
ament_package()
