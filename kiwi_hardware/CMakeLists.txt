cmake_minimum_required(VERSION 3.10)
project(kiwi_hardware)

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
  kiwi_hardware
  SHARED
  src/kiwi_hardware_interface.cpp
  src/motor2040_comms.cpp
)

target_include_directories(
    kiwi_hardware
    PRIVATE
    "include/kiwi_hardware"
)

target_link_libraries(kiwi_hardware PUBLIC serial)

ament_target_dependencies(
  kiwi_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

## INSTALL
install(
        TARGETS kiwi_hardware
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
pluginlib_export_plugin_description_file(hardware_interface kiwi_hardware.xml)

ament_export_include_directories(
  include
)
ament_export_libraries(
  kiwi_hardware
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  libserial-dev
  libserial
  serial
)
ament_package()
