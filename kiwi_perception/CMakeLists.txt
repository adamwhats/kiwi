cmake_minimum_required(VERSION 3.10)
project(kiwi_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  visualization_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PCL REQUIRED)

## BUILD
add_library(grasp_planner SHARED src/grasp_planner.cpp)
target_include_directories(grasp_planner PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(grasp_planner ${PCL_LIBRARIES})
ament_target_dependencies(grasp_planner ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(grasp_planner PLUGIN "kiwi_perception::GraspPlanner" EXECUTABLE "grasp_planner_node")

## INSTALL
install(TARGETS grasp_planner
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
