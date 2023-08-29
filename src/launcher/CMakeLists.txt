cmake_minimum_required(VERSION 3.8)
project(launcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Default to C++14, copied from example
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## List all Dependencies:, copied from example
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    arena_camera_node
    bluespace_ai_xsens_mti_driver
    ros2_ouster
    velodyne
    perception
    mapping
    planning
    controls
    eufs_launcher
    car_interface
)

foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()