cmake_minimum_required(VERSION 3.8)
project(launcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## List all Dependencies:
## Note: no perception and eufs_launcher cause it's in Python
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    arena_camera_node
    xsens_mti_ros2_driver
    ouster_ros
    mapping
    planning
    controls
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