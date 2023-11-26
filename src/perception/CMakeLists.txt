cmake_minimum_required(VERSION 3.5)
project(perception)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(utfr_msgs REQUIRED)
find_package(utfr_common REQUIRED)
find_package(PCL REQUIRED)


include_directories(include)

# Create a library for the clusterer and filter
add_library(${NODE_NAME}_library SHARED src/clusterer.cpp src/filter.cpp src/cone_filter.cpp src/lidar_proc_node.cpp)
target_link_libraries(${NODE_NAME}_library  ${PCL_LIBRARIES})
ament_target_dependencies(${NODE_NAME}_library rclcpp std_msgs geometry_msgs sensor_msgs utfr_msgs utfr_common PCL)

# Create executable for the lidar node and link against the lidar_pipeline library
add_executable(lidar_proc src/lidar_proc.cpp)
target_link_libraries(lidar_proc ${NODE_NAME}_library)
ament_target_dependencies(lidar_proc rclcpp std_msgs geometry_msgs sensor_msgs utfr_msgs utfr_common PCL)

install(TARGETS
  lidar_proc
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${NODE_NAME}_library
  DESTINATION lib)

install(DIRECTORY
  include/
  DESTINATION include)

## Install Launch file
install (
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
install (
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Add your tests here
endif()

ament_package()