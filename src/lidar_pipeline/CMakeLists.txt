cmake_minimum_required(VERSION 3.5)
project(lidar_pipeline)

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
find_package(PCL REQUIRED)


include_directories(include)

# Create a library for the clusterer and filter
add_library(lidar_pipeline SHARED src/clusterer.cpp src/filter.cpp src/cone_filter.cpp)
target_link_libraries(lidar_pipeline  ${PCL_LIBRARIES})
ament_target_dependencies(lidar_pipeline rclcpp std_msgs geometry_msgs sensor_msgs utfr_msgs PCL)

# Create executable for the lidar node and link against the lidar_pipeline library
add_executable(lidar_node src/lidar_node.cpp)
target_link_libraries(lidar_node lidar_pipeline)
ament_target_dependencies(lidar_node rclcpp std_msgs geometry_msgs sensor_msgs utfr_msgs PCL)

install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  lidar_pipeline
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