cmake_minimum_required(VERSION 3.5)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## List all Dependencies:
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    ament_cmake_python
    rclcpp
    rclcpp_components
    std_msgs
    utfr_msgs
    sensor_msgs
    geometry_msgs
    utfr_common
    PCL
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Executables
add_executable(perception src/lidar_proc.cpp)
target_link_libraries(perception perception_library)
# Set Target Dependencies  
ament_target_dependencies(perception ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to executables
target_include_directories(perception
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

## Add Libaries
add_library(perception_library
   src/lidar_proc_node.cpp
   src/clusterer.cpp
   src/filter.cpp
   src/cone_filter.cpp)
# Set Target Dependencies  
ament_target_dependencies(perception_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(perception_library
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

#############
## Install ##
#############

## Install Headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

## Install Libraries
install(TARGETS perception_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS perception 
    DESTINATION lib/${PROJECT_NAME})

## Install Launch file
install (
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

## Install Config file
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

ament_package()
