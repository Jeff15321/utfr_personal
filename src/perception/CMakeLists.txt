cmake_minimum_required(VERSION 3.5)
project(perception CXX)

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
    Eigen3
    tf2
    tf2_ros
    tf2_sensor_msgs
    cv_bridge
    OpenCV
    tf2_geometry_msgs
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Find OpenCV without CUDA
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui dnn)
message(STATUS "=== OpenCV Information ===")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
message(STATUS "======================")

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# First define all executables
add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node rclcpp)

add_executable(perception_node_cpp src/perception_node.cpp)
target_include_directories(perception_node_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_node_cpp rclcpp sensor_msgs OpenCV cv_bridge utfr_msgs geometry_msgs tf2_geometry_msgs tf2_ros)
target_link_libraries(perception_node_cpp
    perception_library
    ${OpenCV_LIBS}
    ${cv_bridge_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    rclcpp::rclcpp
)

add_executable(lidar_proc src/lidar_proc.cpp)
target_link_libraries(lidar_proc perception_library ${PCL_LIBRARIES})
# Set Target Dependencies  
ament_target_dependencies(lidar_proc ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to executables
target_include_directories(lidar_proc
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

# Add test executable
add_executable(test_dnn src/test_dnn.cpp)
target_link_libraries(test_dnn
    ${OpenCV_LIBS}
)

## Add Libraries
add_library(perception_library
   src/lidar_proc_node.cpp
   src/clusterer.cpp
   src/filter.cpp
   src/cone_filter.cpp
   src/perception_node.cpp
   src/deep.cpp
)
# Set Target Dependencies  
ament_target_dependencies(perception_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(perception_library
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

# Link libraries
target_link_libraries(perception_library
    ${OpenCV_LIBS}
    ${cv_bridge_LIBRARIES}
    rclcpp::rclcpp
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  perception/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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
install(TARGETS 
    lidar_proc
    test_node
    perception_node_cpp
    test_dnn
    DESTINATION lib/${PROJECT_NAME})

## Install Launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

## Install Config file
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

## Add these lines to install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

ament_package()
