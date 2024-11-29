cmake_minimum_required(VERSION 3.5)
project(planning)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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
    rclcpp
    rclcpp_components
    std_msgs
    utfr_msgs
    nav_msgs
    geometry_msgs
    utfr_common
    CGAL
    Boost
    eufs_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


# Center path node
add_library(center_path_library
  src/center_path_node.cpp
  src/acceleration.cpp
  src/skidpad.cpp
  src/autocross_trackdrive.cpp
  src/testing.cpp
  # Add other source files as needed
)
ament_target_dependencies(center_path_library ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(center_path_library
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(center_path src/center_path.cpp)
target_link_libraries(center_path center_path_library)
ament_target_dependencies(center_path ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(center_path
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS center_path_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME})

install(TARGETS center_path
  DESTINATION lib/${PROJECT_NAME})


# Controller Node
add_library(controller_library
  src/controller_node.cpp
  # Add other source files as needed
)
ament_target_dependencies(controller_library ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(controller_library
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(controller src/controller.cpp)
target_link_libraries(controller controller_library)
ament_target_dependencies(controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(controller
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS controller_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

install(TARGETS controller
    DESTINATION lib/${PROJECT_NAME})


## Install Headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

## Install Launch file
install (
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

## Install Config file
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

## Install CSV files
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Define Test Executables
  ament_add_gtest(center_path_test
    test/center_path_test.cpp
  )
  ament_target_dependencies(center_path_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(center_path_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  target_sources(center_path_test PRIVATE 
    src/center_path_node.cpp
    src/acceleration.cpp
    src/skidpad.cpp
    src/autocross_trackdrive.cpp
    src/testing.cpp
  )
  install(TARGETS center_path_test DESTINATION test/${PROJECT_NAME})
  
  ament_add_gtest(controller_test
    test/controller_test.cpp
  )
  ament_target_dependencies(controller_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(controller_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  target_sources(controller_test PRIVATE src/controller_node.cpp)
  install(TARGETS controller_test DESTINATION test/${PROJECT_NAME})

endif()

ament_package()
