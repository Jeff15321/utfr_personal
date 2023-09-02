cmake_minimum_required(VERSION 3.5)
project(mapping)

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
)

## Set G2O build flags
option(G2O_BUILD_EXAMPLES "No need for examples" OFF)
option(BUILD_UNITTESTS "No need for unit tests" OFF)
option(G2O_BUILD_APPS "No need for apps" OFF)
option(G2O_BUILD_SLAM3D_TYPES "No need for 3D SLAM" OFF)
option(G2O_USE_LOGGING "No need for logging" OFF)

add_subdirectory(external/g2o external/g2o/build)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Libaries and Executables
function(add_node NODE_NAME)
  add_library(${NODE_NAME}_library
    src/${NODE_NAME}_node.cpp
    # Add other source files as needed
  )
  ament_target_dependencies(${NODE_NAME}_library ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(${NODE_NAME}_library
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" DIRECTORY)
  set(OUTPUT_DIR ${PARENT_DIR}/build/mapping/external/g2o/build)

  add_executable(${NODE_NAME} src/${NODE_NAME}_node.cpp)
  target_link_libraries(${NODE_NAME} ${NODE_NAME}_library ${OUTPUT_DIR}/lib/libg2o_core.so)
  target_link_libraries(${NODE_NAME} ${NODE_NAME}_library)
  ament_target_dependencies(${NODE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(${NODE_NAME}
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  install(TARGETS ${NODE_NAME}_library
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include/${PROJECT_NAME})

  install(TARGETS ${NODE_NAME}
      DESTINATION lib/${PROJECT_NAME})
endfunction()

# Add nodes
add_node(compute_graph)
add_node(build_graph)
add_node(ekf)

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

#############
## Testing ##
#############
# Add your testing code here...

ament_package()
