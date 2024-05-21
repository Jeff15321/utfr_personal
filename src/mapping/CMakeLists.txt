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
    tf2_ros
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

## Setup G2O
set(SETUP "${CMAKE_CURRENT_SOURCE_DIR}/external/g2o_config.sh")
execute_process(
    COMMAND sh ${SETUP}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external
)
link_directories(${CMAKE_SOURCE_DIR}/external/g2o/lib)

# Assume your G2O library files are in the following directory
set(G2O_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/g2o/lib")

# Install the libraries
install(DIRECTORY ${G2O_LIB_DIR}/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        FILES_MATCHING PATTERN "*.so*")

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

  set(G2O_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/g2o")
  target_include_directories(${NODE_NAME}_library PUBLIC /usr/include/eigen3/)
  target_include_directories(${NODE_NAME}_library PUBLIC ${G2O_INCLUDE_DIR})
  
  add_executable(${NODE_NAME} src/${NODE_NAME}.cpp)
  target_link_libraries(${NODE_NAME}_library fmt::fmt)
  target_link_libraries(${NODE_NAME}_library g2o_core g2o_stuff g2o_types_data g2o_types_slam2d g2o_solver_eigen g2o_solver_slam2d_linear)
  
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
find_package(ament_cmake_gtest REQUIRED)

## Define Test Executable for mapping_test
ament_add_gtest(mapping_test
  test/mapping_test.cpp
)
ament_target_dependencies(mapping_test ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Define Test Executable for build_graph_tests 
ament_add_gtest(build_graph_tests
  test/build_graph_tests.cpp 
)
ament_target_dependencies(build_graph_tests ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Find Headers for Testing files
target_include_directories(mapping_test
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(build_graph_tests
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set(G2O_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/g2o")
target_include_directories(mapping_test PUBLIC /usr/include/eigen3/)
target_include_directories(mapping_test PUBLIC ${G2O_INCLUDE_DIR})

target_include_directories(build_graph_tests PUBLIC /usr/include/eigen3/)
target_include_directories(build_graph_tests PUBLIC ${G2O_INCLUDE_DIR})

## Link required files for testing:
target_link_libraries(mapping_test fmt::fmt)
target_link_libraries(mapping_test g2o_core g2o_stuff g2o_types_data g2o_types_slam2d)

target_link_libraries(build_graph_tests fmt::fmt)
target_link_libraries(build_graph_tests g2o_core g2o_stuff g2o_types_data g2o_types_slam2d)

## Add source files related to the tests
target_sources(mapping_test PRIVATE src/build_graph_node.cpp)
target_sources(mapping_test PRIVATE src/compute_graph_node.cpp)
target_sources(mapping_test PRIVATE src/ekf_node.cpp)

target_sources(build_graph_tests PRIVATE src/build_graph_node.cpp)
target_sources(build_graph_tests PRIVATE src/compute_graph_node.cpp)
target_sources(build_graph_tests PRIVATE src/ekf_node.cpp)

## Install testing files
install(TARGETS mapping_test build_graph_tests
    DESTINATION test/${PROJECT_NAME})

ament_package()
