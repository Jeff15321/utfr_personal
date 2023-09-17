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
  
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message("x86_64 (64-bit) architecture detected")
    target_link_libraries(${NODE_NAME} ${NODE_NAME}_library ${CMAKE_CURRENT_SOURCE_DIR}/external/x86_lib/libg2o_core.so)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "i86pc")
    message("x86 (32-bit) architecture detected")
    target_link_libraries(${NODE_NAME} ${NODE_NAME}_library ${CMAKE_CURRENT_SOURCE_DIR}/external/x86_lib/libg2o_core.so)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
    message("ARM architecture detected")
    target_link_libraries(${NODE_NAME} ${NODE_NAME}_library ${CMAKE_CURRENT_SOURCE_DIR}/external/arm_lib/libg2o_core.so)
  else()
      message("Unknown or unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()
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
find_package(ament_cmake_gtest REQUIRED)

## Define Test Executables
ament_add_gtest(mapping_test
  test/mapping_test.cpp
)
ament_target_dependencies(mapping_test ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Find Headers for Testing files
target_include_directories(mapping_test
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set(G2O_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/g2o")
target_include_directories(mapping_test PUBLIC /usr/include/eigen3/)
target_include_directories(mapping_test PUBLIC ${G2O_INCLUDE_DIR})

## Link required files for testing:
target_sources(mapping_test PRIVATE src/build_graph_node.cpp)
target_sources(mapping_test PRIVATE src/compute_graph_node.cpp)
target_sources(mapping_test PRIVATE src/ekf_node.cpp)

## Install testing files
install(TARGETS mapping_test
    DESTINATION test/${PROJECT_NAME})

ament_package()
