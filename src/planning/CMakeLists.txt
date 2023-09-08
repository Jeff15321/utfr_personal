cmake_minimum_required(VERSION 3.5)
project(planning)

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

  add_executable(${NODE_NAME} src/${NODE_NAME}.cpp)
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
add_node(path_optimization)
add_node(controller)
add_node(center_path)

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
# install(
#   DIRECTORY paths
#   DESTINATION share/${PROJECT_NAME})

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
  target_sources(center_path_test PRIVATE src/center_path_node.cpp)
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

  ament_add_gtest(path_optimization_test
    test/path_optimization_test.cpp
  )
  ament_target_dependencies(path_optimization_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_include_directories(path_optimization_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  target_sources(path_optimization_test PRIVATE src/path_optimization_node.cpp)
  install(TARGETS path_optimization_test DESTINATION test/${PROJECT_NAME})
  
endif()

ament_package()
