cmake_minimum_required(VERSION 3.5)
project(controls)

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
    geometry_msgs
    utfr_common
    utfr_msgs
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


## Add Executables
add_executable(controls src/controls.cpp)
target_link_libraries(controls controls_library)
# Set Target Dependencies  
ament_target_dependencies(controls ${THIS_PACKAGE_INCLUDE_DEPENDS})


## Add Headers to executables
target_include_directories(controls
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
      

## Add Libaries
add_library(controls_library
   src/controls_node.cpp
   src/controller/pid_controller.cpp)
# Set Target Dependencies  
ament_target_dependencies(controls_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(controls_library
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
install(TARGETS controls_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables
install(TARGETS controls_library controls
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
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ## Define Test Executables
  ament_add_gtest(controls_unit_tests
    test/controls_unit_tests.cpp
  )
  ament_target_dependencies(controls_unit_tests ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ## Find Headers for Testing files
  target_include_directories(controls_unit_tests
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ## Install testing files
  install(TARGETS controls_unit_tests
      DESTINATION test/${PROJECT_NAME})

endif()


ament_package()
