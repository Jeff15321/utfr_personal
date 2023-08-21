cmake_minimum_required(VERSION 3.5)
project(utfr_common)

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
    sensor_msgs
    geometry_msgs
)

## Find Dependencies
foreach(DEPENDENCY IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

## Export Dependencies for downstream packages
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

## Export Targets for downstream packages 
ament_export_targets(common_library_targets HAS_LIBRARY_TARGET)

## Add Libaries
add_library(common_library
   src/math.cpp)

# Set Target Dependencies  
ament_target_dependencies(common_library ${THIS_PACKAGE_INCLUDE_DEPENDS})

## Add Headers to library
target_include_directories(common_library
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
install(TARGETS common_library
    EXPORT common_library_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME})

## Install Executables : None

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ## Define Test Executables
  ament_add_gtest(utfr_common_test
    test/utfr_common_test.cpp
  )
  ament_target_dependencies(utfr_common_test ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ## Find Headers for Testing files
  target_include_directories(utfr_common_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ## Link required files for testing:
  target_sources(utfr_common_test PRIVATE src/math.cpp)

  ## Install testing files
  install(TARGETS utfr_common_test
      DESTINATION test/${PROJECT_NAME})

endif()


ament_package()
