# Install script for directory: /home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so.0.1.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so.0.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/daniel/dv24/src/mapping/external/g2o/lib/libg2o_types_slam2d.so.0.1.0"
    "/home/daniel/dv24/src/mapping/external/g2o/lib/libg2o_types_slam2d.so.0.1"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so.0.1.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so.0.1"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/daniel/dv24/src/mapping/external/g2o/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/daniel/dv24/src/mapping/external/g2o/lib/libg2o_types_slam2d.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so"
         OLD_RPATH "/home/daniel/dv24/src/mapping/external/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam2d.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam2d" TYPE FILE FILES
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_pointxy.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_lotsofxy.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_offset.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_pointxy.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_pointxy_bearing.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_pointxy_calib.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_pointxy_offset.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_prior.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_twopointsxy.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_se2_xyprior.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/edge_xy_prior.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/g2o_types_slam2d_api.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/parameter_se2_offset.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/se2.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/types_slam2d.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/vertex_point_xy.h"
    "/home/daniel/dv24/src/mapping/external/g2o/g2o/types/slam2d/vertex_se2.h"
    )
endif()

