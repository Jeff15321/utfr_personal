cmake_minimum_required(VERSION 3.5)
project(utfr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/Cone.msg"
  "msg/ConeDetections.msg"
  "msg/ConeMap.msg"
  "msg/ControlCmd.msg"
  "msg/EgoState.msg"
  "msg/Heartbeat.msg"
  "msg/ParametricSpline.msg"
  "msg/PerceptionDebug.msg"
  "msg/PlanningDebug.msg"
  "msg/PoseGraph.msg"
  "msg/PoseGraphData.msg"
  "msg/SensorCan.msg"
  "msg/SystemStatus.msg"
  "msg/TargetState.msg"
  "msg/TrajectoryPath.msg"
  "msg/TrajectoryPoint.msg"
  "msg/VelocityProfile.msg"
  "msg/WaypointPath.msg"
  "msg/LapTime.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()