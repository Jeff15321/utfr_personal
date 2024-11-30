import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Drivers
    gps_dir = get_package_share_directory("xsens_mti_ros2_driver")
    gps_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            gps_dir + "/launch/xsens_mti_node.launch.py"
        )
    )

    # DV stack
    perception_dir = get_package_share_directory("launcher")
    perception_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            perception_dir + "/launch/perception.launch.py"
        )
    )

    planning_dir = get_package_share_directory("planning")
    planning_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            planning_dir + "/launch/planning.launch.py"
        )
    )
    
    mapping_dir = get_package_share_directory("mapping")
    mapping_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            mapping_dir + "/launch/mapping.launch.py"
        )
    )

    controls_dir = get_package_share_directory("controls")
    controls_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            controls_dir + "/launch/controls.launch.py"
        )
    )

    # ld.add_action(gps_launch)
    ld.add_action(perception_launch)
    ld.add_action(planning_launch)
    ld.add_action(mapping_launch)
    ld.add_action(controls_launch)

    return ld
