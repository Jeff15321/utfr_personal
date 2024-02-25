import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # DV stack

    planning_dir = get_package_share_directory("planning")
    planning_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            planning_dir + "/launch/planning.launch.py"
        )
    )

    controls_dir = get_package_share_directory("controls")
    controls_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            controls_dir + "/launch/controls.launch.py"
        )
    )

    ld.add_action(planning_launch)
    ld.add_action(controls_launch)

    return ld
