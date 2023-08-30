import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    interfance_dir = get_package_share_directory("car_interface")
    interfance_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            interfance_dir + "/launch/car_interface.launch.py"
        )
    )

    ld.add_action(interfance_launch)
    return ld
