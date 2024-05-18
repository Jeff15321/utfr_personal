import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # DV stack
    mapping_dir = get_package_share_directory("mapping")
    mapping_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            mapping_dir + "/launch/mapping.launch.py"
        )
    )

    planning_dir = get_package_share_directory("planning")
    planning_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            planning_dir + "/launch/planning.launch.py"
        )
    )

    # controls_dir = get_package_share_directory("controls")
    # controls_launch = IncludeLaunchDescription(
    #     launch_description_sources.PythonLaunchDescriptionSource(
    #         controls_dir + "/launch/controls.launch.py"
    #     )
    # )

    # Sim
    bridge_dir = get_package_share_directory("utfr_sim_bridge")
    bridge_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            bridge_dir + "/launch/utfr_sim_bridge.launch.py"
        )
    )

    sim_dir = get_package_share_directory("eufs_launcher")
    sim_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            sim_dir + "/eufs_launcher.launch.py"
        )
    )

    ld.add_action(mapping_launch)
    ld.add_action(planning_launch)
    # ld.add_action(controls_launch)

    ld.add_action(bridge_launch)
    ld.add_action(sim_launch)

    return ld
