import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Drivers

    # Done in Perception launch:
    # cam_dir = get_package_share_directory("arena_camera_node")
    # cam_launch = IncludeLaunchDescription(
    #     launch_description_sources.PythonLaunchDescriptionSource(
    #         cam_dir + "/launch/lucid_camera.launch.py"
    #     )
    # )

    gps_dir = get_package_share_directory("bluespace_ai_xsens_mit_driver")
    gps_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            gps_dir + "/launch/xsens_mti_node.launch.py"
        )
    )

    # Done in Perception launch:
    # lidar_dir = get_package_share_directory("ouster_ros")
    # lidar_launch = IncludeLaunchDescription(
    #     launch_description_sources.PythonLaunchDescriptionSource(
    #         lidar_dir + "/launch/driver.launch.py"
    #     )
    # )

    # DV stack
    perception_dir = get_package_share_directory("perception")
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

    controls_dir = get_package_share_directory("controls")
    controls_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            controls_dir + "/launch/controls.launch.py"
        )
    )

    # ld.add_action(cam_launch) done in perception
    ld.add_action(gps_launch)
    # ld.add_action(lidar_launch) done in perception

    ld.add_action(perception_launch)
    ld.add_action(planning_launch)
    ld.add_action(controls_launch)

    return ld
