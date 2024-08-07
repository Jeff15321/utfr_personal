import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    # Drivers
    cam_dir = get_package_share_directory("arena_camera_node")
    cam_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            cam_dir + "/launch/lucid_camera.launch.py"
        )
    )

    lidar_dir = get_package_share_directory("ouster_ros")
    lidar_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            lidar_dir + "/launch/driver.launch.py"
        ),
        launch_arguments=[("viz", "False")],
    )

    gps_dir = get_package_share_directory("xsens_mti_ros2_driver")
    gps_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            gps_dir + "/launch/xsens_mti_node.launch.py"
        )
    )

    ld.add_action(cam_launch)
    ld.add_action(lidar_launch)
    ld.add_action(gps_launch)

    return ld
