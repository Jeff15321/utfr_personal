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

    # DV stack
    perception_dir = get_package_share_directory("perception")
    perception_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            perception_dir + "/launch/perception.launch.py"
        )
    )

    extrinsics_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            perception_dir + "/launch/extrinsics.launch.py"
        )
    )

    lidar_proc_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            perception_dir + "/launch/lidar_proc.launch.py"
        )
    )

    # ld.add_action(cam_launch)
    ld.add_action(lidar_launch)
    ld.add_action(perception_launch)
    ld.add_action(extrinsics_launch)
    ld.add_action(lidar_proc_launch)

    return ld
