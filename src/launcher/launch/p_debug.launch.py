import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # run nodes exepct for perception_node for debugging

    # Launch the visualization system
    visualization_dir = get_package_share_directory("visualization")
    visualization_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            visualization_dir + "/launch/visualization.launch.py"
        )
    )

    # Launch the extrinsics processing system
    extrinsics_dir = get_package_share_directory("perception")
    extrinsics_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            extrinsics_dir + "/launch/extrinsics.launch.py"
        )
    )

    # Launch the lidar processing system
    lidar_proc_dir = get_package_share_directory("perception")
    lidar_proc_launch = IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            lidar_proc_dir + "/launch/lidar_proc.launch.py"
        )
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
    )

    # Play the rosbag file (mcap) located in ssd
    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            os.path.expanduser(
                "/media/utfr-dv/1tb ssd/rosbags/aug22_autoX_sensors_static/aug22_autoX_sensors_static.mcap"
            ),
            "-l",
        ],
        output="screen",
    )

    # Add all actions to the launch description
    ld.add_action(visualization_launch)
    ld.add_action(extrinsics_launch)
    ld.add_action(lidar_proc_launch)
    ld.add_action(foxglove_bridge_node)
    ld.add_action(bag_play)

    return ld
