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

    # Declare launch arguments for Foxglove Bridge parameters
    ld.add_action(
        DeclareLaunchArgument(
            "num_threads",
            default_value="4",
            description="Number of threads for ROS node executor",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "min_qos_depth", default_value="1", description="Minimum QoS depth"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "max_qos_depth", default_value="25", description="Maximum QoS depth"
        )
    )

    # run nodes except for perception_node for debugging

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
        parameters=[
            {"num_threads": LaunchConfiguration("num_threads")},
            {"min_qos_depth": LaunchConfiguration("min_qos_depth")},
            {"max_qos_depth": LaunchConfiguration("max_qos_depth")},
        ],
    )

    # Play the rosbag file (mcap) located in ssd
    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            os.path.expanduser(
                "/media/utfr-dv/1tb ssd/rosbags/aug22_autoX_sensors_cw/aug22_autoX_sensors_cw.mcap"
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
