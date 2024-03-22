import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(
        get_package_share_directory("perception"), "config", "config.yaml"
    )

    lidar_proc_node = Node(
        package="perception",
        executable="lidar_proc",
        name="lidar_proc_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    ld.add_action(lidar_proc_node)
    return ld
