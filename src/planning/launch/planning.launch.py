import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(
        get_package_share_directory("planning"), "config", "config.yaml"
    )

    center_path_node = Node(
        package="planning",
        executable="center_path",
        name="center_path_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    controller_node = Node(
        package="planning",
        executable="controller",
        name="controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    ld.add_action(center_path_node)
    ld.add_action(controller_node)
    return ld
