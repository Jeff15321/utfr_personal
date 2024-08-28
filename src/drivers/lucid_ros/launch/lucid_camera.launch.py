import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("arena_camera_node"), "config", "config.yaml"
    )

    left_camera = Node(
        package="arena_camera_node",
        executable="start",
        name="left_camera",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    right_camera = Node(
        package="arena_camera_node",
        executable="start",
        name="right_camera",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    ld.add_action(left_camera)
    ld.add_action(right_camera)
    return ld
