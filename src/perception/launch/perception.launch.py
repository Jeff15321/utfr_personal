import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("perception"), "config", "config.yaml"
    )

    node = Node(
        package="perception",
        executable="perception_node.py",
        name="perception_node",
        output="screen",
        emulate_tty=True,
        parameters=[config],
    )

    ld.add_action(node)
    return ld
