import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("lidar_pipeline"), "config", "config.yaml"
    )
    with open(config, "r") as f:
        config_params = yaml.safe_load(f)["lidar_pipeline_node"]["ros__parameters"]

    node = Node(
        package="lidar_pipeline",
        executable="lidar_node",
        name="lidar_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_params],
    )

    ld.add_action(node)
    return ld
