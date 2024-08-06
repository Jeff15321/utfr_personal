import json
import yaml
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # TODO - figure out how to read from extrinsics.json or just put it in
    # config.yaml

    config_path = os.path.join(
        get_package_share_directory("perception"), "config", "config.yaml"
    )

    with open(config_path, "r") as file:
        config_param = yaml.safe_load(file)["perception_node"]
    lidar_only_detection = config_param["ros__parameters"]["lidar_only_detection"]

    lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "1.5",
            "--yaw",
            "3.141592",
            "--pitch",
            # "0.122173",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            "baselink",
            "--child-frame-id",
            "lidar",
        ],
    )

    ld.add_action(lidar)
    if lidar_only_detection:
        return ld

    left_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.11787",
            # "-0.09",
            "--y",
            "-0.1151",
            # "0",
            "--z",
            "-0.10212",
            # "1.5",
            "--yaw",
            "2.076",
            # "0.523599",
            "--pitch",
            "0.0",
            # "0.122173",
            "--roll",
            "-1.505",
            # "0.7",
            "--frame-id",
            "baselink",
            "--child-frame-id",
            "left_camera",
        ],
    )

    right_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.11787",
            # "0.09",
            "--y",
            "-0.1151",
            # "0",
            "--z",
            "-0.10212",
            # "1.5",
            "--yaw",
            "0.95",
            # "-0.523599",
            "--pitch",
            "0.0",
            # "0.122173",
            "--roll",
            "-1.573",  # "-4.51",  # 1.505",
            # "0",
            "--frame-id",
            "baselink",
            "--child-frame-id",
            "right_camera",
        ],
    )

    ld.add_action(left_camera)
    ld.add_action(right_camera)

    return ld
