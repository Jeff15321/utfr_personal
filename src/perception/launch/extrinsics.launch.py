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

    # ROS2 Coordinate system: X (forward), Y (left), Z (up).
    # Yaw rotates around the Z-axis (positive yaw rotates counterclockwise from above).

    lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.172",
            "--y",
            "-0.014",
            "--z",
            "0.826",
            "--frame-id",
            "baselink",
            "--child-frame-id",
            "os_sensor",
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
            "0.056",
            "--y",
            "0.109",
            "--z",
            "0.014",
            "--yaw",
            "0.541052",
            "--frame-id",
            "os_sensor",
            "--child-frame-id",
            "left_camera",
        ],
    )

    right_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.056",
            "--y",
            "-0.109",
            "--z",
            "0.014",
            "--yaw",
            "-0.541052",
            "--frame-id",
            "os_sensor",
            "--child-frame-id",
            "right_camera",
        ],
    )

    ld.add_action(left_camera)
    ld.add_action(right_camera)

    return ld
