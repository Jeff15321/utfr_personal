import json
import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # TODO - figure out how to read from extrinsics.json or just put it in
    # config.yaml

    left_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "2.5",
            "--y",
            "0",
            "--z",
            "1.5",
            "--yaw",
            #"0.523599",
            "1.6",
            "--pitch",
            "0.122173",
            "--roll",
            "0",
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
            "-1.5",
            "--y",
            "0",
            "--z",
            "1.5",
            "--yaw",
            #"-0.523599",
            "-2.25",
            "--pitch",
            "0.122173",
            "--roll",
            "0",
            "--frame-id",
            "baselink",
            "--child-frame-id",
            "right_camera",
        ],
    )

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
            "0",
            "--pitch",
            "0.122173",
            "--roll",
            "0",
            "--frame-id",
            "baselink",
            "--child-frame-id",
            "lidar",
        ],
    )

    ld.add_action(left_camera)
    ld.add_action(right_camera)
    ld.add_action(lidar)
    return ld
