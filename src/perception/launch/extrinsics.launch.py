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

    # Next testing session
    # imu_link: dynamic transform from mti680g relative to world frame
    # os_sensor: static transform from os_sensor relative to imu_link
    # left_camera: static transform from left_camera relative to os_sensor
    # right_camera: static transform from right_camera relative to os_sensor

    # ground = Node(  # Ground plane
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--z",
    #         "0.265",
    #         "--frame-id",
    #         "ground",
    #         "--child-frame-id",
    #         "imu_link",
    #     ],
    # )

    # lidar = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--x",
    #         "-0.172",
    #         "--y",
    #         "-0.014",
    #         "--z",
    #         "0.826",
    #         "--pitch",
    #         "-0.122173",
    #         "--frame-id",
    #         "imu_link",
    #         "--child-frame-id",
    #         "os_sensor",
    #     ],
    # )

    # ld.add_action(ground)
    # # ld.add_action(lidar)
    # if lidar_only_detection:
    #     return ld

    # left_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--x",
    #         "0.056",
    #         "--y",
    #         "0.109",
    #         "--z",
    #         "0.014",
    #         "--yaw",
    #         "0.541052",
    #         "--frame-id",
    #         "os_sensor",
    #         "--child-frame-id",
    #         "left_camera",
    #     ],
    # )

    # right_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--x",
    #         "0.056",
    #         "--y",
    #         "-0.109",
    #         "--z",
    #         "0.014",
    #         "--yaw",
    #         "-0.541052",
    #         "--frame-id",
    #         "os_sensor",
    #         "--child-frame-id",
    #         "right_camera",
    #     ],
    # )

    # Rosbag testing
    # os_lidar: static transform from os_lidar relative to imu_link, this is 180 degrees yawed from os_sensor
    # ground: static transform from ground relative to imu_link
    # left_camera: static transform from left_camera relative to os_lidar
    # right_camera: static transform from right_camera relative to os_lidar

    # Note: lidar frame is set relative to os_lidar in lidar driver. This will change to os_sensor
    # Note: Perception sensors are pitched down 7 degrees according to cad

    lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.172",
            "--y",
            "0.014",
            "--z",
            "-1.092",
            "--pitch",
            # "-0.122173",
            # "-0.139626",
            "-0.15708",
            # "-0.174533",
            # "-0.191986",
            # "0.20944",
            "--frame-id",
            "os_sensor",
            "--child-frame-id",
            "ground",
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
            "0.47",
            "--pitch",
            "-0.015",
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
            "-0.510",
            "--pitch",
            "-0.02",
            "--frame-id",
            "os_sensor",
            "--child-frame-id",
            "right_camera",
        ],
    )

    ld.add_action(left_camera)
    ld.add_action(right_camera)

    return ld
