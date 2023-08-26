import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(
        get_package_share_directory("mapping"), "config", "config.yaml"
    )

    preprocess_graph_node = Node(
        package="mapping",
        executable="preprocess_graph",
        name="preprocess_graph_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    build_graph_node = Node(
        package="mapping",
        executable="build_graph",
        name="build_graph_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    ekf_node = Node(
        package="mapping",
        executable="ekf",
        name="ekf_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    knn_node = Node(
        package="mapping",
        executable="knn",
        name="knn_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_path],
    )

    ld.add_action(preprocess_graph_node)
    ld.add_action(build_graph_node)
    ld.add_action(ekf_node)
    ld.add_action(knn_node)
    return ld
