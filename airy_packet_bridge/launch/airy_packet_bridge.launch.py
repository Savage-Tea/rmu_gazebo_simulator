# Copyright 2026 SavageTea
#
# Licensed under the Apache License, Version 2.0 (the "License");

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1",
        description="Robot namespace")

    pkg_dir = get_package_share_directory("airy_packet_bridge")

    start_airy_packet_bridge = Node(
        package="airy_packet_bridge",
        executable="airy_packet_bridge_node",
        namespace=namespace,
        output="screen",
        parameters=[os.path.join(pkg_dir, "config", "airy_packet_bridge.yaml")],
    )

    return LaunchDescription([
        declare_namespace,
        start_airy_packet_bridge,
    ])
