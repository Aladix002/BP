#!/usr/bin/env python3
"""Rosbridge (WebSocket :9090) + statický HTTP server (:8080) pre mobilné web UI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory("waverower"), "web_ui")

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbridge_server"),
                    "launch",
                    "rosbridge_websocket_launch.xml",
                ]
            )
        )
    )

    http_server = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "http.server",
            "8080",
            "--bind",
            "0.0.0.0",
            "--directory",
            web_dir,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("serve_http")),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            DeclareLaunchArgument(
                "serve_http",
                default_value="true",
                description="python3 -m http.server na 0.0.0.0:8080 (web_ui)",
            ),
            LogInfo(
                msg=(
                    "Web teleop: v prehliadači http://<IP-tohoto-stroja>:8080 "
                    "→ Pripojiť k ws://<IP>:9090 · Príkazy na /teleop_cmd_vel · Kamera /camera/camera_node/image_raw/compressed"
                )
            ),
            rosbridge,
            http_server,
        ]
    )
