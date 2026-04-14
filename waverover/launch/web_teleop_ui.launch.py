#!/usr/bin/env python3
# Rosbridge WebSocket (default 9090) pre roslibjs + volitelny HTTP server pre staticke subory web_ui (8080).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Cesta k nainstalovanemu share/waverover/web_ui (index.html, js)
    web_dir = os.path.join(get_package_share_directory("waverover"), "web_ui")

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

    # Jednoduchy static server; pre produkciu by stacil nginx, tu staci na LAN vyvoj
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
                    "Web teleop: http://<IP>:8080  ws://<IP>:9090  /teleop_cmd_vel  kamera .../image_raw/compressed"
                )
            ),
            rosbridge,
            http_server,
        ]
    )
