#!/usr/bin/env python3
# Na PC v tej istej LAN ako RPi: iba SLAM (+ volitelne RViz). Web teleop vzdy na RPi (runtime_stack use_web).
# Podmienky:
#   - RPi a PC: rovnaky ROS_DOMAIN_ID (default 0).
#   - RPi: runtime_stack ... use_offboard_slam:=true (web zostava default use_web:=true).
#   - Prehliadac: http://<rpi_ip>:8080

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _slam_and_extras(context, *args, **kwargs):
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"

    pkg = get_package_share_directory("waverover")
    slam_params = os.path.join(pkg, "params", "slam.yaml")

    actions = [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-c",
                        "cfg=0; "
                        "for i in $(seq 1 120); do "
                        "if ros2 lifecycle set --no-daemon --spin-time 5 /slam_toolbox configure; then "
                        "echo '[pc_slam_web] slam_toolbox: configure OK'; cfg=1; break; fi; "
                        "sleep 0.25; "
                        "done; "
                        "if [ \"$cfg\" != 1 ]; then "
                        "echo '[pc_slam_web] slam_toolbox: configure FAILED'; exit 1; fi; "
                        "for i in $(seq 1 120); do "
                        "if ros2 lifecycle set --no-daemon --spin-time 5 /slam_toolbox activate; then "
                        "echo '[pc_slam_web] slam_toolbox: activate OK'; exit 0; fi; "
                        "sleep 0.25; "
                        "done; "
                        "echo '[pc_slam_web] slam_toolbox: activate FAILED'; exit 1",
                    ],
                    output="screen",
                )
            ],
        ),
        LogInfo(
            msg=(
                "pc_slam_web: cakam /scan + TF z RPi (odom->base_link). "
                "Teleop UI: na RPi ros2 launch ... runtime_stack (web). "
                "Ak mapa nestiha: firewall multicast (DDS), ROS_DOMAIN_ID."
            )
        ),
    ]

    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(pkg, "params", "slam.rviz")],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="RViz so slam.rviz (na slabom PC daj false)",
        ),
        OpaqueFunction(function=_slam_and_extras),
    ])
