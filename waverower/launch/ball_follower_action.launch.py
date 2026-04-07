#!/usr/bin/env python3
"""Kamera + motor + FollowBall ActionServer (bez BT runnera)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("waverower")
    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    image_topic = LaunchConfiguration("image_topic").perform(context)

    if profile == "daylight":
        profile_file = os.path.join(pkg, "config", "ball_follow_white_daylight.yaml")
    else:
        profile_file = os.path.join(pkg, "config", "ball_follow_white_indoor.yaml")

    return [
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode": "auto",
                "pwm_min":       400,
                "pwm_max":       4095,
                "max_wheel_speed": 0.4,
                "wheel_base":    0.20,
            }],
        ),
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            parameters=[{"width": 800, "height": 600, "format": "XRGB8888"}],
        ),
        Node(
            package="waverower",
            executable="ball_follower_action.py",
            name="ball_follower",
            output="screen",
            parameters=[
                profile_file,
                {
                    "image_topic": image_topic,
                    "cmd_topic": "/cmd_vel",
                    "ball_color": "orange",
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="indoor"),  # indoor|daylight
        DeclareLaunchArgument("image_topic", default_value="/camera/camera_node/image_raw"),
        OpaqueFunction(function=_setup),
    ])
