#!/usr/bin/env python3
# Minimalny stack len na sledovanie lopty: motor v auto, kamera, FollowBall server, behavior tree runner.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("waverower")
    params_file = os.path.join(pkg, "config", "ball_follow.yaml")
    image_topic = LaunchConfiguration("image_topic").perform(context)
    cmd_topic = LaunchConfiguration("cmd_topic").perform(context)
    color = LaunchConfiguration("ball_color").perform(context)

    return [
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                # auto: motor pocuva /cmd_vel z ball_follower (nie teleop)
                "control_mode":          "auto",
                "cmd_vel_invert_linear": True,
                "invert_linear":         True,
                "wheel_base":            2.0,
                "pwm_min":               400,
                "pwm_max":               4095,
                "smooth_alpha":          0.20,
            }],
        ),
        Node(
            package="camera_ros",
            executable="camera_node",
            namespace="camera",
            name="camera_node",
            output="screen",
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
            ],
            parameters=[{
                "width": 320,
                "height": 240,
                "format": "XRGB8888",
            }],
        ),
        Node(
            package="waverower",
            executable="ball_follower_action.py",
            name="ball_follower",
            output="screen",
            parameters=[
                params_file,
                {
                    "image_topic": image_topic,
                    "cmd_topic": cmd_topic,
                    "ball_color": color,
                },
            ],
        ),
        Node(
            package="waverower",
            executable="ball_follow_bt_runner.py",
            name="ball_follow_bt_runner",
            output="screen",
            parameters=[{
                "ball_color": color,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("ball_color", default_value="orange"),
        OpaqueFunction(function=_setup),
    ])
