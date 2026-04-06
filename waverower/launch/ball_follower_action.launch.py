#!/usr/bin/env python3
"""Kamera + motor + FollowBall ActionServer (nie priamy ball_follower.py node)."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode": "auto",
                "i2c_bus": 1,
                "i2c_address": 64,
                "cmd_vel_invert_linear": True,
                "cmd_vel_invert_angular": True,
            }],
        ),

        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            parameters=[{
                "camera": 0,
                "format": "BGR888",
                "width": 640,
                "height": 480,
            }],
        ),

        Node(
            package="waverower",
            executable="ball_follower_action.py",
            name="ball_follower",
            output="screen",
            parameters=[{
                "image_topic": "/camera/camera_node/image_raw",
                "cmd_topic": "/cmd_vel",
                "ball_color": "white",
                "angular_kp": 0.5,
                "linear_kp": 0.3,
                "target_radius_px": 80.0,
                "min_radius_px": 15.0,
                "max_radius_px": 100.0,
                "stop_radius_px": 72.0,
                "stop_radius_scale": 1.5,
                "invert_angular": False,
                "mirror_camera_x": False,
                "linear_max": 0.15,
                "angular_max": 1.0,
                "search_speed": 0.6,
                "image_use_best_effort_qos": True,
            }],
        ),
    ])
