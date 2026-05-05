#!/usr/bin/env python3
# PC: optical flow a/alebo ball_follow cez DDS (rovnaky ROS_DOMAIN_ID ako RPi).
# RPi: kamera ostava na palube; optical_flow v runtime_stack vypni cez offboard_optical_flow:=true.
# Ball follow: na RPi ball_follow_standalone.launch.py offboard_cv:=true (motor + kamera).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opaque(context, *args, **kwargs):
    pkg = get_package_share_directory("waverover")
    params_file = os.path.join(pkg, "config", "ball_follow.yaml")

    en_flow = LaunchConfiguration("enable_optical_flow").perform(context).lower() in (
        "true", "1", "yes",
    )
    en_ball = LaunchConfiguration("enable_ball_follow").perform(context).lower() in (
        "true", "1", "yes",
    )

    if not en_flow and not en_ball:
        return [
            LogInfo(
                msg=(
                    "[cv_remote_pc] aspon jedno z enable_optical_flow / enable_ball_follow musi byt true."
                )
            ),
        ]

    actions = [
        LogInfo(
            msg=(
                "[cv_remote_pc] ROS_DOMAIN_ID musi sediet s RPi; obraz: "
                f"{LaunchConfiguration('image_topic').perform(context)}"
            )
        ),
    ]

    if en_flow:
        dbg_show = LaunchConfiguration("optical_flow_debug_show").perform(context).lower() == "true"
        dbg_img = LaunchConfiguration("optical_flow_debug_publish_image").perform(context).lower() == "true"
        actions.append(
            Node(
                package="waverover",
                executable="optical_flow",
                name="optical_flow_node",
                output="screen",
                parameters=[{
                    "enabled": True,
                    "correction_gain": float(LaunchConfiguration("optical_flow_correction_gain").perform(context)),
                    "max_correction": float(LaunchConfiguration("optical_flow_max_correction").perform(context)),
                    "forward_threshold": 0.05,
                    "steer_deadzone": 0.12,
                    "min_features": 15,
                    "image_topic": LaunchConfiguration("image_topic").perform(context),
                    "teleop_topic": LaunchConfiguration("teleop_topic").perform(context),
                    "output_topic": LaunchConfiguration("optical_flow_output_topic").perform(context),
                    "debug_show": dbg_show,
                    "debug_publish_image": dbg_img,
                    "debug_window_scale": 2,
                    "debug_window_name": "optical_flow",
                }],
            )
        )

    if en_ball:
        sub_comp = LaunchConfiguration("subscribe_compressed").perform(context).lower() in (
            "true", "1", "yes",
        )
        find_timeout = float(LaunchConfiguration("find_timeout_sec").perform(context))
        actions.extend([
            Node(
                package="waverover",
                executable="action_node.py",
                name="ball_follower",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "image_topic": LaunchConfiguration("image_topic").perform(context),
                        "cmd_topic": LaunchConfiguration("cmd_topic").perform(context),
                        "subscribe_compressed": sub_comp,
                    },
                ],
            ),
            Node(
                package="waverover",
                executable="bt_runner.py",
                name="ball_follow_bt_runner",
                output="screen",
                parameters=[{
                    "find_timeout_sec": find_timeout,
                }],
            ),
        ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="Rovnaky na RPi a PC.",
        ),
        SetEnvironmentVariable(
            name="ROS_DOMAIN_ID",
            value=LaunchConfiguration("ros_domain_id"),
        ),
        DeclareLaunchArgument(
            "enable_optical_flow",
            default_value="true",
            description="Spustit optical_flow uzol na PC.",
        ),
        DeclareLaunchArgument(
            "enable_ball_follow",
            default_value="false",
            description="Spustit ball_follow + behavior tree na PC.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera_node/image_raw/compressed",
            description="JPEG z kamery na RPi (camera_ros).",
        ),
        DeclareLaunchArgument("subscribe_compressed", default_value="true"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("find_timeout_sec", default_value="0.0"),
        DeclareLaunchArgument("teleop_topic", default_value="/teleop_cmd_vel"),
        DeclareLaunchArgument("optical_flow_output_topic", default_value="/teleop_cmd_vel_corrected"),
        DeclareLaunchArgument("optical_flow_correction_gain", default_value="2.4"),
        DeclareLaunchArgument("optical_flow_max_correction", default_value="0.45"),
        DeclareLaunchArgument(
            "optical_flow_debug_show",
            default_value="false",
            description="OpenCV okno na PC (DISPLAY).",
        ),
        DeclareLaunchArgument(
            "optical_flow_debug_publish_image",
            default_value="true",
            description="JPEG vizualizacia na /optical_flow/viz/compressed.",
        ),
        OpaqueFunction(function=_opaque),
    ])
