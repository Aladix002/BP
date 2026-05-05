#!/usr/bin/env python3
# Motor (auto), kamera, ball_follow, behavior tree.
# offboard_cv:=true — len motor + kamera na RPi; ball_follow na PC (cv_remote_pc.launch.py).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg         = get_package_share_directory("waverover")
    params_file = os.path.join(pkg, "config", "ball_follow.yaml")
    image_topic = LaunchConfiguration("image_topic").perform(context)
    cmd_topic   = LaunchConfiguration("cmd_topic").perform(context)
    find_timeout = float(LaunchConfiguration("find_timeout_sec").perform(context))
    ros_domain = LaunchConfiguration("ros_domain_id").perform(context)
    try:
        cam_id = int(LaunchConfiguration("camera_id").perform(context))
    except ValueError:
        cam_id = 0
    sub_comp = LaunchConfiguration("subscribe_compressed").perform(context).lower() in (
        "true", "1", "yes",
    )
    offboard_cv = LaunchConfiguration("offboard_cv").perform(context).lower() in (
        "true", "1", "yes",
    )

    nodes = [
        Node(
            package="waverover",
            executable="waverover_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode":          "auto",
                "cmd_vel_invert_linear": False,
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
                ("image_raw",    "/camera/image_raw"),
                ("camera_info",  "/camera/camera_info"),
            ],
            parameters=[{
                "camera": cam_id,
                "width":  320,
                "height": 240,
                "format": "XRGB8888",
                "fps":    20.0,
            }],
        ),
    ]

    if offboard_cv:
        nodes.append(
            LogInfo(
                msg=(
                    "offboard_cv:=true — ball_follow na PC: "
                    f"export ROS_DOMAIN_ID={ros_domain} && "
                    "ros2 launch waverover cv_remote_pc.launch.py "
                    "enable_optical_flow:=false enable_ball_follow:=true"
                )
            )
        )
    else:
        nodes.extend([
            Node(
                package="waverover",
                executable="action_node.py",
                name="ball_follower",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "image_topic": image_topic,
                        "cmd_topic":   cmd_topic,
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

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="ROS 2 DDS domain; rovnaky na RPi a PC pri offboard_cv.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera_node/image_raw/compressed",
            description="JPEG: camera_ros; raw Image: nastav subscribe_compressed:=false",
        ),
        DeclareLaunchArgument("subscribe_compressed", default_value="true"),
        DeclareLaunchArgument("camera_id", default_value="0", description="libcamera index pre camera_ros"),
        DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("find_timeout_sec", default_value="0.0"),
        DeclareLaunchArgument(
            "offboard_cv",
            default_value="false",
            description="true = ball_follow + BT len na PC (cv_remote_pc.launch.py)",
        ),
        SetEnvironmentVariable(
            name="ROS_DOMAIN_ID",
            value=LaunchConfiguration("ros_domain_id"),
        ),
        OpaqueFunction(function=_setup),
    ])
