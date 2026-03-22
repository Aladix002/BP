#!/usr/bin/env python3
"""Spustí waverower v režime auto (cmd_vel → HAT). I2C vlastní len tento proces — nespúšťaj druhý motorový node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="cmd_vel",
                description="Topic s geometry_msgs/Twist (Nav2 controller_server).",
            ),
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("i2c_address", default_value="64"),
            DeclareLaunchArgument("wheel_separation_m", default_value="0.20"),
            DeclareLaunchArgument("max_wheel_linear_m_s", default_value="0.35"),
            DeclareLaunchArgument("cmd_vel_timeout_ms", default_value="250"),
            DeclareLaunchArgument(
                "control_mode",
                default_value="auto",
                description="manual | auto — pre Nav2 nechaj auto.",
            ),
            Node(
                package="waverower",
                executable="waverower",
                name="wasd_motor_hat_node",
                output="screen",
                parameters=[
                    {
                        "control_mode": LaunchConfiguration("control_mode"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "i2c_address": LaunchConfiguration("i2c_address"),
                        "wheel_separation_m": LaunchConfiguration("wheel_separation_m"),
                        "max_wheel_linear_m_s": LaunchConfiguration("max_wheel_linear_m_s"),
                        "cmd_vel_timeout_ms": LaunchConfiguration("cmd_vel_timeout_ms"),
                    }
                ],
            ),
        ]
    )
