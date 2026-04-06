#!/usr/bin/env python3
"""1D Kalman vyhladenie Imu (/imu -> /imu/filtered)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            DeclareLaunchArgument("input_topic", default_value="/imu"),
            DeclareLaunchArgument("output_topic", default_value="/imu/filtered"),
            DeclareLaunchArgument("process_noise_accel", default_value="0.001"),
            DeclareLaunchArgument("process_noise_gyro", default_value="0.000001"),
            DeclareLaunchArgument("measurement_noise_accel", default_value="0.05"),
            DeclareLaunchArgument("measurement_noise_gyro", default_value="0.001"),
            Node(
                package="waverower",
                executable="imu_kalman_filter.py",
                name="imu_kalman_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "process_noise_accel": LaunchConfiguration("process_noise_accel"),
                        "process_noise_gyro": LaunchConfiguration("process_noise_gyro"),
                        "measurement_noise_accel": LaunchConfiguration("measurement_noise_accel"),
                        "measurement_noise_gyro": LaunchConfiguration("measurement_noise_gyro"),
                    }
                ],
            ),
        ]
    )
