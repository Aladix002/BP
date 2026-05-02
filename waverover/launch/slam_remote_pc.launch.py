#!/usr/bin/env python3
# SLAM na PC v distribuovanom ROS 2: robot (RPi) bezi runtime_stack s use_slam:=false;
# tu len slam_toolbox + volitelne RViz. Rovnaky ROS_DOMAIN_ID a routovatelna siet medzi PC a RPi.
#
# RPi:  ros2 launch waverover runtime_stack.launch.py use_slam:=false ros_domain_id:=42
# PC:   ros2 launch waverover slam_remote_pc.launch.py ros_domain_id:=42 use_rviz:=true
#
# Predpoklad: na RPi uz publikuju /scan, TF (odom->base_link z cmd_vel_odom), static laser/imu.
# Ak discovery nefunguje cez WiFi, nastav CycloneDDS / Fast DDS peers (viz dokumentacia ROS 2).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("waverover")
    slam_params = os.path.join(pkg, "params", "slam.yaml")
    rviz_config = os.path.join(pkg, "params", "slam.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="Musí byť rovnaký na RPi aj PC (napr. 42).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="RViz so slam.rviz na PC",
        ),
        SetEnvironmentVariable(
            name="ROS_DOMAIN_ID",
            value=LaunchConfiguration("ros_domain_id"),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "cfg=0; "
                    "for i in $(seq 1 120); do "
                    "if ros2 lifecycle set --no-daemon --spin-time 5 /slam_toolbox configure; then "
                    "echo '[slam_remote_pc] slam_toolbox: configure OK'; cfg=1; break; fi; "
                    "sleep 0.25; "
                    "done; "
                    "if [ \"$cfg\" != 1 ]; then "
                    "echo '[slam_remote_pc] slam_toolbox: configure FAILED'; exit 1; fi; "
                    "for i in $(seq 1 120); do "
                    "if ros2 lifecycle set --no-daemon --spin-time 5 /slam_toolbox activate; then "
                    "echo '[slam_remote_pc] slam_toolbox: activate OK'; exit 0; fi; "
                    "sleep 0.25; "
                    "done; "
                    "echo '[slam_remote_pc] slam_toolbox: activate FAILED'; exit 1",
                ],
                output="screen",
            )],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", rviz_config],
        ),
    ])
