#!/usr/bin/env python3
# PC: slam_toolbox + simple_nav_node + RViz (distribuovany SLAM + bodova navigacia).
# RPi: runtime_stack.launch.py use_offboard_slam:=true (rovnaky ROS_DOMAIN_ID; /scan, /imu, /odom, motor).
# Ciel: RViz "2D Goal Pose" -> /goal_pose. Parametre simple_nav zodpovedaju simple_nav.launch.py (use_nav uzol).
# Gainy + cmd_vel_filter_alpha (vyhladenie proti sekaniu od SLAM/TF); loop_hz vyssie ako 20 pre hladsi cmd_vel.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _slam_lifecycle(context, *args, **kwargs):
    delay = float(LaunchConfiguration("slam_configure_delay_sec").perform(context))
    delay = max(delay, 1.0)
    return [
        TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
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
                )
            ],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("waverover")
    slam_params = os.path.join(pkg, "params", "slam.yaml")
    rviz_config = os.path.join(pkg, "params", "slam.rviz")

    teleop_max_a = LaunchConfiguration("teleop_max_angular")
    drive_speed = LaunchConfiguration("drive_speed")
    drive_fwd_sc = LaunchConfiguration("drive_forward_scale")
    rotate_done_d = LaunchConfiguration("rotate_done_deg")
    rotate_fast_d = LaunchConfiguration("rotate_fast_deg")
    rotate_om_min = LaunchConfiguration("rotate_omega_min")
    rotate_kp = LaunchConfiguration("rotate_kp")
    steer_dead_deg = LaunchConfiguration("drive_steer_deadband_deg")
    goal_tol = LaunchConfiguration("goal_tolerance_m")
    approach_m = LaunchConfiguration("approach_slowdown_m")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="Rovnaky na RPi a PC.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="RViz so slam.rviz na PC",
        ),
        DeclareLaunchArgument(
            "use_nav",
            default_value="true",
            description="true = simple_nav_node (rovnaka logika ako simple_nav.launch); false = len SLAM + RViz",
        ),
        DeclareLaunchArgument(
            "slam_configure_delay_sec",
            default_value="3.0",
            description="Oneskorenie pred prvy pokus lifecycle configure/activate (ako simple_nav.launch)",
        ),
        DeclareLaunchArgument(
            "teleop_max_angular",
            default_value="2.0",
            description="Zhoda s motor_hat_node na RPi; simple_nav rotacie = scale * tato hodnota",
        ),
        DeclareLaunchArgument(
            "cmd_vel_angular_gain",
            default_value="10.0",
            description=(
                "cmd_vel.angular.z: (wheel_base 1.0/0.20=5) * 2x rychlost. Uprav podla RPi wheel_base / zelanej otacky."
            ),
        ),
        DeclareLaunchArgument(
            "cmd_vel_linear_gain",
            default_value="2.0",
            description="cmd_vel linear.*: 2x rychlejsia jazda vpred (DRIVING) a rotate_linear_nudge.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_filter_alpha",
            default_value="0.32",
            description=(
                "simple_nav EMA na cmd_vel po gainoch (0.2–0.4 hladke; 1.0 vypnute). Znizuje sekave prikazy pri distribuovanom SLAM."
            ),
        ),
        DeclareLaunchArgument(
            "simple_nav_loop_hz",
            default_value="35.0",
            description="Frekvencia simple_nav slucky (vyssie = hladsi vystup; default 35 vs 20 all-in-one).",
        ),
        DeclareLaunchArgument("drive_speed", default_value="0.75"),
        DeclareLaunchArgument("drive_forward_scale", default_value="1.5"),
        DeclareLaunchArgument(
            "rotate_angular_scale",
            default_value="0.8",
            description="ROTATING: max |omega| = teleop_max_angular * scale (napr. 0.8 = 80 % ako v UI)",
        ),
        DeclareLaunchArgument(
            "rotate_done_deg",
            default_value="7.0",
            description="ROTATING hotovo ak |heading_err| < tol [deg] (IMU), napr. ±7°",
        ),
        DeclareLaunchArgument(
            "rotate_fast_deg",
            default_value="12.0",
            description="Ak |heading_err| > tolto [deg], plna otacka (identicke defaulty ako simple_nav.launch)",
        ),
        DeclareLaunchArgument("rotate_omega_min", default_value="1.0"),
        DeclareLaunchArgument("rotate_kp", default_value="4.5"),
        DeclareLaunchArgument("rotate_linear_nudge", default_value="0.0"),
        DeclareLaunchArgument("drive_steer_deadband_deg", default_value="10.0"),
        DeclareLaunchArgument("goal_tolerance_m", default_value="0.35"),
        DeclareLaunchArgument("approach_slowdown_m", default_value="0.60"),

        SetEnvironmentVariable(
            name="ROS_DOMAIN_ID",
            value=LaunchConfiguration("ros_domain_id"),
        ),
        LogInfo(
            msg=(
                "[slam_remote_pc] RPi: runtime_stack use_offboard_slam:=true. "
                "simple_nav: gainy + cmd_vel_filter_alpha + vyssia loop_hz (proti sekaniu)."
            )
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        OpaqueFunction(function=_slam_lifecycle),
        Node(
            package="waverover",
            executable="simple_nav_node.py",
            name="simple_nav_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_nav")),
            parameters=[{
                "map_frame": "map",
                "base_frame": "base_link",
                "imu_topic": "/imu",
                "teleop_max_angular": teleop_max_a,
                "rotate_angular_scale": LaunchConfiguration("rotate_angular_scale"),
                "rotate_kp": rotate_kp,
                "rotate_done_deg": rotate_done_d,
                "rotate_fast_deg": rotate_fast_d,
                "rotate_omega_min": rotate_om_min,
                "rotate_linear_nudge": LaunchConfiguration("rotate_linear_nudge"),
                "drive_speed": drive_speed,
                "drive_forward_scale": drive_fwd_sc,
                "drive_steer_kp": 0.7,
                "drive_steer_max": 0.25,
                "drive_steer_deadband_deg": steer_dead_deg,
                "rerotate_threshold_rad": 0.52,
                "goal_tolerance_m": goal_tol,
                "approach_slowdown_m": approach_m,
                "loop_hz": LaunchConfiguration("simple_nav_loop_hz"),
                "tf_timeout_sec": 0.15,
                "auto_mode_switch": True,
                "cmd_vel_angular_gain": LaunchConfiguration("cmd_vel_angular_gain"),
                "cmd_vel_linear_gain": LaunchConfiguration("cmd_vel_linear_gain"),
                "cmd_vel_filter_alpha": LaunchConfiguration("cmd_vel_filter_alpha"),
            }],
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
