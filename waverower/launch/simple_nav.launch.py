#!/usr/bin/env python3
"""
simple_nav.launch.py – SLAM + IMU, volitelne navigacia k cielu (simple_nav_node).
Nahradzuje slam.launch.py: use_nav:=false = len SLAM mapovanie, use_nav:=true (default) = navigacia.

SLAM lifecycle: default oneskorenie 8 s / 11 s (IMU + LiDAR + uzol musia byt nahore);
  rychly stroj: slam_configure_delay_sec:=2 slam_activate_delay_sec:=4

  ros2 launch waverower simple_nav.launch.py                   # SLAM + navigacia
  ros2 launch waverower simple_nav.launch.py use_nav:=false    # len SLAM mapovanie
  ros2 launch waverower simple_nav.launch.py use_rviz:=true

Ciel: RViz 2D Goal Pose -> /goal_pose (NIE Nav2 panel).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _simple_nav_cmd_vel_odom(context, *args, **kwargs):
    max_lin = float(LaunchConfiguration("teleop_max_linear").perform(context))
    max_lin = max(max_lin, 1e-3)
    scale = 0.4 / max_lin
    return [
        Node(
            package="waverower",
            executable="cmd_vel_odom.py",
            name="cmd_vel_odom",
            output="screen",
            parameters=[{
                "cmd_topic": "/teleop_cmd_vel",
                "cmd_topic_auto": "/cmd_vel",
                "odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": True,
                "timeout_sec": 0.4,
                "update_rate_hz": 30.0,
                "use_imu_yaw": True,
                "imu_topic": "/imu",
                "linear_scale": scale,
                "auto_linear_vel_negate": False,
            }],
        ),
    ]


def _slam_lifecycle_timers(context, *args, **kwargs):
    cfg = float(LaunchConfiguration("slam_configure_delay_sec").perform(context))
    act = float(LaunchConfiguration("slam_activate_delay_sec").perform(context))
    cfg = max(cfg, 1.0)
    act = max(act, cfg + 2.0)
    return [
        TimerAction(
            period=cfg,
            actions=[ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "configure"],
                output="screen",
            )],
        ),
        TimerAction(
            period=act,
            actions=[ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/slam_toolbox", "activate"],
                output="screen",
            )],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("waverower")
    slam_params = os.path.join(pkg, "params", "slam.yaml")
    rviz_config = os.path.join(pkg, "params", "slam.rviz")
    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    waver_sim_share = get_package_share_directory("waver_sim")
    robot_xacro = os.path.join(waver_sim_share, "description", "robot.urdf.xacro")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    imu_port = LaunchConfiguration("imu_serial_port")
    imu_baud = LaunchConfiguration("imu_baud_rate")
    i2c_bus = LaunchConfiguration("i2c_bus")
    i2c_addr = LaunchConfiguration("i2c_address")
    drive_speed = LaunchConfiguration("drive_speed")
    drive_fwd_sc = LaunchConfiguration("drive_forward_scale")
    rotate_done_d = LaunchConfiguration("rotate_done_deg")
    rotate_fast_d = LaunchConfiguration("rotate_fast_deg")
    rotate_om_min = LaunchConfiguration("rotate_omega_min")
    rotate_kp = LaunchConfiguration("rotate_kp")
    steer_dead_deg = LaunchConfiguration("drive_steer_deadband_deg")
    goal_tol = LaunchConfiguration("goal_tolerance_m")
    approach_m = LaunchConfiguration("approach_slowdown_m")
    teleop_max_l = LaunchConfiguration("teleop_max_linear")
    teleop_max_a = LaunchConfiguration("teleop_max_angular")
    use_rviz = LaunchConfiguration("use_rviz")
    use_nav = LaunchConfiguration("use_nav")

    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="/dev/serial/by-id/usb-Arduino_Nano_R4_3501110A36313236694133344B573230-if00",
        ),
        DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument(
            "teleop_max_linear",
            default_value="1.0",
            description="Motor + cmd_vel_odom: linear_scale = 0.4 / hodnota (ako runtime_stack)",
        ),
        DeclareLaunchArgument(
            "teleop_max_angular",
            default_value="2.0",
            description="Motor + simple_nav: max |angular.z| [rad/s] pri 100 % (zhoda s web/manual)",
        ),
        DeclareLaunchArgument(
            "slam_configure_delay_sec",
            default_value="8.0",
            description="Cas od startu po 'lifecycle configure' (async_slam_toolbox musi byt v graf-e)",
        ),
        DeclareLaunchArgument(
            "slam_activate_delay_sec",
            default_value="11.0",
            description="Cas od startu po 'lifecycle activate' (min. o ~2 s viac ako configure)",
        ),

        DeclareLaunchArgument("drive_speed", default_value="0.75"),
        DeclareLaunchArgument("drive_forward_scale", default_value="1.5"),
        DeclareLaunchArgument(
            "rotate_angular_scale",
            default_value="0.8",
            description="ROTATING: |angular.z| = teleop_max_angular * scale (ako 80 % angular v manuale)",
        ),
        DeclareLaunchArgument(
            "rotate_done_deg",
            default_value="10.0",
            description="ROTATING hotovo ak |heading_err| < tol [deg] (IMU; napr. ±10°)",
        ),
        DeclareLaunchArgument(
            "rotate_fast_deg",
            default_value="12.0",
            description="Ak |heading_err| > tolto [deg], plna otacka (nie pomaly P-regulator)",
        ),
        DeclareLaunchArgument("rotate_omega_min", default_value="1.0"),
        DeclareLaunchArgument("rotate_kp", default_value="4.5"),
        DeclareLaunchArgument(
            "rotate_linear_nudge",
            default_value="0.0",
            description="ROTATING: linear.x>0 posuva aj odom/RViz vpred; nechaj 0 pre ciste tocenie na mieste",
        ),
        DeclareLaunchArgument(
            "drive_steer_deadband_deg",
            default_value="10.0",
            description="DRIVING: ak |kurzova chyba| < tol [deg], ide rovno (bez korekcie omega)",
        ),
        DeclareLaunchArgument("goal_tolerance_m", default_value="0.35"),
        DeclareLaunchArgument("approach_slowdown_m", default_value="0.60"),
        DeclareLaunchArgument(
            "use_nav",
            default_value="true",
            description="true = SLAM + simple_nav_node (navigacia k cieľu); false = len SLAM mapovanie",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_xacro]),
                    value_type=str,
                ),
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_imu",
            arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
        ),

        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode": "auto",
                "correction_mode": "imu",
                "i2c_bus": i2c_bus,
                "i2c_address": i2c_addr,
                "pwm_min": 400,
                "pwm_max": 4095,
                "teleop_max_linear": teleop_max_l,
                "teleop_max_angular": teleop_max_a,
                "max_wheel_speed": 0.4,
                "invert_linear": True,
                "cmd_vel_invert_linear": False,
                "cmd_vel_tank_mix": True,
                "wheel_base": 1.0,   # half_b=0.5; pri angular.z=1.6 -> v_l/r=+-0.80 (80% PWM)
                "deadzone": 0.03,
                "smooth_alpha": 0.35,
                "cmd_vel_timeout_ms": 600,
                "imu_correction": True,
                "imu_kp": 0.30,
                "imu_ki": 0.05,
                "imu_kd": 0.01,
                "imu_deadband": 0.02,
                "imu_windup": 0.30,
                "imu_sign": -1.0,
            }],
        ),

        Node(
            package="waverower",
            executable="imu_serial_fusion_bridge.py",
            name="imu_serial_fusion_bridge",
            output="screen",
            parameters=[{
                "serial_port": imu_port,
                "baud_rate": imu_baud,
                "frame_id": "imu_link",
                "topic": "/imu",
            }],
        ),

        OpaqueFunction(function=_simple_nav_cmd_vel_odom),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(ld19_launch)),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        OpaqueFunction(function=_slam_lifecycle_timers),

        Node(
            package="waverower",
            executable="simple_nav_node.py",
            name="simple_nav_node",
            output="screen",
            condition=IfCondition(use_nav),
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
                "loop_hz": 20.0,
                "tf_timeout_sec": 0.15,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
        ),
    ])
