#!/usr/bin/env python3
"""Jeden stack: motor + LiDAR wander + IMU + voliteľne kamera; manuál (web/teleop) alebo wander.

Zarovnanie pri jazde rovno (predvolene IMU):
  correction_mode:=imu | optical_flow | none
  Optical flow: correction_mode:=optical_flow use_camera:=true

Prepínanie počas behu (služby):
  ros2 service call /waverower/switch_to_wander std_srvs/srv/Trigger
  ros2 service call /waverower/switch_to_manual std_srvs/srv/Trigger

Spustenie:
  ros2 launch waverower runtime_stack.launch.py
  ros2 launch waverower runtime_stack.launch.py stack_mode:=wander use_lidar:=true
  ros2 launch waverower runtime_stack.launch.py correction_mode:=imu use_camera:=false
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opaque(context, *args, **kwargs):
    mode = LaunchConfiguration("stack_mode").perform(context)
    if mode not in ("manual", "wander"):
        raise RuntimeError("stack_mode musí byť manual alebo wander")

    ctrl = "auto" if mode == "wander" else "manual"
    wand_en = mode == "wander"

    correction_mode = LaunchConfiguration("correction_mode").perform(context)
    if correction_mode not in ("imu", "optical_flow", "none"):
        raise RuntimeError("correction_mode musí byť imu, optical_flow alebo none")

    use_imu_corr = correction_mode == "imu"
    use_flow = correction_mode == "optical_flow"

    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    use_lidar = LaunchConfiguration("use_lidar").perform(context) == "true"
    use_imu = LaunchConfiguration("use_imu").perform(context) == "true"
    use_camera = LaunchConfiguration("use_camera").perform(context) == "true"
    use_web = LaunchConfiguration("use_web").perform(context) == "true"
    use_teleop = LaunchConfiguration("use_teleop").perform(context) == "true"
    use_ball = LaunchConfiguration("use_ball_follow").perform(context) == "true"
    flow_algo = LaunchConfiguration("flow_algo").perform(context)

    try:
        get_package_share_directory("camera_ros")
        have_cam = True
    except PackageNotFoundError:
        have_cam = False

    use_flow_lk = use_flow and flow_algo != "farneback"
    use_flow_fb = use_flow and flow_algo == "farneback"

    motor_twist_topic = (
        "/teleop_cmd_vel_corrected" if use_flow else "/teleop_cmd_vel"
    )

    # Wander + optical flow: wander → /cmd_vel_raw → optical_flow → /cmd_vel → motor
    wander_cmd_topic = "/cmd_vel_raw" if (wand_en and use_flow) else "/cmd_vel"

    invert_ang = mode == "manual"

    motor_params = {
        "control_mode": ctrl,
        "i2c_bus": int(LaunchConfiguration("i2c_bus").perform(context)),
        "i2c_address": int(LaunchConfiguration("i2c_address").perform(context)),
        "manual_twist_topic": motor_twist_topic,
        "cmd_vel_invert_linear": False,
        "cmd_vel_invert_angular": invert_ang,
        "imu_correction": use_imu_corr,
        "imu_yaw_kp": 0.15,
        "imu_yaw_ki": 0.05,
        "imu_yaw_kd": 0.01,
        "imu_yaw_deadband": 0.02,
        "imu_yaw_integral_limit": 0.3,
    }

    cid_raw = LaunchConfiguration("camera_id").perform(context)
    try:
        cam_param = int(cid_raw)
    except ValueError:
        cam_param = cid_raw

    actions = [
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[motor_params],
        ),
        Node(
            package="waverower",
            executable="lidar_wander.py",
            name="lidar_wander_node",
            output="screen",
            parameters=[{
                "enabled": wand_en,
                "threshold_m": float(LaunchConfiguration("threshold_m").perform(context)),
                "forward_speed": float(LaunchConfiguration("forward_speed").perform(context)),
                "turn_speed": float(LaunchConfiguration("turn_speed").perform(context)),
                "lidar_rotation_deg": float(LaunchConfiguration("lidar_rotation_deg").perform(context)),
                "cmd_topic": wander_cmd_topic,
            }],
        ),
        Node(
            package="waverower",
            executable="robot_mode_switch.py",
            name="robot_mode_switch",
            output="screen",
        ),
    ]

    if use_imu:
        actions.extend([
            Node(
                package="waverower",
                executable="imu_serial_fusion_bridge.py",
                name="imu_serial_fusion_bridge",
                output="screen",
                parameters=[{
                    "serial_port": LaunchConfiguration("imu_serial_port").perform(context),
                    "baud_rate": int(LaunchConfiguration("imu_baud_rate").perform(context)),
                    "frame_id": LaunchConfiguration("imu_frame_id").perform(context),
                    "topic": "/imu",
                }],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_imu",
                arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
            ),
        ])

    if use_lidar:
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(ld19_launch)),
        )

    if have_cam and use_camera:
        actions.append(
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera_node",
                namespace="camera",
                output="screen",
                parameters=[{"camera": cam_param}],
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            )
        )
    elif use_camera and not have_cam:
        actions.append(LogInfo(msg="use_camera:=true vyžaduje balík camera_ros."))

    if use_flow_lk:
        flow_params_lk = {
            "correction_gain": 1.5,
            "max_correction": 0.3,
            "forward_threshold": 0.05,
            "steer_deadzone": 0.12,
            "min_features": 15,
        }
        if wand_en:
            flow_params_lk["teleop_topic"] = "/cmd_vel_raw"
            flow_params_lk["output_topic"] = "/cmd_vel"
        actions.append(
            Node(
                package="waverower",
                executable="optical_flow",
                name="optical_flow_node",
                output="screen",
                parameters=[flow_params_lk],
            )
        )
    if use_flow_fb:
        flow_params_fb = {
            "correction_gain": 1.5,
            "max_correction": 0.3,
            "forward_threshold": 0.05,
            "steer_deadzone": 0.12,
        }
        if wand_en:
            flow_params_fb["teleop_topic"] = "/cmd_vel_raw"
            flow_params_fb["output_topic"] = "/cmd_vel"
        actions.append(
            Node(
                package="waverower",
                executable="optical_flow_dense",
                name="optical_flow_node",
                output="screen",
                parameters=[flow_params_fb],
            )
        )
    if use_teleop:
        actions.append(
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                emulate_tty=True,
                remappings=[("cmd_vel", "/teleop_cmd_vel")],
            )
        )
    if use_ball and use_camera and have_cam and mode == "manual":
        actions.append(
            Node(
                package="waverower",
                executable="ball_follower_action.py",
                name="ball_follower",
                output="screen",
                parameters=[{
                    "image_topic": "/camera/image_raw",
                    "cmd_topic": "/cmd_vel",
                    "invert_angular": False,
                }],
            )
        )

    if use_web:
        pkg = get_package_share_directory("waverower")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "web_teleop_ui.launch.py"),
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
        DeclareLaunchArgument("stack_mode", default_value="manual", description="manual | wander (počiatočný režim)"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("camera_id", default_value="0"),
        DeclareLaunchArgument("use_web", default_value="true", description="rosbridge + HTTP :8080"),
        DeclareLaunchArgument(
            "correction_mode",
            default_value="imu",
            description="Zarovnanie: imu | optical_flow | none (predvolene imu).",
        ),
        DeclareLaunchArgument("flow_algo", default_value="lk"),
        DeclareLaunchArgument("use_teleop", default_value="false"),
        DeclareLaunchArgument("use_ball_follow", default_value="false"),
        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument("imu_serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("threshold_m", default_value="0.30"),
        DeclareLaunchArgument("forward_speed", default_value="0.10"),
        DeclareLaunchArgument("turn_speed", default_value="1.80"),
        DeclareLaunchArgument("lidar_rotation_deg", default_value="-90.0"),
        LogInfo(msg=(
            "runtime_stack: /waverower/switch_to_{manual,wander} · web ak use_web:=true"
        )),
        OpaqueFunction(function=_opaque),
    ])
