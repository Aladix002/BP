#!/usr/bin/env python3
"""Motor + LiDAR wander + IMU + volitelne kamera; manual (web/teleop) alebo wander.

correction_mode: imu | optical_flow | none

Sluzby:
  ros2 service call /waverower/switch_to_wander std_srvs/srv/Trigger
  ros2 service call /waverower/switch_to_manual std_srvs/srv/Trigger

Spustenie: ros2 launch waverower runtime_stack.launch.py
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Musi sediet s waverower/web_ui/app.js (WANDER_TURN_RATIO)
WANDER_TURN_RATIO = 18.0

# camera_ros: rovnaké rozlíšenie ako manual_bringup / ball follow
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FORMAT = "XRGB8888"


def _opaque(context, *args, **kwargs):
    mode = LaunchConfiguration("stack_mode").perform(context)
    if mode not in ("manual", "wander"):
        raise RuntimeError("stack_mode musi byt manual alebo wander")

    ctrl = "auto" if mode == "wander" else "manual"
    wand_en = mode == "wander"

    correction_mode = LaunchConfiguration("correction_mode").perform(context)
    if correction_mode not in ("imu", "optical_flow", "none"):
        raise RuntimeError("correction_mode musi byt imu, optical_flow alebo none")

    use_imu_corr = correction_mode == "imu"
    use_flow = correction_mode == "optical_flow"
    flow_algo = LaunchConfiguration("flow_algo").perform(context)
    if flow_algo not in ("lk", "farneback"):
        raise RuntimeError("flow_algo musi byt lk alebo farneback")

    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    use_lidar = LaunchConfiguration("use_lidar").perform(context) == "true"
    use_imu = LaunchConfiguration("use_imu").perform(context) == "true"
    use_camera = LaunchConfiguration("use_camera").perform(context) == "true"
    use_web = LaunchConfiguration("use_web").perform(context) == "true"
    use_teleop = LaunchConfiguration("use_teleop").perform(context) == "true"

    try:
        get_package_share_directory("camera_ros")
        have_cam = True
    except PackageNotFoundError:
        have_cam = False

    wander_cmd_topic = "/cmd_vel"

    invert_ang = mode == "manual"

    max_lin = float(LaunchConfiguration("teleop_max_linear").perform(context))
    max_ang = float(LaunchConfiguration("teleop_max_angular").perform(context))
    w_scale = float(LaunchConfiguration("wander_speed_scale").perform(context))

    auto_wander_fwd = w_scale * max_lin
    auto_wander_turn = auto_wander_fwd * WANDER_TURN_RATIO

    def _wander_speed(name: str, auto_v: float) -> float:
        raw = LaunchConfiguration(name).perform(context).strip().lower()
        if raw in ("auto", "", "-1"):
            return auto_v
        return float(raw)

    wander_fwd = _wander_speed("forward_speed", auto_wander_fwd)
    wander_turn = _wander_speed("turn_speed", auto_wander_turn)

    motor_params = {
        "control_mode":       ctrl,
        "correction_mode":    correction_mode,
        "i2c_bus":            int(LaunchConfiguration("i2c_bus").perform(context)),
        "i2c_address":        int(LaunchConfiguration("i2c_address").perform(context)),
        # PWM rozsah
        "pwm_min":            400,
        "pwm_max":            4095,
        # Teleop normalizacia (zhodne s manual_bringup / web UI fetch)
        "teleop_max_linear":  max_lin,
        "teleop_max_angular": max_ang,
        "invert_linear":      True,
        "smooth_alpha":       0.20,
        # Auto rezim (cmd_vel)
        "max_wheel_speed":    0.4,
        "wheel_base":         0.20,
        "cmd_vel_invert_linear": True,
        "cmd_vel_timeout_ms": 600,
        # IMU korekcia
        "imu_correction":     use_imu_corr,
        "imu_kp":             0.30,
        "imu_ki":             0.05,
        "imu_kd":             0.01,
        "imu_deadband":       0.02,
        "imu_windup":         0.30,
        "imu_sign":           -1.0,
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
                "forward_speed": wander_fwd,
                "turn_speed": wander_turn,
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
                parameters=[{
                    "camera": cam_param,
                    "width": CAMERA_WIDTH,
                    "height": CAMERA_HEIGHT,
                    "format": CAMERA_FORMAT,
                }],
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            )
        )
    elif use_camera and not have_cam:
        actions.append(LogInfo(msg="use_camera:=true vyzaduje balik camera_ros."))

    if have_cam and use_camera:
        if flow_algo == "lk":
            actions.append(
                Node(
                    package="waverower",
                    executable="optical_flow",
                    name="optical_flow_node",
                    output="screen",
                    parameters=[{
                        "enabled": use_flow,
                        "correction_gain": 2.4,
                        "max_correction": 0.45,
                        "forward_threshold": 0.05,
                        "steer_deadzone": 0.12,
                        "min_features": 15,
                        "image_topic": "/camera/camera_node/image_raw/compressed",
                        "teleop_topic": "/teleop_cmd_vel",
                        "output_topic": "/teleop_cmd_vel_corrected",
                    }],
                )
            )
        else:
            actions.append(
                Node(
                    package="waverower",
                    executable="optical_flow_dense",
                    name="optical_flow_node",
                    output="screen",
                    parameters=[{
                        "enabled": use_flow,
                        "correction_gain": 2.4,
                        "max_correction": 0.45,
                        "forward_threshold": 0.05,
                        "steer_deadzone": 0.12,
                        "image_topic": "/camera/camera_node/image_raw/compressed",
                        "teleop_topic": "/teleop_cmd_vel",
                        "output_topic": "/teleop_cmd_vel_corrected",
                    }],
                )
            )
    elif use_flow:
        actions.append(LogInfo(msg="correction_mode:=optical_flow vyzaduje use_camera:=true a camera_ros."))

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
        DeclareLaunchArgument("stack_mode", default_value="manual", description="manual | wander"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("camera_id", default_value="0"),
        DeclareLaunchArgument("use_web", default_value="true", description="rosbridge + HTTP :8080"),
        DeclareLaunchArgument(
            "correction_mode",
            default_value="imu",
            description="zarovnanie: imu | optical_flow | none",
        ),
        DeclareLaunchArgument("flow_algo", default_value="lk"),
        DeclareLaunchArgument("use_teleop", default_value="false"),
        DeclareLaunchArgument("i2c_bus", default_value="1"),
        DeclareLaunchArgument("i2c_address", default_value="64"),
        DeclareLaunchArgument(
            "imu_serial_port",
            default_value="/dev/serial/by-id/usb-Arduino_Nano_R4_3501110A36313236694133344B573230-if00",
            description=(
                "Arduino IMU (115200). Stabilna cesta by-id. "
                "Ak /imu neprudi: ls /dev/serial/by-id/ alebo imu_serial_port:=/dev/ttyACM2"
            ),
        ),
        DeclareLaunchArgument("imu_baud_rate", default_value="115200"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("threshold_m", default_value="0.30"),
        DeclareLaunchArgument(
            "teleop_max_linear",
            default_value="1.0",
            description="Zhoda s manual_bringup motor; web UI berie z /motor_hat_node",
        ),
        DeclareLaunchArgument(
            "teleop_max_angular",
            default_value="2.0",
            description="Zhoda s manual_bringup motor; max |angular.z| pri 100 % skale",
        ),
        DeclareLaunchArgument(
            "wander_speed_scale",
            default_value="0.5",
            description="0..1 ako Speed scale vo web UI; wander forward = scale * teleop_max_linear",
        ),
        DeclareLaunchArgument(
            "forward_speed",
            default_value="auto",
            description="auto = wander_speed_scale * teleop_max_linear; inak cislo [m/s]",
        ),
        DeclareLaunchArgument(
            "turn_speed",
            default_value="auto",
            description="auto = forward * 18 (ako web UI); inak cislo [rad/s]",
        ),
        DeclareLaunchArgument("lidar_rotation_deg", default_value="-90.0"),
        LogInfo(msg="runtime_stack: /waverower/switch_to_{manual,wander}; web ak use_web:=true"),
        OpaqueFunction(function=_opaque),
    ])
