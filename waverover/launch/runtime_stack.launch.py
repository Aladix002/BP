#!/usr/bin/env python3
# Jeden stack: motor + LiDAR wander + IMU + volitelne kamera + web (rosbridge + volitelne WebRTC video).
# Distribuovane: use_offboard_slam:=true na RPi (web + senzory); na PC iba SLAM: pc_slam_web.launch.py (ziaden web teleop na PC).
# Prepinanie: ros2 service call /waverover/switch_to_wander|switch_to_manual std_srvs/srv/Trigger

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Pomer vpred / otacanie pre auto vypocet wander rychlosti; musi sediet s web_ui (WANDER_TURN_RATIO)
WANDER_TURN_RATIO = 18.0

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FORMAT = "XRGB8888"


def _opaque(context, *args, **kwargs):
    # stack_mode: manual = teleop/web na /teleop_cmd_vel; wander = lidar_wander posiela /cmd_vel
    mode = LaunchConfiguration("stack_mode").perform(context)
    if mode not in ("manual", "wander"):
        raise RuntimeError("stack_mode musi byt manual alebo wander")

    ctrl = "auto" if mode == "wander" else "manual"
    wand_en = mode == "wander"

    correction_mode = LaunchConfiguration("correction_mode").perform(context)
    if correction_mode not in ("imu", "optical_flow", "none"):
        raise RuntimeError("correction_mode musi byt imu, optical_flow alebo none")

    use_imu_corr = correction_mode == "imu"
    # optical_flow uzol koriguje teleop len ked je zapnuty enabled a kamera bezi
    use_flow = correction_mode == "optical_flow"

    ldlidar_share = get_package_share_directory("ldlidar_ros2")
    ld19_launch = os.path.join(ldlidar_share, "launch", "ld19.launch.py")

    use_lidar = LaunchConfiguration("use_lidar").perform(context) == "true"
    use_imu = LaunchConfiguration("use_imu").perform(context) == "true"
    use_camera = LaunchConfiguration("use_camera").perform(context) == "true"
    use_web = LaunchConfiguration("use_web").perform(context) == "true"
    use_webrtc_camera = LaunchConfiguration("use_webrtc_camera").perform(context) == "true"
    use_teleop = LaunchConfiguration("use_teleop").perform(context) == "true"
    use_slam = LaunchConfiguration("use_slam").perform(context) == "true"
    use_offboard_slam = LaunchConfiguration("use_offboard_slam").perform(context) == "true"
    run_onboard_slam = use_slam and not use_offboard_slam
    use_ekf = LaunchConfiguration("use_ekf").perform(context) == "true"
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    use_cmd_vel_odom = LaunchConfiguration("use_cmd_vel_odom").perform(context) == "true"

    try:
        get_package_share_directory("camera_ros")
        have_cam = True
    except PackageNotFoundError:
        have_cam = False

    use_robot_model = LaunchConfiguration("use_robot_model").perform(context) == "true"
    try:
        waver_sim_share = get_package_share_directory("waver_sim")
        robot_model_file = LaunchConfiguration("robot_model_file").perform(context).strip()
        if not robot_model_file:
            robot_model_file = os.path.join(waver_sim_share, "description", "robot.urdf.xacro")
        have_waver_sim = True
    except PackageNotFoundError:
        waver_sim_share = ""
        robot_model_file = ""
        have_waver_sim = False

    wander_cmd_topic = "/cmd_vel"

    max_lin = float(LaunchConfiguration("teleop_max_linear").perform(context))
    max_ang = float(LaunchConfiguration("teleop_max_angular").perform(context))
    w_scale = float(LaunchConfiguration("wander_speed_scale").perform(context))

    # Zakladna rychlost wanderu z rovnakej skaly ako UI (slider * max linear)
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
        "pwm_min":            400,
        "pwm_max":            4095,
        "teleop_max_linear":  max_lin,
        "teleop_max_angular": max_ang,
        "invert_linear":      True,
        "smooth_alpha":       0.20,
        "max_wheel_speed":    0.4,
        "wheel_base":         0.20,
        "cmd_vel_invert_linear": False,
        "cmd_vel_timeout_ms": 600,
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
            package="waverover",
            executable="waverover_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[motor_params],
        ),
        Node(
            package="waverover",
            executable="lidar_wander.py",
            name="lidar_wander_node",
            output="screen",
            parameters=[{
                # enabled: v wander rezime True = uzol generuje cmd_vel; v manual False aby nesiel do auta
                "enabled": wand_en,
                "threshold_m": float(LaunchConfiguration("threshold_m").perform(context)),
                "forward_speed": wander_fwd,
                "turn_speed": wander_turn,
                "lidar_rotation_deg": float(LaunchConfiguration("lidar_rotation_deg").perform(context)),
                "cmd_topic": wander_cmd_topic,
                "turn_blocked_deg": 180.0,
                "turn_tolerance_deg": 2.0,
                "lidar_early_exit": False,
                "lidar_early_exit_min_deg": 40.0,
            }],
        ),
        # Sluzby na prepnutie parametrov motor + wander za behu (web vola tie iste sluzby)
        Node(
            package="waverover",
            executable="robot_mode_switch.py",
            name="robot_mode_switch",
            output="screen",
        ),
    ]

    if use_imu:
        actions.extend([
            Node(
                package="waverover",
                executable="imu_serial_fusion_bridge.py",
                name="imu_serial_fusion_bridge",
                output="screen",
                parameters=[{
                    "serial_port": LaunchConfiguration("imu_serial_port").perform(context),
                    "baud_rate": int(LaunchConfiguration("imu_baud_rate").perform(context)),
                    "frame_id": LaunchConfiguration("imu_frame_id").perform(context),
                    "topic": "/imu",
                    # Gyro pre integraciu uhla vo wanderi: bez dodatocneho Kalmana v moste (spracovanie akcelerometra vo firmware ostava).
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
        if use_web and use_webrtc_camera:
            actions.append(
                Node(
                    package="waverover",
                    executable="webrtc_camera_node.py",
                    name="webrtc_camera",
                    output="screen",
                    parameters=[{
                        "image_topic": "/camera/camera_node/image_raw/compressed",
                        "http_port": 8765,
                    }],
                )
            )
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
                    "fps": 20.0,
                }],
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
            )
        )
    elif use_camera and not have_cam:
        actions.append(LogInfo(msg="use_camera:=true vyzaduje balik camera_ros."))

    optical_flow_debug_show = (
        LaunchConfiguration("optical_flow_debug_show").perform(context).lower() == "true"
    )
    optical_flow_debug_publish_image = (
        LaunchConfiguration("optical_flow_debug_publish_image").perform(context).lower() == "true"
    )

    if have_cam and use_camera:
        # LK optical flow: topic musi byt compressed (JPEG z camera_ros), vystup ide do motor uzla ked correction_mode=optical_flow
        actions.append(
            Node(
                package="waverover",
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
                    "debug_show": optical_flow_debug_show,
                    "debug_publish_image": optical_flow_debug_publish_image,
                    "debug_window_scale": 2,
                    "debug_window_name": "optical_flow",
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

    if use_cmd_vel_odom:
        actions.append(
            Node(
                package="waverover",
                executable="cmd_vel_odom.py",
                name="cmd_vel_odom",
                output="screen",
                parameters=[{
                    "cmd_topic":      "/teleop_cmd_vel",
                    "cmd_topic_auto": "/cmd_vel",
                    "odom_topic":     "/odom",
                    "odom_frame":     "odom",
                    "base_frame":     "base_link",
                    "publish_tf":     True,
                    "timeout_sec":    0.4,
                    "update_rate_hz": 30.0,
                    "use_imu_yaw":    True,
                    "imu_topic":      "/imu",
                    # Skutocna rychlost robota = cmd_linear / teleop_max_linear * max_wheel_speed
                    "linear_scale":   0.4 / max_lin,
                }],
            )
        )

    if use_offboard_slam and use_slam:
        actions.append(
            LogInfo(
                msg=(
                    "use_offboard_slam:=true — SLAM na RPi vypnuty. Na PC (len SLAM): "
                    "source install/setup.bash && export ROS_DOMAIN_ID=0 && "
                    "ros2 launch waverover pc_slam_web.launch.py"
                )
            )
        )

    if run_onboard_slam:
        if not use_lidar:
            actions.append(LogInfo(msg="use_slam:=true ale use_lidar:=false - SLAM sa nespusta (chyba /scan)."))
        else:
            pkg = get_package_share_directory("waverover")
            slam_params = os.path.join(pkg, "params", "slam.yaml")
            ekf_params = os.path.join(pkg, "params", "ekf.yaml")
            if not use_ekf and not use_cmd_vel_odom:
                actions.append(
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="odom_to_base_link",
                        output="screen",
                        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
                    )
                )
            if use_ekf:
                actions.append(
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="ekf_filter_node",
                        output="screen",
                        parameters=[ekf_params],
                    )
                )
            actions.extend([
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params],
                ),
                # Dva samostatne ros2 lifecycle casto zlyhaju: (1) daemon vs. novy uzol → "Node not found",
                # (2) activate skor ako configure → "Unknown transition ... available: configure, shutdown".
                # Jeden bash: opakovane configure, potom az activate (--no-daemon = priama discovery).
                TimerAction(
                    period=2.0,
                    actions=[ExecuteProcess(
                        cmd=[
                            "bash",
                            "-c",
                            "cfg=0; "
                            "for i in $(seq 1 120); do "
                            "if ros2 lifecycle set --no-daemon --spin-time 5 /slam_toolbox configure; then "
                            "echo '[runtime_stack] slam_toolbox: configure OK'; cfg=1; break; fi; "
                            "sleep 0.25; "
                            "done; "
                            "if [ \"$cfg\" != 1 ]; then "
                            "echo '[runtime_stack] slam_toolbox: configure FAILED (skusaj colcon build + znovu launch)'; exit 1; fi; "
                            "for i in $(seq 1 120); do "
                            "if ros2 lifecycle set --no-daemon --spin-time 5 /slam_toolbox activate; then "
                            "echo '[runtime_stack] slam_toolbox: activate OK'; exit 0; fi; "
                            "sleep 0.25; "
                            "done; "
                            "echo '[runtime_stack] slam_toolbox: activate FAILED'; exit 1",
                        ],
                        output="screen",
                    )],
                ),
            ])
            if use_rviz:
                actions.append(
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", os.path.join(pkg, "params", "slam.rviz")],
                    )
                )

    if use_robot_model:
        if have_waver_sim:
            robot_description = ParameterValue(
                Command(["xacro ", robot_model_file]),
                value_type=str,
            )
            actions.append(
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[{"robot_description": robot_description}],
                )
            )
        else:
            actions.append(LogInfo(msg="use_robot_model:=true vyzaduje balik waver_sim (robot.urdf.xacro)."))

    if use_web:
        pkg = get_package_share_directory("waverover")
        web_ui = os.path.join(pkg, "web_ui")
        rb_port = int(LaunchConfiguration("rosbridge_port").perform(context))
        http_port = int(LaunchConfiguration("web_http_port").perform(context))
        http_bind = LaunchConfiguration("web_http_bind").perform(context).strip() or "0.0.0.0"
        if not os.path.isdir(web_ui):
            actions.append(
                LogInfo(msg="use_web:=true ale zlozka web_ui chyba v share/waverover — colcon build waverover")
            )
        else:
            http_cmd = f'cd "{web_ui}" && exec python3 -m http.server {http_port} --bind {http_bind}'
            actions.extend([
                Node(
                    package="rosbridge_server",
                    executable="rosbridge_websocket",
                    name="rosbridge_websocket",
                    output="screen",
                    parameters=[{"port": rb_port}],
                ),
                Node(
                    package="rosapi",
                    executable="rosapi_node",
                    name="rosapi",
                    output="screen",
                ),
                ExecuteProcess(cmd=["bash", "-lc", http_cmd], output="screen"),
                LogInfo(
                    msg=(
                        f"web: rosbridge ws://0.0.0.0:{rb_port} | "
                        f"HTTP http://{http_bind}:{http_port}/ (napr. http://<rpi>:{http_port})"
                    )
                ),
            ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
        DeclareLaunchArgument("stack_mode", default_value="manual", description="manual | wander"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("camera_id", default_value="0"),
        DeclareLaunchArgument("use_robot_model", default_value="true", description="robot_state_publisher z waver_sim URDF"),
        DeclareLaunchArgument("robot_model_file", default_value="", description="cesta k URDF/Xacro; prazdne = autodetect z waver_sim"),
        DeclareLaunchArgument("use_web", default_value="true", description="rosbridge + HTTP pre web_ui"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("web_http_port", default_value="8080"),
        DeclareLaunchArgument(
            "web_http_bind",
            default_value="0.0.0.0",
            description="HTTP server pre web_ui; 0.0.0.0 = LAN",
        ),
        DeclareLaunchArgument(
            "use_webrtc_camera",
            default_value="true",
            description="WebRTC video pre web UI (pip: aiortc aiohttp av); false = len rosbridge JPEG",
        ),
        DeclareLaunchArgument(
            "correction_mode",
            default_value="imu",
            description="zarovnanie: imu | optical_flow | none",
        ),
        DeclareLaunchArgument(
            "optical_flow_debug_show",
            default_value="false",
            description="OpenCV okno na stroji kde bezi uzol (DISPLAY); na PC pouzi optical_flow_debug_publish_image",
        ),
        DeclareLaunchArgument(
            "optical_flow_debug_publish_image",
            default_value="true",
            description="JPEG vizualizacia na /optical_flow/viz/compressed — rqt_image_view na PC (rovnaky ROS_DOMAIN_ID)",
        ),
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
        DeclareLaunchArgument(
            "threshold_m",
            default_value="0.30",
            description="LiDAR wander: ak je predok blizsie ako tato vzdialenost [m], zastavi a zacne otacanie",
        ),
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
        DeclareLaunchArgument("use_slam", default_value="true", description="slam_toolbox async (vyzaduje use_lidar:=true)"),
        DeclareLaunchArgument(
            "use_offboard_slam",
            default_value="false",
            description="true = SLAM na PC (pc_slam_web); web teleop len na RPi (runtime_stack use_web); RPi: senzory + motor + /scan + odom TF",
        ),
        DeclareLaunchArgument("use_ekf", default_value="false", description="robot_localization EKF (IMU->odom)"),
        DeclareLaunchArgument("use_rviz", default_value="false", description="RViz2 + slam.rviz"),
        DeclareLaunchArgument("use_cmd_vel_odom", default_value="true", description="odometria z cmd_vel -> odom->base_link TF"),
        LogInfo(msg="runtime_stack: /waverover/switch_to_{manual,wander}; web ak use_web:=true"),
        OpaqueFunction(function=_opaque),
    ])
