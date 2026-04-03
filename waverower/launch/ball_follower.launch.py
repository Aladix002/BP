#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),

        # Motor node
        Node(
            package="waverower",
            executable="waverower_motor",
            name="motor_hat_node",
            output="screen",
            parameters=[{
                "control_mode": "auto",
                "i2c_bus": 1,
                "i2c_address": 64,
                # /cmd_vel: zosúladenie s teleop (teleop_invert_linear); otočenie do strany pri lopte
                "cmd_vel_invert_linear": False,
                "cmd_vel_invert_angular": True,
                # Vypnúť snap v auto móde – cmd_vel je už škálované, snap spôsobuje skok na 100% PWM
                "snap_threshold": 0.95,
                # Nižší boost → proporcionálna rýchlosť bez skokov (default 2.35 pri snap=0.22 = 100%)
                "pwm_boost": 2.0,
            }],
        ),

        # Kamera – BGR888 priamo, 640x480 pre rýchlosť
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            parameters=[{
                "camera": 0,
                "format": "BGR888",
                "width":  640,
                "height": 480,
            }],
            # Štandardný topic: /camera/camera_node/image_raw (bez remapu na /camera/image_raw)
        ),

        # Ball follower
        Node(
            package="waverower",
            executable="ball_follower.py",
            name="ball_follower",
            output="screen",
            parameters=[{
                # Musí sedieť s camera_ros: ns=camera, name=camera_node → .../camera_node/image_raw
                "image_topic":      "/camera/camera_node/image_raw",
                "cmd_topic":        "/cmd_vel",
                "ball_color":       "white",
                "angular_kp":       2.0,
                "linear_kp":        0.3,
                "target_radius_px": 50.0,
                "min_radius_px":    15.0,
                # Zahodí obrovské biele bloby (sedák…); lopta na zemi býva rádovo desiatky px
                "max_radius_px":    100.0,
                # Zastavenie pri r ≥ stop_radius_px * stop_radius_scale (1.5× väčší vizuálny prah)
                "stop_radius_px":    72.0,
                "stop_radius_scale": 1.5,
                # Nechaj False ak používaš cmd_vel_invert_angular na motoroch (inak dvojnásobný flip)
                # Normalizovaný x_err prah – kým |x_err| > center_tol, len otáčame
                "center_tol":       0.18,
                "invert_angular":   False,
                # Skús True ak je „ľavá/pravá“ v obraze opačne ako v reáli (kamera)
                "mirror_camera_x":  False,
                "linear_max":       0.15,
                "angular_max":      1.0,
                "search_speed":     0.6,
                "image_use_best_effort_qos": True,
            }],
        ),
    ])
