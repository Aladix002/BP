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
                # /cmd_vel: zosuladenie s teleop; otocenie pri lopte
                "cmd_vel_invert_linear": False,
                "cmd_vel_invert_angular": True,
                # Vypnut snap v auto mode (cmd_vel uz skalovane)
                "snap_threshold": 0.95,
                # Nizsi boost -> proporcionalna rychlost
                "pwm_boost": 2.0,
            }],
        ),

        # Kamera BGR888 640x480
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            parameters=[{
                "camera": 0,
                "format": "BGR888",
                "width": 640,
                "height": 480,
            }],
            # topic: /camera/camera_node/image_raw
        ),

        Node(
            package="waverower",
            executable="ball_follower.py",
            name="ball_follower",
            output="screen",
            parameters=[{
                "image_topic": "/camera/camera_node/image_raw",
                "cmd_topic": "/cmd_vel",
                "ball_color": "white",
                "forward_speed": 0.20,  # m/s dopredu za loptou
                "angular_speed": 0.55,  # rad/s korekcia smeru
                "stop_radius_px": 110.0,  # zastavit ked lopta velka (blizko)
                "min_radius_px": 20.0,  # ignorovat sum
                "max_radius_px": 200.0,  # ignorovat velke bloby
                "min_circularity": 0.72,  # prisnejsi kruh
                "max_contour_area_ratio": 0.10,  # max 10% plochy obrazu
                "search_burst_speed": 0.75,  # rad/s pri hladani
                "burst_on_sec": 0.20,
                "burst_off_sec": 0.35,  # pauza na detekciu
                "image_use_best_effort_qos": True,
            }],
        ),
    ])
