#!/usr/bin/env python3
# Pomocne funkcie zdielane viacerymi uzlami (simple_nav_node, cmd_vel_odom).
import math


def quat_to_yaw(q) -> float:
    # Extrahuje yaw (otocenie okolo osi Z) z kvaterniona ROS (geometry_msgs/Quaternion).
    # Pouziva vzorec z Euler ZYX dekompozicie: atan2(2(wz+xy), 1-2(y^2+z^2)).
    # Vysledok je v radianoch, rozsah -pi..+pi.
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def imu_quat_to_yaw(q) -> float:
    # Rovnake ako quat_to_yaw, ale s opacnym znamienkom.
    # Pouziva sa pre IMU (Arduino MPU6050) kde ROS konvencia a fyzicka os Z IMU
    # su prehodene voci sebe -> zaporny yaw zodpoveda kladnemu otoceniu robota.
    return -quat_to_yaw(q)
