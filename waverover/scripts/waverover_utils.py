#!/usr/bin/env python3
# Spolocne util pre simple_nav a cmd_vel_odom.
import math


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def imu_quat_to_yaw(q) -> float:
    # Rovnake ako quat_to_yaw, ale s opacnym znamienkom.
    # Pouziva sa pre IMU (Arduino MPU6050) kde ROS konvencia a fyzicka os Z IMU
    # su prehodene voci sebe -> zaporny yaw zodpoveda kladnemu otoceniu robota.
    return -quat_to_yaw(q)
