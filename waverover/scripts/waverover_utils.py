#!/usr/bin/env python3
import math


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def imu_quat_to_yaw(q) -> float:
    return -quat_to_yaw(q)
