/* global window */
"use strict";
// Centralne nazvy topicov a typov pre roslibjs; musia sediet s realnymi nazvami v ROS (camera namespace).

window.WR = window.WR || {};

Object.assign(window.WR, {
  TELEOP_TOPIC: "/teleop_cmd_vel",
  TWIST_TYPE: "geometry_msgs/msg/Twist",
  // camera_ros + namespace: skutocny compressed topic (nie /camera/image_raw)
  CAMERA_TOPIC: "/camera/camera_node/image_raw/compressed",
  CAMERA_TYPE: "sensor_msgs/msg/CompressedImage",
  IMU_TOPIC: "/imu",
  IMU_TYPE: "sensor_msgs/msg/Imu",
  IMU_DBG_TOPIC: "/motor_hat_node/drive_debug",
  IMU_DBG_TYPE: "std_msgs/msg/Float64MultiArray",
  MOTOR_NODE: "/motor_hat_node",
  WANDER_NODE: "/lidar_wander_node",
  OPTICAL_NODE: "/optical_flow_node",
  PUBLISH_HZ: 30,
  SLIDER_SCALE: { min: 0.0, max: 1.0, step: 0.01 },
  // Rovnaky pomer ako WANDER_TURN_RATIO v runtime_stack.launch.py
  WANDER_TURN_RATIO: 18.0,
  WANDER_THRESHOLD_RANGE: { min: 0.05, max: 1.5, step: 0.01 },
  IMU_DEFAULTS: {
    imu_correction: true,
    imu_kp: 0.30,
    imu_ki: 0.05,
    imu_kd: 0.01,
    imu_deadband: 0.02,
    imu_windup: 0.30,
  },
  PTYPE_BOOL: 1,
  PTYPE_DOUBLE: 3,
  PTYPE_STRING: 4,
});
