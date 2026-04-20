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
  /** WebRTC signalizácia (HTTP); samotné video ide SRTP/UDP medzi prehliadačom a robotom. */
  WEBRTC_SIGNAL_PORT: 8765,
  /** Ak true, najprv sa skúsi uzol webrtc_camera_node; pri zlyhaní fallback na rosbridge (TCP). */
  USE_WEBRTC_CAMERA: true,
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
  PTYPE_BOOL: 1,
  PTYPE_DOUBLE: 3,
  PTYPE_STRING: 4,
});

/** Základ URL pre POST /offer a GET /health (rovnaký host ako rosbridge WebSocket). */
window.WR.webrtcSignalBaseUrl = function webrtcSignalBaseUrl() {
  let host = "127.0.0.1";
  try {
    const u = typeof localStorage !== "undefined" ? localStorage.getItem("waverover_ws_url") : null;
    if (u) host = new URL(u).hostname;
    else if (typeof window !== "undefined" && window.location && window.location.hostname) {
      host = window.location.hostname;
    }
  } catch (_) {}
  const p = window.WR.WEBRTC_SIGNAL_PORT;
  return `http://${host}:${p}`;
};
