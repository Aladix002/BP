/* global window */
"use strict";
// Topic nazvy pre roslibjs (musia sediet s ROS).

window.WR = window.WR || {};

Object.assign(window.WR, {
  TELEOP_TOPIC: "/teleop_cmd_vel",
  TWIST_TYPE: "geometry_msgs/msg/Twist",
  CAMERA_TOPIC: "/camera/camera_node/image_raw/compressed",
  CAMERA_TYPE: "sensor_msgs/msg/CompressedImage",
  WEBRTC_SIGNAL_PORT: 8765,
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
  WANDER_TURN_RATIO: 18.0,
  WANDER_THRESHOLD_RANGE: { min: 0.05, max: 1.5, step: 0.01 },
  PTYPE_BOOL: 1,
  PTYPE_DOUBLE: 3,
  PTYPE_STRING: 4,
});

function _sanitizeMediaHost(raw) {
  let s = String(raw || "").trim();
  if (!s) return "";
  s = s.replace(/^https?:\/\//i, "");
  s = s.split("/")[0];
  s = s.split(":")[0];
  return s;
}

window.WR.robotMediaHost = function robotMediaHost() {
  try {
    const u = typeof localStorage !== "undefined" ? localStorage.getItem("waverover_ws_url") : null;
    if (u) return _sanitizeMediaHost(new URL(u).hostname) || "127.0.0.1";
  } catch (_) {}
  if (typeof window !== "undefined" && window.location && window.location.hostname) {
    return window.location.hostname;
  }
  return "127.0.0.1";
};

window.WR.webrtcSignalBaseUrl = function webrtcSignalBaseUrl() {
  const h = window.WR.robotMediaHost();
  const p = window.WR.WEBRTC_SIGNAL_PORT;
  return `http://${h}:${p}`;
};
