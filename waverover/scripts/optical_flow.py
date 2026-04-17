#!/usr/bin/env python3
# Lucas-Kanade sparse optical flow: odhaduje horizontalny drift sceny -> korekcia angular.z teleop prikazu.
# Aktivny len ked robot ide vpred a pouzivatel netoci (forward_threshold / steer_deadzone).
# /optical_flow_debug (Float64MultiArray): [mean_dx_px, mean_dx_norm, flow_corr, n_good, n_corners]

import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray


class OpticalFlowNode(Node):
    def __init__(self) -> None:
        super().__init__("optical_flow_node")

        self.declare_parameter("correction_gain",          1.5)
        self.declare_parameter("max_correction",           0.3)
        self.declare_parameter("forward_threshold",        0.05)
        self.declare_parameter("steer_deadzone",           0.12)
        self.declare_parameter("min_features",             15)
        self.declare_parameter("enabled",                  False)
        self.declare_parameter("image_topic",              "/camera/camera_node/image_raw/compressed")
        self.declare_parameter("teleop_topic",             "/teleop_cmd_vel")
        self.declare_parameter("output_topic",             "/teleop_cmd_vel_corrected")
        self.declare_parameter("debug_topic",              "/optical_flow_debug")
        self.declare_parameter("debug_show",               False)
        self.declare_parameter("debug_publish_image",      False)
        self.declare_parameter("debug_image_topic",        "/optical_flow/viz/compressed")
        self.declare_parameter("debug_window_scale",       2)
        self.declare_parameter("debug_window_name",        "optical_flow")

        img_topic    = self.get_parameter("image_topic").value
        teleop_topic = self.get_parameter("teleop_topic").value
        out_topic    = self.get_parameter("output_topic").value

        self._lock            = threading.Lock()
        self._prev_gray       = None
        self._flow_correction = 0.0
        self._latest_teleop   = Twist()

        self._sub_image  = self.create_subscription(
            CompressedImage, img_topic, self._image_cb, qos_profile_sensor_data)
        self._sub_teleop = self.create_subscription(
            Twist, teleop_topic, self._teleop_cb, 10)

        self._pub_cmd   = self.create_publisher(Twist, out_topic, 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, self.get_parameter("debug_topic").value, 10)
        self._pub_viz   = self.create_publisher(CompressedImage, self.get_parameter("debug_image_topic").value,
                                                qos_profile_sensor_data)

        self.create_timer(0.05, self._timer_cb)  # 20 Hz

        self.get_logger().info(
            f"OpticalFlow: image={img_topic} teleop={teleop_topic} out={out_topic}"
            f" gain={self.get_parameter('correction_gain').value}"
            f" max_corr={self.get_parameter('max_correction').value}")

    def _teleop_cb(self, msg: Twist) -> None:
        with self._lock:
            self._latest_teleop = msg

    def _image_cb(self, msg: CompressedImage) -> None:
        if not self.get_parameter("enabled").value:
            with self._lock:
                self._prev_gray = None
                self._flow_correction = 0.0
            return

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if frame is None:
            return

        if frame.shape[1] > 320:
            h, w = frame.shape
            frame = cv2.resize(frame, (320, h * 320 // w))

        W = frame.shape[1]
        want_show = self.get_parameter("debug_show").value
        want_pub  = self.get_parameter("debug_publish_image").value
        min_feat  = int(self.get_parameter("min_features").value)

        mean_dx_px = mean_dx_norm = flow_corr = 0.0
        n_corners = n_good = 0
        viz_data  = None

        with self._lock:
            if self._prev_gray is None:
                self._prev_gray = frame
                self._publish_debug(0.0, 0.0, 0.0, 0, 0)
                return

            prev_pts = cv2.goodFeaturesToTrack(self._prev_gray, maxCorners=100, qualityLevel=0.01, minDistance=10)
            n_corners = len(prev_pts) if prev_pts is not None else 0

            if n_corners < min_feat:
                self._prev_gray = frame
                self._flow_correction = 0.0
                self._publish_debug(0.0, 0.0, 0.0, 0, n_corners)
                return

            curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(self._prev_gray, frame, prev_pts, None)
            mask = status.ravel().astype(bool)
            good_prev = prev_pts[mask]
            good_curr = curr_pts[mask]
            n_good = int(mask.sum())

            if n_good > 0:
                mean_dx_px   = float(np.mean(good_curr[:, 0, 0] - good_prev[:, 0, 0]))
                mean_dx_norm = mean_dx_px / W

            if n_good >= min_feat:
                flow_corr = float(np.clip(-mean_dx_norm * self.get_parameter("correction_gain").value,
                                          -self.get_parameter("max_correction").value,
                                           self.get_parameter("max_correction").value))
                self._flow_correction = flow_corr
            else:
                self._flow_correction = 0.0

            self._publish_debug(mean_dx_px, mean_dx_norm, flow_corr, n_good, n_corners)

            if (want_show or want_pub) and n_good > 0:
                viz_data = (frame.copy(), good_prev, good_curr)

            self._prev_gray = frame

        if viz_data is not None:
            self._publish_viz(viz_data, mean_dx_px, mean_dx_norm, flow_corr, n_good, n_corners,
                              want_show, want_pub, msg)

    def _publish_debug(self, mean_dx_px: float, mean_dx_norm: float, flow_corr: float,
                       n_good: int, n_corners: int) -> None:
        msg = Float64MultiArray()
        msg.data = [mean_dx_px, mean_dx_norm, flow_corr, float(n_good), float(n_corners)]
        self._pub_debug.publish(msg)

    def _publish_viz(self, viz_data, mean_dx_px, mean_dx_norm, flow_corr,
                     n_good, n_corners, want_show, want_pub, src_msg) -> None:
        frame_gray, good_prev, good_curr = viz_data
        vis = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)
        for a, b in zip(good_prev.reshape(-1, 2), good_curr.reshape(-1, 2)):
            cv2.line(vis, tuple(a.astype(int)), tuple(b.astype(int)), (0, 255, 120), 1, cv2.LINE_AA)
            cv2.circle(vis, tuple(a.astype(int)), 2, (255, 80, 0), -1, cv2.LINE_AA)
            cv2.circle(vis, tuple(b.astype(int)), 2, (200, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(vis,
                    f"mean_dx_px {mean_dx_px:.3f}  norm {mean_dx_norm:.3f}"
                    f"  corr {flow_corr:.3f}  ok {n_good}/{n_corners}",
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        sc = int(np.clip(self.get_parameter("debug_window_scale").value, 1, 8))
        if sc > 1:
            vis = cv2.resize(vis, None, fx=sc, fy=sc, interpolation=cv2.INTER_NEAREST)

        if want_pub:
            ok, jpeg = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                out = CompressedImage()
                out.header = src_msg.header
                if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
                    out.header.stamp = self.get_clock().now().to_msg()
                out.format = "jpeg"
                out.data = jpeg.tobytes()
                self._pub_viz.publish(out)

        if want_show:
            cv2.imshow(self.get_parameter("debug_window_name").value, vis)
            cv2.waitKey(1)

    def _timer_cb(self) -> None:
        with self._lock:
            out = Twist()
            out.linear  = self._latest_teleop.linear
            out.angular = self._latest_teleop.angular
            correction  = self._flow_correction

        if not self.get_parameter("enabled").value:
            self._pub_cmd.publish(out)
            return

        moving_forward = abs(out.linear.x) > self.get_parameter("forward_threshold").value
        user_steering  = abs(out.angular.z) > self.get_parameter("steer_deadzone").value

        if moving_forward and not user_steering:
            out.angular.z = float(np.clip(out.angular.z + correction, -1.5, 1.5))

        self._pub_cmd.publish(out)


def main() -> None:
    rclpy.init()
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
