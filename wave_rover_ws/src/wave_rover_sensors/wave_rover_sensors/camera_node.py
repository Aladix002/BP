#!/usr/bin/env python3
"""
Camera node – publishes compressed JPEG images only (bandwidth-friendly for RPi).

Publishes:
  /camera/compressed    (sensor_msgs/CompressedImage)  – always on
  /camera/camera_info   (sensor_msgs/CameraInfo)
  /camera/image_raw     (sensor_msgs/Image)             – opt-in, default OFF

Parameters:
  device_id        [int]    default 0        – /dev/videoX index
  width            [int]    default 640
  height           [int]    default 480
  fps              [float]  default 30.0
  frame_id         [string] default "camera_link"
  jpeg_quality     [int]    default 80       – JPEG compression quality 1-100
  publish_raw      [bool]   default false    – also publish uncompressed Image
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Header

try:
    import cv2
    import numpy as np
    _HAS_CV = True
except ImportError:
    _HAS_CV = False


def _make_camera_info(width: int, height: int, frame_id: str) -> CameraInfo:
    """Build a minimal CameraInfo (pinhole, no distortion)."""
    ci = CameraInfo()
    ci.header.frame_id = frame_id
    ci.width  = width
    ci.height = height
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    # Approximate focal length for 640×480, 66° HFOV
    fx = width / (2.0 * 0.577)
    fy = fx
    cx = width  / 2.0
    cy = height / 2.0
    ci.k = [fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0]
    ci.r = [1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0]
    return ci


class CameraNode(Node):

    def __init__(self) -> None:
        super().__init__('camera_node')

        self.declare_parameter('device_id',    0)
        self.declare_parameter('width',        640)
        self.declare_parameter('height',       480)
        self.declare_parameter('fps',          30.0)
        self.declare_parameter('frame_id',     'camera_link')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('publish_raw',  False)

        self._device    = self.get_parameter('device_id').value
        self._width     = self.get_parameter('width').value
        self._height    = self.get_parameter('height').value
        self._fps       = max(1.0, self.get_parameter('fps').value)
        self._frame_id  = self.get_parameter('frame_id').value
        self._quality   = self.get_parameter('jpeg_quality').value
        self._do_raw    = self.get_parameter('publish_raw').value

        self._cap: 'cv2.VideoCapture | None' = None

        if not _HAS_CV:
            self.get_logger().error(
                'opencv-python not installed – camera node will publish nothing')
        else:
            self._open_camera()

        # Compressed is always created; raw is optional
        self._pub_comp = self.create_publisher(CompressedImage, 'camera/compressed',   10)
        self._pub_info = self.create_publisher(CameraInfo,      'camera/camera_info',  10)
        self._pub_img  = self.create_publisher(Image,           'camera/image_raw',    10) if self._do_raw else None

        period = 1.0 / self._fps
        self._timer = self.create_timer(period, self._tick)

        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'CameraNode ready  device=%d  %dx%d @ %.0f fps  quality=%d  raw=%s',
            self._device, self._width, self._height, self._fps,
            self._quality, self._do_raw,
        )

    def _open_camera(self) -> None:
        cap = cv2.VideoCapture(self._device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS,          self._fps)
        if not cap.isOpened():
            self.get_logger().error('Failed to open /dev/video%d', self._device)
            return
        self._cap = cap

    def _tick(self) -> None:
        if not _HAS_CV or self._cap is None:
            return

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed – reopening')
            self._cap.release()
            self._open_camera()
            return

        now = self.get_clock().now().to_msg()

        # ── Compressed JPEG (always published) ───────────────────────────
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            comp_msg = CompressedImage()
            comp_msg.header.stamp    = now
            comp_msg.header.frame_id = self._frame_id
            comp_msg.format = 'jpeg'
            comp_msg.data   = buf.tobytes()
            self._pub_comp.publish(comp_msg)

        # ── CameraInfo (always published) ────────────────────────────────
        ci = _make_camera_info(self._width, self._height, self._frame_id)
        ci.header.stamp = now
        self._pub_info.publish(ci)

        # ── Raw Image (opt-in only) ───────────────────────────────────────
        if self._do_raw and self._pub_img is not None:
            img_msg = Image()
            img_msg.header.stamp    = now
            img_msg.header.frame_id = self._frame_id
            img_msg.height   = frame.shape[0]
            img_msg.width    = frame.shape[1]
            img_msg.encoding = 'bgr8'
            img_msg.step     = frame.shape[1] * 3
            img_msg.data     = frame.tobytes()
            self._pub_img.publish(img_msg)

        self.get_logger().debug('Camera frame captured %dx%d', frame.shape[1], frame.shape[0])

    def _on_params(self, params):
        for p in params:
            if p.name == 'jpeg_quality':
                self._quality = max(1, min(100, int(p.value)))
            elif p.name == 'publish_raw':
                self._do_raw = bool(p.value)
        return SetParametersResult(successful=True)

    def destroy_node(self) -> None:
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
