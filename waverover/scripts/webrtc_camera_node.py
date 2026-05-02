#!/usr/bin/env python3
# WebRTC stream z CompressedImage; signal HTTP /offer. pip: aiortc aiohttp av.
from __future__ import annotations

import asyncio
import sys
import threading
from typing import Callable, Optional, Set

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

try:
    from aiohttp import web
    from aiortc import RTCPeerConnection, RTCSessionDescription
    from aiortc import VideoStreamTrack
    from av import VideoFrame
except ImportError as e:
    print(
        "Chybaju zavislosti WebRTC: pip install aiortc aiohttp av",
        file=sys.stderr,
    )
    raise e


class CameraVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, get_bgr: Callable[[], np.ndarray]) -> None:
        super().__init__()
        self._get_bgr = get_bgr

    async def recv(self) -> VideoFrame:
        pts, time_base = await self.next_timestamp()
        img = self._get_bgr()
        frame = VideoFrame.from_ndarray(img, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame


class WebrtcCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("webrtc_camera")
        self.declare_parameter("image_topic", "/camera/camera_node/image_raw/compressed")
        self.declare_parameter("http_port", 8765)

        self._lock = threading.Lock()
        self._bgr: Optional[np.ndarray] = None
        self._cb_group = ReentrantCallbackGroup()
        topic = self.get_parameter("image_topic").value
        self.create_subscription(
            CompressedImage,
            topic,
            self._image_cb,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )
        self._pcs: Set[RTCPeerConnection] = set()
        self.get_logger().info(
            f"WebRTC camera: topic={topic}  signal http://0.0.0.0:{self.get_parameter('http_port').value}/offer"
        )

    def _get_latest_bgr(self) -> np.ndarray:
        with self._lock:
            if self._bgr is not None:
                return self._bgr.copy()
        return np.zeros((240, 320, 3), dtype=np.uint8)

    def _image_cb(self, msg: CompressedImage) -> None:
        if not msg.data:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            self._bgr = frame

    async def _on_offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        for pc in list(self._pcs):
            await pc.close()
        self._pcs.clear()

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def _state() -> None:
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self._pcs.discard(pc)

        track = CameraVideoTrack(self._get_latest_bgr)
        pc.addTrack(track)
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        for _ in range(200):
            if pc.iceGatheringState == "complete":
                break
            await asyncio.sleep(0.02)

        return web.json_response(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        )

    async def _health(self, _request: web.Request) -> web.Response:
        return web.Response(text="ok")

    def make_app(self) -> web.Application:
        @web.middleware
        async def cors_middleware(request: web.Request, handler):
            if request.method == "OPTIONS":
                resp = web.Response()
            else:
                resp = await handler(request)
            resp.headers["Access-Control-Allow-Origin"] = "*"
            resp.headers["Access-Control-Allow-Methods"] = "POST, GET, OPTIONS"
            resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
            return resp

        app = web.Application(middlewares=[cors_middleware])
        app.router.add_post("/offer", self._on_offer)
        app.router.add_get("/health", self._health)
        return app


def main() -> None:
    rclpy.init()
    node = WebrtcCameraNode()
    port = int(node.get_parameter("http_port").value)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    async def run_http() -> None:
        app = node.make_app()
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", port)
        await site.start()
        try:
            await asyncio.Future()
        finally:
            await runner.cleanup()

    try:
        asyncio.run(run_http())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
