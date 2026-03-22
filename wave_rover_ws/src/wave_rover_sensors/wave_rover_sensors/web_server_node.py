#!/usr/bin/env python3
"""
Web server node – serves the Wave Rover mobile UI over HTTP.

Navigate to http://ROBOT_IP:8888 from any device on the same WiFi.

Parameters:
  port      [int]     default 8888
  web_dir   [string]  default <package_share>/web
"""

import os
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node


class _Handler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass   # silence access log


class WebServerNode(Node):

    def __init__(self) -> None:
        super().__init__('web_server_node')

        self.declare_parameter('port',    8888)
        self.declare_parameter('web_dir', '')

        port = self.get_parameter('port').value

        # Resolve web directory
        web_dir = self.get_parameter('web_dir').value
        if not web_dir or not os.path.isdir(web_dir):
            try:
                from ament_index_python.packages import get_package_share_directory
                web_dir = os.path.join(
                    get_package_share_directory('wave_rover_bringup'), 'web')
            except Exception:
                # Fallback: source-tree location
                web_dir = os.path.join(
                    os.path.dirname(__file__),
                    '..', '..', '..', '..', 'share', 'wave_rover_bringup', 'web')

        if not os.path.isdir(web_dir):
            self.get_logger().error('Web directory not found: %s', web_dir)
            return

        handler = lambda *a, **kw: _Handler(*a, directory=web_dir, **kw)
        server  = HTTPServer(('0.0.0.0', port), handler)

        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        # Get local IP for display
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            ip = '127.0.0.1'

        self.get_logger().info(
            'Web UI available at http://%s:%d  (also http://localhost:%d)',
            ip, port, port,
        )
        self.get_logger().info(
            'Open this URL on any phone/tablet connected to the same WiFi')


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
