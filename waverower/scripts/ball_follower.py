#!/usr/bin/env python3
"""Sledovanie lopty (priamy rezim), bez action servera."""

import rclpy

from ball_follower_base import BallFollowerBase


class BallFollower(BallFollowerBase):
    """Sledovanie stale zapnute."""

    pass


def main() -> None:
    rclpy.init()
    node = BallFollower()
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
