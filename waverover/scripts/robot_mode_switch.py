#!/usr/bin/env python3
# Trigger sluzby: prepnutie motor/wander parametrov cez AsyncParameterClient.

import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
try:
    from rclpy.parameter_client import AsyncParameterClient as _RemoteParamClient
except ImportError:  # pragma: no cover
    from rclpy.parameter_client import AsyncParametersClient as _RemoteParamClient

from std_srvs.srv import Trigger


class RobotModeSwitch(Node):
    def __init__(self) -> None:
        super().__init__("robot_mode_switch")

        self._cb_group = ReentrantCallbackGroup()

        self._motor = _RemoteParamClient(self, "/motor_hat_node", callback_group=self._cb_group)
        self._wander = _RemoteParamClient(self, "/lidar_wander_node", callback_group=self._cb_group)

        self._srv_manual = self.create_service(
            Trigger, "/waverover/switch_to_manual", self._on_manual,
            callback_group=self._cb_group,
        )
        self._srv_wander = self.create_service(
            Trigger, "/waverover/switch_to_wander", self._on_wander,
            callback_group=self._cb_group,
        )
        self._srv_nav = self.create_service(
            Trigger, "/waverover/switch_to_nav", self._on_nav,
            callback_group=self._cb_group,
        )
        self._srv_shutdown = self.create_service(
            Trigger, "/waverover/shutdown", self._on_shutdown,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "Sluzby: /waverover/switch_to_{manual,wander,nav}, /waverover/shutdown"
        )

    def _wait_clients(self) -> bool:
        if hasattr(self._motor, "wait_for_services"):
            ok_m = self._motor.wait_for_services(timeout_sec=5.0)
            ok_w = self._wander.wait_for_services(timeout_sec=5.0)
        else:
            ok_m = self._motor.wait_for_service(timeout_sec=5.0)
            ok_w = self._wander.wait_for_service(timeout_sec=5.0)
        if not ok_m:
            self.get_logger().warn("motor_hat_node set_parameters nedostupne")
        if not ok_w:
            self.get_logger().warn("lidar_wander_node set_parameters nedostupne")
        return ok_m and ok_w

    def _wait_future(self, future, timeout_sec: float = 10.0) -> bool:
        t0 = time.monotonic()
        while not future.done() and (time.monotonic() - t0) < timeout_sec:
            time.sleep(0.005)
        return future.done()

    def _switch(self, motor_mode: str, wander_enabled: bool) -> Trigger.Response:
        resp = Trigger.Response()
        if not self._wait_clients():
            resp.success = False
            resp.message = "parameter services not ready"
            return resp
        fut_m = self._motor.set_parameters([
            Parameter("control_mode", Parameter.Type.STRING, motor_mode),
        ])
        if not self._wait_future(fut_m):
            resp.success = False
            resp.message = "timeout: motor set_parameters"
            return resp
        fut_w = self._wander.set_parameters([
            Parameter("enabled", Parameter.Type.BOOL, wander_enabled),
        ])
        if not self._wait_future(fut_w):
            resp.success = False
            resp.message = "timeout: lidar_wander set_parameters"
            return resp
        rm = fut_m.result()
        rw = fut_w.result()
        ok = (bool(rm) and all(r.successful for r in rm.results)
              and bool(rw) and all(r.successful for r in rw.results))
        resp.success = ok
        resp.message = motor_mode if ok else "set_parameters failed"
        return resp

    def _on_manual(self, _req: Trigger.Request, _resp: Trigger.Response) -> Trigger.Response:
        return self._switch("manual", False)

    def _on_wander(self, _req: Trigger.Request, _resp: Trigger.Response) -> Trigger.Response:
        return self._switch("auto", True)

    def _on_nav(self, _req: Trigger.Request, _resp: Trigger.Response) -> Trigger.Response:
        return self._switch("auto", False)

    def _on_shutdown(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Shutdown requested via /waverover/shutdown")
        resp.success = True
        resp.message = "Shutting down"
        threading.Timer(1.0, lambda: subprocess.run(["sudo", "systemctl", "poweroff"], check=False)).start()
        return resp


def main() -> None:
    rclpy.init()
    node = RobotModeSwitch()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
