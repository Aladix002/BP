#!/usr/bin/env python3
# Arduino (MPU6050) posiela CSV po USB. Tento uzol parsuje riadky, plni sensor_msgs/Imu.
# Vlakno cita seriu (neblokuje spin); spravy idu cez frontu do timeru co publikuje na /imu.

import math
import os
import queue
import re
import select
import subprocess
import termios
import threading
from glob import glob
from typing import BinaryIO, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

_IMU_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


GRAVITY = 9.80665
DEG2RAD = math.pi / 180.0


def _baud_from_param(node: Node) -> int:
    pv = node.get_parameter("baud_rate").get_parameter_value()
    if pv.type == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if pv.type == ParameterType.PARAMETER_STRING and pv.string_value:
        return int(pv.string_value.strip())
    return 115200


def _quaternion_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple:
    # Euler ZYX v stupnoch -> kvaternion pre Imu.orientation (ROS konvencia)
    r = roll_deg * DEG2RAD
    p = pitch_deg * DEG2RAD
    y = yaw_deg * DEG2RAD
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yq = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, yq, z, w)


def _sorted_tty_acm() -> List[str]:
    # ttyACM10 pred ttyACM2 pri lex sorte - preto vlastne radenie podla cisla
    paths = glob("/dev/ttyACM*")

    def sort_key(p: str) -> tuple:
        m = re.search(r"ACM(\d+)$", p)
        return (int(m.group(1)) if m else 9999, p)

    return sorted(paths, key=sort_key)


def _csv_line_looks_like_imu(line: str) -> bool:
    # Rychly test pred plnym parse (scan portov)
    if not line or line.startswith("ERR") or "READ_FAIL" in line:
        return False
    parts = [p.strip() for p in line.split(",")]
    n = len(parts)
    if n not in (15, 16, 20):
        return False
    if len(parts) > 0 and parts[0].isalpha():
        return False
    try:
        f = [float(x) for x in parts]
    except ValueError:
        return False
    if any(f[i] == -1.0 for i in range(1, min(8, len(f)))):
        return False
    return True


def _build_imu_port_candidates(requested: str) -> List[str]:
    # Poradie skusania: uzivatelov port, potom stabilne by-id, potom vsetky ACM, USB
    seen = set()
    out: List[str] = []

    def add(p: str) -> None:
        if p and os.path.exists(p) and p not in seen:
            seen.add(p)
            out.append(p)

    req = (requested or "").strip()
    if req:
        add(req)
    for p in sorted(glob("/dev/serial/by-id/*"), key=lambda x: os.path.basename(x).lower()):
        if "arduino" in os.path.basename(p).lower():
            add(p)
    for p in _sorted_tty_acm():
        add(p)
    for p in sorted(glob("/dev/ttyUSB*")):
        add(p)
    return out


def _open_imu_serial_probed(
    candidates: List[str],
    baud: int,
    run_stty: bool,
    log_warn,
) -> Tuple[BinaryIO, str]:
    # Pre kazdy kandidat: select caka na data, readline musi byt platny IMU CSV
    for path in candidates:
        if not os.path.exists(path):
            continue
        fp: Optional[BinaryIO] = None
        try:
            if run_stty:
                subprocess.run(
                    ["stty", "-F", path, str(baud), "raw", "-echo"],
                    check=False,
                    capture_output=True,
                )
            fp = open(path, "rb")
            try:
                termios.tcflush(fp.fileno(), termios.TCIFLUSH)
            except (OSError, termios.error):
                pass
            for _ in range(24):
                r, _, _ = select.select([fp], [], [], 0.4)
                if not r:
                    log_warn(f"IMU scan: {path} - cakanie na riadok (timeout)")
                    break
                raw = fp.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="ignore").strip()
                if _csv_line_looks_like_imu(line):
                    out_fp = fp
                    fp = None  # nezatvarat v finally: vraciame otvoreny port
                    return out_fp, path
        except OSError as ex:
            log_warn(f"IMU scan: {path} — {ex}")
        finally:
            if fp is not None:
                try:
                    fp.close()
                except OSError:
                    pass
    raise OSError(
        "Ziaden z kandidatskych portov neposlal platny IMU CSV (15/16/20 poli). "
        f"Skontroluj: {', '.join(candidates[:6])}{'...' if len(candidates) > 6 else ''}"
    )


def _open_imu_serial_simple(path: str, baud: int, run_stty: bool) -> BinaryIO:
    # Bez probe - ked scan_serial_ports:=false
    if run_stty:
        subprocess.run(
            ["stty", "-F", path, str(baud), "raw", "-echo"],
            check=False,
            capture_output=True,
        )
    return open(path, "rb")


class ImuSerialFusionBridge(Node):
    # Datova draha: _read_loop (thread) -> fronta -> _flush_publish (1ms timer) -> publisher
    # Ak je zero_gyro_on_start: prvy N vzoriek len na priemer biasu, /imu sa nepublikuje
    def __init__(self) -> None:
        super().__init__("imu_serial_fusion_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "/imu")
        self.declare_parameter("run_stty", True)
        self.declare_parameter("use_best_effort_qos", False)
        self.declare_parameter("publish_extra_fields", True)
        self.declare_parameter("extra_topic", "/imu/arduino_extra")
        self.declare_parameter("fill_orientation_from_fusion", True)
        self.declare_parameter("zero_yaw_on_start", True)
        # Gyro (rad/s): odcita priemer z prvych N vzoriek pri pokoji; omega ~ 0 v /imu ako pri vynulovani
        self.declare_parameter("zero_gyro_on_start", True)
        self.declare_parameter("gyro_zero_warmup_samples", 25)
        # True: vyskusa kandidatov (serial_port, by-id Arduino, ttyACM0..N, ttyUSB*) kym nepride IMU CSV
        self.declare_parameter("scan_serial_ports", True)

        requested_port = self.get_parameter("serial_port").get_parameter_value().string_value
        scan = self.get_parameter("scan_serial_ports").get_parameter_value().bool_value
        baud = _baud_from_param(self)
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        run_stty = self.get_parameter("run_stty").get_parameter_value().bool_value
        use_be = self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        self._pub_extra = self.get_parameter("publish_extra_fields").get_parameter_value().bool_value
        extra_topic = self.get_parameter("extra_topic").get_parameter_value().string_value
        self._fill_ori = self.get_parameter("fill_orientation_from_fusion").get_parameter_value().bool_value
        self._zero_yaw = self.get_parameter("zero_yaw_on_start").get_parameter_value().bool_value
        self._zero_gyro = self.get_parameter("zero_gyro_on_start").get_parameter_value().bool_value
        gw_pv = self.get_parameter("gyro_zero_warmup_samples").get_parameter_value()
        try:
            self._gyro_warmup = max(1, int(gw_pv.integer_value))
        except Exception:
            self._gyro_warmup = 25
        self._yaw_offset: Optional[float] = None
        self._gyro_sum = [0.0, 0.0, 0.0]
        self._gyro_n = 0
        self._gyro_bias: Optional[tuple] = None  # (bx,by,bz) rad/s po warmupe


        port: str
        try:
            if scan:
                candidates = _build_imu_port_candidates(requested_port)
                if not candidates:
                    raise OSError(
                        "Nenasiel som ziadny seriovy port "
                        "(serial_port /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB*)."
                    )
                self.get_logger().info(
                    "IMU scan: skusam porty: " + ", ".join(candidates[:8])
                    + (" ..." if len(candidates) > 8 else "")
                )
                self._fp, port = _open_imu_serial_probed(
                    candidates, baud, run_stty, self.get_logger().warn
                )
            else:
                port = requested_port.strip() or "/dev/ttyACM0"
                if not os.path.exists(port):
                    raise OSError(f"Port neexistuje: {port}")
                self._fp = _open_imu_serial_simple(port, baud, run_stty)
        except OSError as ex:
            self.get_logger().fatal(f"Seriovy port IMU: {ex}")
            raise

        try:
            termios.tcflush(self._fp.fileno(), termios.TCIFLUSH)
        except (OSError, termios.error):
            pass

        qos = qos_profile_sensor_data if use_be else _IMU_QOS
        self._pub = self.create_publisher(Imu, topic, qos)
        self._pub_ex = (
            self.create_publisher(Float64MultiArray, extra_topic, 10)
            if self._pub_extra
            else None
        )
        self._queue: queue.Queue = queue.Queue(maxsize=256)
        self._queue_ex: queue.Queue = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        # 1 kHz flush: z threadu do ROS bez blokovania read_loop
        self.create_timer(0.001, self._flush_publish)

        self.get_logger().info(
            f"IMU fusion serial -> {topic} ({port} @ {baud}, 15/20 CSV, frame_id={self._frame_id})"
        )
        if self._zero_gyro:
            self.get_logger().info(
                f"zero_gyro_on_start: prvych {self._gyro_warmup} vzoriek sa nepublikuje "
                "(robot nech je v pokoji), potom sa odhadne bias gyro."
            )

    def _flush_publish(self) -> None:
        # Omezene mnozstvo za tick aby spin nezastal pri zahlteni
        for _ in range(64):
            try:
                msg = self._queue.get_nowait()
            except queue.Empty:
                break
            self._pub.publish(msg)
        if self._pub_ex:
            for _ in range(8):
                try:
                    ex = self._queue_ex.get_nowait()
                except queue.Empty:
                    break
                self._pub_ex.publish(ex)

    def _read_loop(self) -> None:
        # Blokujuce readline z /dev; parse len platnych CSV
        while not self._stop.is_set() and rclpy.ok():
            try:
                raw = self._fp.readline()
            except OSError:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("ERR") or "READ_FAIL" in line:
                continue
            parsed = self._parse_line(line)
            if parsed is None:
                continue
            imu_msg, extra = parsed
            try:
                self._queue.put_nowait(imu_msg)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(imu_msg)
                except queue.Full:
                    pass
            if extra is not None and self._pub_ex:
                try:
                    self._queue_ex.put_nowait(extra)
                except queue.Full:
                    pass

    def _apply_yaw_offset(self, yaw_deg: float) -> float:
        # Od prveho yaw odcitame offset aby 0 bolo "startovacia orientacia"
        if not self._zero_yaw:
            return yaw_deg
        if self._yaw_offset is None:
            self._yaw_offset = yaw_deg
            self.get_logger().info(f"Yaw vynulovany, pociatok: {yaw_deg:.2f} deg")
        return yaw_deg - self._yaw_offset

    def _parse_line(self, line: str) -> Optional[tuple]:
        # Format 15: akcelerometer g, gyro dps, bez plnej orientacie
        # Format 16: ako 15 + yaw
        # Format 20: fusion uhly + extra polia (quaternion z roll,pitch,yaw)
        parts = [p.strip() for p in line.split(",")]
        n = len(parts)
        if n not in (15, 16, 20):
            if len(parts) > 0 and parts[0].isalpha():
                return None
            return None
        try:
            f = [float(x) for x in parts]
        except ValueError:
            return None
        if any(f[i] == -1.0 for i in range(1, 8)):
            return None

        ax_g, ay_g, az_g = f[8], f[9], f[10]
        gx_dps, gy_dps, gz_dps = f[12], f[13], f[14]

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id

        ax_ms2 = ax_g * GRAVITY
        ay_ms2 = ay_g * GRAVITY
        az_ms2 = az_g * GRAVITY
        gx_rad = gx_dps * DEG2RAD
        gy_rad = gy_dps * DEG2RAD
        gz_rad = gz_dps * DEG2RAD

        if self._zero_gyro:
            if self._gyro_bias is None:
                self._gyro_sum[0] += gx_rad
                self._gyro_sum[1] += gy_rad
                self._gyro_sum[2] += gz_rad
                self._gyro_n += 1
                if self._gyro_n >= self._gyro_warmup:
                    # Pouzivame n_samples aby sme nepretlacili n = len(parts) z vonkajsieho scope
                    n_samples = float(self._gyro_n)
                    self._gyro_bias = (
                        self._gyro_sum[0] / n_samples,
                        self._gyro_sum[1] / n_samples,
                        self._gyro_sum[2] / n_samples,
                    )
                    self.get_logger().info(
                        "Gyro bias odhad (priemer z %d vzoriek, rad/s): "
                        "wx=%.5f wy=%.5f wz=%.5f"
                        % (self._gyro_n, self._gyro_bias[0], self._gyro_bias[1], self._gyro_bias[2])
                    )
            if self._gyro_bias is not None:
                gx_rad -= self._gyro_bias[0]
                gy_rad -= self._gyro_bias[1]
                gz_rad -= self._gyro_bias[2]

        out.linear_acceleration.x = ax_ms2
        out.linear_acceleration.y = ay_ms2
        out.linear_acceleration.z = az_ms2
        out.angular_velocity.x = gx_rad
        out.angular_velocity.y = gy_rad
        out.angular_velocity.z = gz_rad

        extra_msg: Optional[Float64MultiArray] = None

        if n == 20 and self._fill_ori:
            # roll_f, pitch_f, yaw_gyro z CSV
            roll_deg, pitch_deg, yaw_deg = f[17], f[18], f[19]
            yaw_deg = self._apply_yaw_offset(yaw_deg)
            qx, qy, qz, qw = _quaternion_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
            out.orientation.x = qx
            out.orientation.y = qy
            out.orientation.z = qz
            out.orientation.w = qw
            for i in range(9):
                out.orientation_covariance[i] = 0.01
            if self._pub_extra:
                extra_msg = Float64MultiArray()
                extra_msg.data = [float(f[15]), float(f[16])]
        elif n == 16 and self._fill_ori:
            # 16 poli: 15 + yaw_gyro
            yaw_deg = self._apply_yaw_offset(f[15])
            qx, qy, qz, qw = _quaternion_from_rpy_deg(0.0, 0.0, yaw_deg)
            out.orientation.x = qx
            out.orientation.y = qy
            out.orientation.z = qz
            out.orientation.w = qw
            for i in range(9):
                out.orientation_covariance[i] = 0.05  # len yaw
        else:
            out.orientation_covariance[0] = -1.0

        # Pocas warm-upu este nemame bias - neposielame /imu (inak by omega ukazovalo surovy offset)
        if self._zero_gyro and self._gyro_bias is None:
            return None

        return (out, extra_msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        if hasattr(self, "_reader") and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if hasattr(self, "_fp") and self._fp:
            try:
                self._fp.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuSerialFusionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
