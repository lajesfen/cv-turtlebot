#!/usr/bin/env python3
import os
import socket
import threading
import base64
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2

# IR del Create 3: si el paquete no está, seguimos sin IR (avisando).
try:
    from irobot_create_msgs.msg import IrIntensityVector
    HAS_IR = True
    IR_IMPORT_ERR = None
except Exception as e:
    HAS_IR = False
    IR_IMPORT_ERR = e


class UdpTelemetryNode(Node):
    def __init__(self):
        super().__init__("udp_telemetry")

        # ========= Parámetros =========
        self.declare_parameter("port", 6000)
        self.declare_parameter("robot_name", "turtlebotoscar")
        self.declare_parameter("pairing_code", "oscar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw")
        self.declare_parameter("ir_topic", "/ir_intensity")
        self.declare_parameter("odom_topic", "/odom")

        port         = self.get_parameter("port").get_parameter_value().integer_value
        self.robot_name   = self.get_parameter("robot_name").get_parameter_value().string_value
        self.pairing_code = self.get_parameter("pairing_code").get_parameter_value().string_value
        scan_topic   = self.get_parameter("scan_topic").get_parameter_value().string_value
        image_topic  = self.get_parameter("image_topic").get_parameter_value().string_value
        ir_topic     = self.get_parameter("ir_topic").get_parameter_value().string_value
        odom_topic   = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        self.get_logger().info(f"=== enviador.py iniciado ===")
        self.get_logger().info(f"ROS_DOMAIN_ID={self.ros_domain_id}  robot_name={self.robot_name}  pairing={self.pairing_code}")
        if not HAS_IR:
            self.get_logger().error(f"NO se pudo importar irobot_create_msgs -> SIN IR. Detalle: {IR_IMPORT_ERR}")

        # ========= Socket UDP =========
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.get_logger().info(f"Telemetría UDP escuchando en 0.0.0.0:{port}")

        self.authorized_addr = None
        self.get_logger().info("Esperando HELLO de la laptop para emparejar...")

        # ========= Throttle de envío =========
        self.ir_min_dt   = 1.0 / 20.0
        self.odom_min_dt = 1.0 / 20.0
        self.last_ir = 0.0
        self.last_odom = 0.0

        # ========= Contadores para logs =========
        self.cnt = {"img": 0, "scan": 0, "ir": 0, "odom": 0}
        self.seen = {"img": False, "scan": False, "ir": False, "odom": False}

        # ========= Subscripciones =========
        self.bridge = CvBridge()
        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.sub_img  = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        if HAS_IR:
            self.sub_ir = self.create_subscription(IrIntensityVector, ir_topic, self.ir_callback, qos_profile_sensor_data)

        # ========= Timer de estado (log cada 2 s) =========
        self.create_timer(2.0, self.status_log)

        # ========= Hilo UDP (HELLO/ACK) =========
        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_loop, daemon=True)
        self.udp_thread.start()

    def status_log(self):
        estado = f"PAREADA {self.authorized_addr}" if self.authorized_addr else "esperando HELLO"
        self.get_logger().info(
            f"[estado] {estado} | enviados/2s -> img:{self.cnt['img']} scan:{self.cnt['scan']} "
            f"ir:{self.cnt['ir']} odom:{self.cnt['odom']}"
        )
        for k in self.cnt:
            self.cnt[k] = 0

    # ================== Hilo UDP (HELLO/ACK) ==================
    def udp_loop(self):
        self.get_logger().info("Hilo UDP iniciado (esperando HELLO).")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                parts = data.decode("utf-8").strip().split()
                if not parts:
                    continue
                if parts[0] == "HELLO":
                    self.handle_hello(parts, addr)
                else:
                    self.get_logger().warn(f"Mensaje inesperado desde {addr}: '{data[:40]}'")
            except Exception as e:
                self.get_logger().error(f"Error en udp_loop: {e}")
                break

    def handle_hello(self, parts, addr):
        self.get_logger().info(f"HELLO recibido desde {addr}: {parts}")
        if len(parts) < 3:
            self.get_logger().warn("HELLO inválido (faltan campos).")
            return
        try:
            desired_domain = int(parts[1])
        except ValueError:
            self.get_logger().warn(f"HELLO domain_id inválido: '{parts[1]}'")
            return
        pairing_code = parts[2]

        if pairing_code != self.pairing_code:
            self.get_logger().warn(f"RECHAZADO: pairing_code '{pairing_code}' != '{self.pairing_code}'")
            return
        if desired_domain != self.ros_domain_id:
            self.get_logger().warn(f"RECHAZADO: domain {desired_domain} != {self.ros_domain_id}")
            return

        if self.authorized_addr is None:
            self.authorized_addr = addr
            self.get_logger().info(f"*** LAPTOP EMPAREJADA: {addr} ***")
        elif addr != self.authorized_addr:
            self.get_logger().warn(f"HELLO desde {addr} pero ya hay PC: {self.authorized_addr}")
            return

        ack = f"ACK {self.ros_domain_id} {self.robot_name}".encode("utf-8")
        self.sock.sendto(ack, addr)
        self.get_logger().info(f"ACK enviado a {addr}")

    # ================== Callbacks ==================
    def scan_callback(self, msg: LaserScan):
        if not self.seen["scan"]:
            self.seen["scan"] = True
            self.get_logger().info(f"Primer /scan recibido (n={len(msg.ranges)} rayos).")
        if self.authorized_addr is None:
            return
        ranges = list(msg.ranges)
        n = len(ranges)
        header = (f"SCAN {self.ros_domain_id} {self.robot_name} "
                  f"{msg.header.stamp.sec} {msg.header.stamp.nanosec} "
                  f"{msg.angle_min} {msg.angle_increment} {n}")
        text = f"{header} " + " ".join(f"{r:.3f}" for r in ranges)
        try:
            self.sock.sendto(text.encode("utf-8"), self.authorized_addr)
            self.cnt["scan"] += 1
        except Exception as e:
            self.get_logger().error(f"Error enviando SCAN: {e}")

    def image_callback(self, msg: Image):
        if not self.seen["img"]:
            self.seen["img"] = True
            self.get_logger().info(f"Primera imagen recibida ({msg.width}x{msg.height}).")
        if self.authorized_addr is None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, jpeg = cv2.imencode(".jpg", cv_img)
            if not ok:
                return
            b64 = base64.b64encode(jpeg.tobytes()).decode("ascii")
            header = (f"IMG {self.ros_domain_id} {self.robot_name} "
                      f"{msg.header.stamp.sec} {msg.header.stamp.nanosec}")
            self.sock.sendto(f"{header} {b64}".encode("utf-8"), self.authorized_addr)
            self.cnt["img"] += 1
        except Exception as e:
            self.get_logger().error(f"Error en image_callback: {e}")

    def ir_callback(self, msg):
        if not self.seen["ir"]:
            self.seen["ir"] = True
            self.get_logger().info(f"Primer /ir_intensity recibido ({len(msg.readings)} sensores).")
        if self.authorized_addr is None:
            return
        now = time.monotonic()
        if now - self.last_ir < self.ir_min_dt:
            return
        self.last_ir = now
        vals = [int(r.value) for r in msg.readings]
        header = (f"IR {self.ros_domain_id} {self.robot_name} "
                  f"{msg.header.stamp.sec} {msg.header.stamp.nanosec}")
        text = header + " " + " ".join(str(v) for v in vals)
        try:
            self.sock.sendto(text.encode("utf-8"), self.authorized_addr)
            self.cnt["ir"] += 1
        except Exception as e:
            self.get_logger().error(f"Error enviando IR: {e}")

    def odom_callback(self, msg: Odometry):
        if not self.seen["odom"]:
            self.seen["odom"] = True
            self.get_logger().info("Primer /odom recibido.")
        if self.authorized_addr is None:
            return
        now = time.monotonic()
        if now - self.last_odom < self.odom_min_dt:
            return
        self.last_odom = now
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        text = (f"ODOM {self.ros_domain_id} {self.robot_name} "
                f"{msg.header.stamp.sec} {msg.header.stamp.nanosec} "
                f"{p.x:.4f} {p.y:.4f} {yaw:.5f}")
        try:
            self.sock.sendto(text.encode("utf-8"), self.authorized_addr)
            self.cnt["odom"] += 1
        except Exception as e:
            self.get_logger().error(f"Error enviando ODOM: {e}")

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
