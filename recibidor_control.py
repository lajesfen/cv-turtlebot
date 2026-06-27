#!/usr/bin/env python3
# CORRE EN EL ROBOT. Recibe (v,w) por UDP y publica en /cmd_vel.
# Watchdog: si no llega comando en WATCHDOG_S, frena solo (seguridad).
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

USE_STAMPED = True       # True -> TwistStamped en /cmd_vel (TB4 Jazzy).
                         # False -> Twist en /cmd_vel_unstamped. Cambia si no se mueve.
CMD_PORT   = 5007        # debe coincidir con cerebro.py / controller (CONTROL_PORT)
PUBLISH_HZ = 50.0
WATCHDOG_S = 0.5         # sin comando en este tiempo -> (0,0)


class UdpCmdVelNode(Node):
    def __init__(self):
        super().__init__("udp_cmd_vel")

        if USE_STAMPED:
            self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        else:
            self.pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)

        self.v = 0.0
        self.w = 0.0
        self.last_rx = self.get_clock().now()
        self.lock = threading.Lock()
        self.got_first = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", CMD_PORT))
        self.get_logger().info(f"=== recibidor_control.py === escuchando (v,w) en 0.0.0.0:{CMD_PORT}")
        self.get_logger().info(f"Publicando en {'/cmd_vel (TwistStamped)' if USE_STAMPED else '/cmd_vel_unstamped (Twist)'}")

        self.running = True
        threading.Thread(target=self.rx_loop, daemon=True).start()
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_cmd)
        self.create_timer(1.0, self.status_log)
        self.last_logged = (0.0, 0.0)

    def status_log(self):
        with self.lock:
            v, w, last = self.v, self.w, self.last_rx
        dt = (self.get_clock().now() - last).nanoseconds * 1e-9
        wd = " [WATCHDOG: frenado]" if dt > WATCHDOG_S else ""
        self.get_logger().info(f"[ctrl] ultimo (v={v:+.2f}, w={w:+.2f})  hace {dt:.2f}s{wd}")

    def rx_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) < 8:
                    continue
                v, w = struct.unpack("ff", data[:8])
                with self.lock:
                    self.v, self.w = float(v), float(w)
                    self.last_rx = self.get_clock().now()
                if not self.got_first:
                    self.got_first = True
                    self.get_logger().info("*** Primer comando recibido de la laptop ***")
            except Exception as e:
                self.get_logger().error(f"rx_loop: {e}")
                break

    def publish_cmd(self):
        with self.lock:
            v, w, last = self.v, self.w, self.last_rx
        dt = (self.get_clock().now() - last).nanoseconds * 1e-9
        if dt > WATCHDOG_S:
            v, w = 0.0, 0.0

        if USE_STAMPED:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = v
            msg.twist.angular.z = w
        else:
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UdpCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
