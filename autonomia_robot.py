#!/usr/bin/env python3
# CORRE EN LA RASPBERRY DEL ROBOT (nodo ROS2). Autonomia A BORDO: no depende del wifi.
# Capa 1: avanzar + esquivar con LiDAR /scan + registrar QR (log con timestamp).
# Arranque/parada con el BOTON 1 del TurtleBot (interrupcion fisica). SIN YOLO todavia.
#
# Correr en el robot:   export ROS_DOMAIN_ID=67 && python3 ~/autonomia_robot.py
# NO correr a la vez que recibidor_control.py (ambos publican /cmd_vel).
import time
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
import cv2

# Boton fisico (opcional). Si no esta, se arranca con START_ARMED=True.
try:
    from irobot_create_msgs.msg import InterfaceButtons
    HAS_BUTTONS = True
except Exception:
    HAS_BUTTONS = False

# ==================== CONFIG ====================
USE_STAMPED = False         # ESTE robot: create3_repub escucha Twist en /cmd_vel_unstamped (best_effort).
                            #            /cmd_vel (TwistStamped) NO tiene suscriptores -> no mueve.
START_ARMED = False         # False = arranca en IDLE; se arma con el boton 1
STAGE       = 1
QR_LOG_FILE = "/home/ubuntu/checkpoints_log.txt"

# Velocidades (a bordo NO hay delay -> puede ser un poco mas agil)
LIN   = 0.18
ANG   = 1.0
CREEP = 0.10

# Anticolision por LiDAR (metros)
D_STOP  = 0.35
D_SLOW  = 0.80
D_CLEAR = 0.50
K_STEER = 0.6
VMIN_FRAC = 0.4

# Geometria del LiDAR (USA EL FRONT_DEG QUE CALIBRASTE EN cerebro.py)
FRONT_DEG      = 0.0
FRONT_HALF_DEG = 25.0
SIDE_DEG       = 50.0
SIDE_HALF_DEG  = 25.0
R_MIN, R_MAX   = 0.06, 10.0
SCAN_DEBUG     = True

# Follow-the-Gap (navegacion suave: apunta al hueco mas grande)
FGM_ARC_DEG   = 90.0    # mira +-90 grados alrededor del frente
FGM_RMAX      = 3.0     # recorta distancias lejanas (m)
FGM_BUBBLE_M  = 0.25    # radio de seguridad alrededor del obstaculo mas cercano (m)
FGM_GAP_MIN_M = 0.6     # un rayo cuenta como "libre" si r > esto (m)
K_HEADING     = 0.8     # ganancia de giro hacia el hueco

CONTROL_HZ = 20.0
CMD_LISTEN_PORT = 5008     # comandos ARM/PAUSE/STOP/TOGGLE desde la laptop (control_teclas.py)
QR_HZ      = 5.0            # deteccion de QR limitada (cuida CPU de la Pi)
MAX_DV = 0.04              # suavizado (slew): cambio max de v por ciclo
MAX_DW = 0.20
DEAD_S = 0.7               # sin /scan por mas de esto -> parar
# ===============================================


class AutonomiaRobot(Node):
    def __init__(self):
        super().__init__("autonomia_robot")

        if USE_STAMPED:
            self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        else:
            # create3_repub escucha en best_effort -> publicamos best_effort para empatar
            self.pub = self.create_publisher(Twist, "/cmd_vel_unstamped", qos_profile_sensor_data)

        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.img_cb, qos_profile_sensor_data)
        if HAS_BUTTONS:
            self.create_subscription(InterfaceButtons, "/interface_buttons", self.btn_cb, qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.qr = cv2.QRCodeDetector()

        self.scan = None              # (ranges, amin, ainc)
        self.scan_t = 0.0
        self.checkpoints = set()
        self.armed = START_ARMED
        self.escape_dir = 0
        self.v_cur = 0.0
        self.w_cur = 0.0
        self._btn_prev = False
        self._last_qr = 0.0
        self._last_log = 0.0

        self.create_timer(1.0 / CONTROL_HZ, self.control_step)

        # Comandos desde la laptop (tecla) por UDP, ademas del boton fisico
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(("0.0.0.0", CMD_LISTEN_PORT))
        threading.Thread(target=self._cmd_listen, daemon=True).start()

        self.get_logger().info("=== autonomia_robot iniciada ===")
        if HAS_BUTTONS:
            self.get_logger().info("Pulsa el BOTON 1 del TurtleBot para ARMAR/PAUSAR.")
        else:
            self.get_logger().warn("Sin InterfaceButtons: usa START_ARMED=True para arrancar.")
        self.get_logger().info(f"Estado inicial: {'ARMADO' if self.armed else 'IDLE'}")

    # ---------------- Callbacks ----------------
    def scan_cb(self, msg: LaserScan):
        self.scan = (list(msg.ranges), msg.angle_min, msg.angle_increment)
        self.scan_t = time.time()

    def img_cb(self, msg: Image):
        now = time.time()
        if now - self._last_qr < 1.0 / QR_HZ:
            return
        self._last_qr = now
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            data, _, _ = self.qr.detectAndDecode(img)
            if data and data not in self.checkpoints:
                self.checkpoints.add(data)
                ts = time.strftime("%Y-%m-%d %H:%M:%S")
                line = f"{ts} | STAGE {STAGE} | CHECKPOINT {len(self.checkpoints)}/3 | QR='{data}'"
                self.get_logger().info(f">>> {line} <<<")
                try:
                    with open(QR_LOG_FILE, "a", encoding="utf-8") as f:
                        f.write(line + "\n")
                except Exception as e:
                    self.get_logger().error(f"No se pudo escribir log QR: {e}")
        except Exception as e:
            self.get_logger().error(f"img_cb: {e}")

    def btn_cb(self, msg: InterfaceButtons):
        pressed = bool(msg.button_1.is_pressed)
        if pressed and not self._btn_prev:        # flanco de subida
            self.armed = not self.armed
            self.get_logger().info(f">>> BOTON 1: {'ARMADO (corriendo)' if self.armed else 'PAUSA (idle)'} <<<")
        self._btn_prev = pressed

    def _cmd_listen(self):
        # Escucha teclas/comandos desde la laptop (control_teclas.py)
        while True:
            try:
                data, _ = self.cmd_sock.recvfrom(256)
                c = data.decode(errors="ignore").strip().upper()
                if c in ("ARM", "GO"):
                    self.armed = True
                elif c in ("PAUSE", "STOP", "IDLE"):
                    self.armed = False
                elif c == "TOGGLE":
                    self.armed = not self.armed
                else:
                    continue
                self.get_logger().info(f">>> CMD laptop: {c} -> {'ARMADO' if self.armed else 'IDLE'} <<<")
            except Exception as e:
                self.get_logger().error(f"cmd_listen: {e}")
                break

    # ---------------- Logica ----------------
    def sector_min(self, ranges, amin, ainc, center_deg, half_deg):
        center = math.radians(center_deg)
        half = math.radians(half_deg)
        best = R_MAX
        best_ang = None
        for i, r in enumerate(ranges):
            if not (R_MIN < r < R_MAX):
                continue
            a = amin + i * ainc
            d = math.atan2(math.sin(a - center), math.cos(a - center))
            if abs(d) <= half and r < best:
                best = r
                best_ang = math.degrees(a)
        return best, best_ang

    def decide(self, scan):
        # Follow-the-Gap: apunta al hueco mas grande del arco frontal (suave, sin stop-spin)
        ranges, amin, ainc = scan
        fr = math.radians(FRONT_DEG)
        arc = math.radians(FGM_ARC_DEG)

        angs, rs = [], []
        for i, r in enumerate(ranges):
            a = amin + i * ainc
            rel = math.atan2(math.sin(a - fr), math.cos(a - fr))  # angulo relativo al frente
            if abs(rel) <= arc:
                if r != r or r <= R_MIN:               # nan o muy cerca -> bloqueado
                    rr = 0.0
                elif r >= FGM_RMAX or math.isinf(r):    # lejos/invalido -> libre
                    rr = FGM_RMAX
                else:
                    rr = r
                angs.append(rel)
                rs.append(rr)
        if not rs:
            return 0.0, ANG   # sin datos en el arco -> gira a buscar

        order = sorted(range(len(angs)), key=lambda k: angs[k])
        angs = [angs[k] for k in order]
        rs   = [rs[k]   for k in order]

        # Burbuja de seguridad alrededor del punto mas cercano
        imin = min(range(len(rs)), key=lambda k: rs[k] if rs[k] > 0 else 9e9)
        rmin = rs[imin]
        if 0 < rmin < FGM_RMAX:
            bubble = math.atan2(FGM_BUBBLE_M, max(rmin, 0.05))
            for k in range(len(rs)):
                if abs(angs[k] - angs[imin]) <= bubble:
                    rs[k] = 0.0

        # Mayor hueco contiguo con r > umbral
        best_lo = best_hi = -1
        lo = None
        for k in range(len(rs)):
            if rs[k] > FGM_GAP_MIN_M:
                if lo is None:
                    lo = k
                if best_lo < 0 or (k - lo) > (best_hi - best_lo):
                    best_lo, best_hi = lo, k
            else:
                lo = None

        now = time.time()
        log = (now - self._last_log > 0.5)
        if log:
            self._last_log = now

        if best_lo < 0:
            # Encajonado (sin hueco): gira hacia el lado mas profundo
            ideep = max(range(len(rs)), key=lambda k: rs[k])
            d = 1.0 if angs[ideep] >= 0 else -1.0
            if log and SCAN_DEBUG:
                self.get_logger().info(f"[FGM] sin hueco -> giro {'IZQ' if d > 0 else 'DER'}")
            return 0.0, d * ANG

        # Objetivo: punto mas profundo dentro del mayor hueco
        itar = max(range(best_lo, best_hi + 1), key=lambda k: rs[k])
        heading = angs[itar]
        front = min((rs[k] for k in range(len(rs))
                     if abs(angs[k]) <= math.radians(FRONT_HALF_DEG) and rs[k] > 0), default=FGM_RMAX)
        w = max(-ANG, min(ANG, K_HEADING * heading))
        v = LIN * max(VMIN_FRAC, min(1.0, front / D_SLOW))
        if log and SCAN_DEBUG:
            self.get_logger().info(f"[FGM] heading={math.degrees(heading):.0f}deg front={front:.2f} v={v:.2f} w={w:.2f}")
        return v, w

    def control_step(self):
        scan = self.scan
        age = time.time() - self.scan_t

        if not self.armed:
            vt, wt = 0.0, 0.0
            if scan is not None and SCAN_DEBUG and (time.time() - self._last_log > 0.5):
                self._last_log = time.time()
                fr, _ = self.sector_min(scan[0], scan[1], scan[2], FRONT_DEG, FRONT_HALF_DEG)
                gmin, gang = self.sector_min(scan[0], scan[1], scan[2], FRONT_DEG, 180.0)
                self.get_logger().info(f"[IDLE/SCAN] front={fr:.2f} min_global={gmin:.2f}@{gang} (calibra FRONT_DEG)")
        elif scan is None or age > DEAD_S:
            vt, wt = 0.0, 0.0
        else:
            vt, wt = self.decide(scan)

        # Suavizado slew-rate
        self.v_cur += max(-MAX_DV, min(MAX_DV, vt - self.v_cur))
        self.w_cur += max(-MAX_DW, min(MAX_DW, wt - self.w_cur))

        if USE_STAMPED:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = self.v_cur
            m.twist.angular.z = self.w_cur
        else:
            m = Twist()
            m.linear.x = self.v_cur
            m.angular.z = self.w_cur
        self.pub.publish(m)


def main():
    rclpy.init()
    node = AutonomiaRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # parada final
    try:
        node.v_cur = node.w_cur = 0.0
        node.control_step()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
