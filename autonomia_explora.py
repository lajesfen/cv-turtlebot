#!/usr/bin/env python3
# =====================================================================================
# autonomia_explora.py  -- CORRE EN LA RASPBERRY DEL ROBOT (nodo ROS2). Autonomia A BORDO.
# v3: seguir-el-hueco continuo + Kalman 1D del rumbo + control PD (sin I) + histeresis
#     + giros de esquina por /odom en lazo cerrado.
#
# PIPELINE DE DECISION (esto es lo que cambia respecto a v2):
#   LiDAR /scan ->[elige hueco que cabe + histeresis]-> rumbo crudo z
#               ->[Kalman 1D: estima rumbo limpio y su velocidad]-> (theta_f, thetadot_f)
#               ->[control PD: w = Kp*theta_f + Kd*thetadot_f]-> w (giro)
#               ->[slew]-> /cmd_vel_unstamped
#
#   Por que Kalman y no solo bajar ruido a mano: el rumbo del hueco (centroide del
#   arco libre del LiDAR) es RUIDOSO y SALTARIN. Un control PD necesita la derivada del
#   error, y derivar una senal ruidosa amplifica el ruido (el robot tiembla). El filtro
#   de Kalman es el estimador optimo recursivo (predice con un modelo de velocidad
#   constante y corrige con la medicion) y nos entrega de regalo la derivada YA SUAVIZADA,
#   thetadot_f, sin diferenciacion numerica. Ref: Thrun, Burgard & Fox, "Probabilistic
#   Robotics" (MIT Press), cap. 3; Welch & Bishop, "An Introduction to the Kalman Filter".
#
#   Por que PD y NO PID: el termino integral I aqui estorba (windup al saturar el giro,
#   y el objetivo -el hueco- salta de forma discreta, asi que la I integra "el mundo
#   cambio", no un sesgo a corregir). La D amortigua la oscilacion. Ref: Astrom & Murray,
#   "Feedback Systems" (Princeton), cap. 10. Para SEGUIR-HUECO va PD; la I tendria sentido
#   en SEGUIR-PARED a distancia constante (no es nuestro caso).
#
#   Histeresis: no cambiamos de hueco objetivo salvo que otro sea claramente mejor (coste
#   de "pegajosidad" al rumbo previo). Mata el flip-flop entre dos huecos parecidos.
#
#   Giros por /odom: girar por tiempo SUB-rota (el Create3 topa velocidad y tiene rampa,
#   asi que integral(w dt) < w*t). Giramos en LAZO CERRADO hasta que el yaw de /odom
#   alcanza el objetivo. Se dispara por comando (UDP: LEFT/RIGHT) -> lo usara la capa
#   semantica (YOLO de senales) o para pruebas. Ref del problema: HANDOFF seccion 7.
#
# Signos (ROS REP-103): angular.z > 0 = IZQUIERDA (CCW); < 0 = DERECHA (CW).
# Correr:  export ROS_DOMAIN_ID=67 && python3 ~/autonomia_explora.py
# NO correr junto con autonomia_robot.py / recibidor_control.py (todos publican cmd_vel).
# =====================================================================================
import time
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2

try:
    from irobot_create_msgs.msg import InterfaceButtons
    HAS_BUTTONS = True
except Exception:
    HAS_BUTTONS = False

try:
    from irobot_create_msgs.msg import HazardDetectionVector
    HAS_HAZARD = True
except Exception:
    HAS_HAZARD = False

# ============================== CONFIG (m, rad/s, grados) =============================
USE_STAMPED = False
START_ARMED = False
STAGE       = 1
QR_LOG_FILE = "/home/ubuntu/checkpoints_log.txt"

# --- Velocidades ---
LIN      = 0.15          # crucero (m/s).
W_MAX    = 1.2           # tope de giro en crucero (rad/s).
W_SEARCH = 0.8           # giro en el sitio cuando no hay hueco de frente (DERECHA).
LEFT_BIAS = 0.0          # sesgo fijo a la izquierda (0 = ninguno).

# --- Control PD del rumbo (sin termino I) ---
KP_HEADING = 1.0         # proporcional: gira segun cuanto este desviado el hueco.
KD_HEADING = 0.35        # derivativo: amortigua; usa la velocidad del Kalman (no diferencia cruda).

# --- Kalman 1D del rumbo (modelo velocidad-constante: estado [theta, thetadot]) ---
KF_R       = 0.0030      # varianza de la MEDICION (rumbo del hueco). Sube si la cam/LiDAR ruidoso.
KF_Q_TH    = 0.0010      # ruido de proceso en theta.
KF_Q_THD   = 0.0200      # ruido de proceso en thetadot (cuanto dejamos que cambie el giro).

# --- Histeresis al elegir hueco ---
STICK_W    = 0.5         # coste por alejarse del rumbo elegido el frame anterior (pegajosidad).

# --- Que tan "transitable" es un rumbo ---
D_CLEAR   = 0.70         # un rayo cuenta como "libre" si su distancia supera esto.
D_SLOW    = 0.90         # a partir de aqui la velocidad escala hacia abajo.
D_EMERG   = 0.18         # < esto de frente = emergencia (casi el limite fisico del LiDAR).
ROBOT_PASS = 0.42        # ancho libre minimo para que el robot quepa (TB4 ~0.34 + margen).
TURN_FULL_DEG = 55.0     # si el hueco pide >= este giro, la velocidad cae a ~0 (gira en sitio).

# --- Geometria del cono ---
FRONT_DEG       = 0.0    # CALIBRAR: que angulo del laser es el frente (ver receta).
FRONT_HALF_DEG  = 20.0   # cono de "panico/emergencia" justo al frente.
SEARCH_HALF_DEG = 120.0  # busco huecos en +-120 deg.
R_MIN, R_MAX    = 0.06, 12.0

# --- Giro de esquina por odometria (lazo cerrado) ---
TURN_STEP_DEG   = 90.0   # cuanto gira un comando LEFT/RIGHT.
KP_TURN         = 1.6    # ganancia P del giro por yaw.
W_TURN_MAX      = 1.4    # tope de giro durante un giro de esquina.
W_TURN_MIN      = 0.30   # piso para que no se estanque cerca del objetivo.
TURN_TOL_DEG    = 2.5    # tolerancia para dar por terminado el giro.
W_TURN_NOMINAL  = 0.9    # fallback por tiempo si NO hay /odom (menos preciso).

# --- Tiempos / suavizado ---
CONTROL_HZ = 20.0
MAX_DV     = 0.04
MAX_DW     = 0.30
DEAD_S     = 0.7
CMD_LISTEN_PORT = 5008
QR_HZ = 5.0
SCAN_DEBUG = True
# =====================================================================================


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class AutonomiaExplora(Node):
    def __init__(self):
        super().__init__("autonomia_explora")

        if USE_STAMPED:
            self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        else:
            self.pub = self.create_publisher(Twist, "/cmd_vel_unstamped", qos_profile_sensor_data)

        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.img_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_cb, qos_profile_sensor_data)
        if HAS_BUTTONS:
            self.create_subscription(InterfaceButtons, "/interface_buttons", self.btn_cb, qos_profile_sensor_data)
        if HAS_HAZARD:
            self.create_subscription(HazardDetectionVector, "/hazard_detection", self.hazard_cb, qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.qr = cv2.QRCodeDetector()

        self.scan = None
        self.scan_t = 0.0
        self.yaw = 0.0
        self.have_odom = False

        self.state = "DRIVE"             # DRIVE | BUSCA | TURN
        self.armed = START_ARMED
        self.v_cur = 0.0
        self.w_cur = 0.0
        self.t_state = time.time()
        self.bump_flag = False
        self._btn_prev = False
        self._last_qr = 0.0
        self._last_log = 0.0

        # Histeresis
        self.prev_heading = 0.0

        # Kalman 1D: x=[theta, thetadot], P 2x2. Se (re)inicia al entrar a DRIVE.
        self.kf_x = [0.0, 0.0]
        self.kf_P = [[0.05, 0.0], [0.0, 0.5]]
        self.kf_ready = False

        # Giro de esquina
        self.turn_request = None         # rad pedidos (lo setea el hilo UDP)
        self.turn_mode = None            # "odom" | "time"
        self.target_yaw = 0.0
        self.turn_t_end = 0.0
        self.turn_dir = 0.0

        self.checkpoints = set()
        self.create_timer(1.0 / CONTROL_HZ, self.control_step)

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(("0.0.0.0", CMD_LISTEN_PORT))
        threading.Thread(target=self._cmd_listen, daemon=True).start()

        self.get_logger().info("=== autonomia_explora v3 (Kalman+PD+histeresis+giro/odom) ===")
        self.get_logger().info(f"Estado inicial: {'ARMADO' if self.armed else 'IDLE'} | boton1 o tecla g.")

    # ------------------------------- Callbacks -------------------------------
    def scan_cb(self, msg: LaserScan):
        self.scan = (list(msg.ranges), msg.angle_min, msg.angle_increment)
        self.scan_t = time.time()

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.have_odom = True

    def hazard_cb(self, msg):
        try:
            for d in msg.detections:
                if d.type == 1:                       # 1 = BUMP
                    self.bump_flag = True
                    return
        except Exception:
            pass

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
                with open(QR_LOG_FILE, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
        except Exception as e:
            self.get_logger().error(f"img_cb: {e}")

    def btn_cb(self, msg):
        pressed = bool(msg.button_1.is_pressed)
        if pressed and not self._btn_prev:
            self.armed = not self.armed
            self.get_logger().info(f">>> BOTON 1: {'ARMADO' if self.armed else 'PAUSA'} <<<")
        self._btn_prev = pressed

    def _cmd_listen(self):
        while True:
            try:
                data, _ = self.cmd_sock.recvfrom(256)
                c = data.decode(errors="ignore").strip().upper()
                if c in ("ARM", "GO"):
                    self.armed = True; self._enter("DRIVE")
                elif c in ("PAUSE", "STOP", "IDLE"):
                    self.armed = False
                elif c == "TOGGLE":
                    self.armed = not self.armed
                elif c in ("LEFT", "L", "L90"):
                    self.turn_request = math.radians(TURN_STEP_DEG)     # +90 = IZQUIERDA
                elif c in ("RIGHT", "R", "R90"):
                    self.turn_request = -math.radians(TURN_STEP_DEG)    # -90 = DERECHA
                else:
                    continue
                self.get_logger().info(f">>> CMD laptop: {c}")
            except Exception as e:
                self.get_logger().error(f"cmd_listen: {e}")
                break

    # ------------------------------- Kalman 1D -------------------------------
    def kf_reset(self, theta0):
        self.kf_x = [theta0, 0.0]
        self.kf_P = [[0.05, 0.0], [0.0, 0.5]]
        self.kf_ready = True

    def kf_step(self, z, dt):
        # PREDICCION (modelo velocidad-constante): theta += thetadot*dt
        th, thd = self.kf_x
        p00, p01 = self.kf_P[0]
        p10, p11 = self.kf_P[1]
        th_p  = th + thd * dt
        thd_p = thd
        # P_pred = F P F^T + Q  (desarrollado a mano para F=[[1,dt],[0,1]])
        p00p = p00 + dt * (p10 + p01) + dt * dt * p11 + KF_Q_TH
        p01p = p01 + dt * p11
        p10p = p10 + dt * p11
        p11p = p11 + KF_Q_THD
        # CORRECCION con la medicion z del rumbo (H = [1, 0])
        S  = p00p + KF_R
        k0 = p00p / S
        k1 = p10p / S
        y  = wrap(z - th_p)                 # innovacion (con wrap por si cruza +-pi)
        th_n  = th_p + k0 * y
        thd_n = thd_p + k1 * y
        self.kf_x = [th_n, thd_n]
        self.kf_P = [[(1 - k0) * p00p, (1 - k0) * p01p],
                     [-k1 * p00p + p10p, -k1 * p01p + p11p]]
        return th_n, thd_n

    # ------------------------------- LiDAR helpers -------------------------------
    def _rel(self, a):
        fr = math.radians(FRONT_DEG)
        return math.atan2(math.sin(a - fr), math.cos(a - fr))

    def sector_min(self, scan, center_rad, half_rad):
        ranges, amin, ainc = scan
        best = R_MAX
        for i, r in enumerate(ranges):
            if not (R_MIN < r < R_MAX):
                continue
            d = wrap(self._rel(amin + i * ainc) - center_rad)
            if abs(d) <= half_rad and r < best:
                best = r
        return best

    def frente_ciego(self, scan):
        ranges, amin, ainc = scan
        half = math.radians(FRONT_HALF_DEG)
        tot = inv = 0
        for i, r in enumerate(ranges):
            if abs(self._rel(amin + i * ainc)) <= half:
                tot += 1
                if (r != r) or (r <= R_MIN) or math.isinf(r):
                    inv += 1
        return tot > 0 and (inv / tot) > 0.5

    def mejor_hueco(self, scan):
        # Devuelve (rumbo_rad, dist_min) del mejor hueco transitable, con HISTERESIS:
        # el coste suma |centro| (girar poco), un bono si va a la izquierda, y un castigo
        # por alejarse del rumbo elegido el frame anterior (pegajosidad STICK_W).
        ranges, amin, ainc = scan
        search = math.radians(SEARCH_HALF_DEG)

        pts = []
        for i, r in enumerate(ranges):
            rel = self._rel(amin + i * ainc)
            if abs(rel) > search:
                continue
            if (r != r) or math.isinf(r):
                d = R_MAX
            elif r <= R_MIN:
                d = 0.0
            else:
                d = r
            pts.append((rel, d))
        if not pts:
            return None
        pts.sort(key=lambda p: p[0])

        mejor = None
        n = len(pts)
        i = 0
        while i < n:
            if pts[i][1] > D_CLEAR:
                j = i
                dmin = pts[i][1]
                while j + 1 < n and pts[j + 1][1] > D_CLEAR:
                    j += 1
                    dmin = min(dmin, pts[j][1])
                dw = pts[j][0] - pts[i][0]
                cuerda = 2.0 * dmin * math.sin(max(dw, 0.0) / 2.0)   # cabe el robot?
                if cuerda >= ROBOT_PASS:
                    centro = 0.5 * (pts[i][0] + pts[j][0])
                    costo = (abs(centro)
                             - (0.08 if centro > 0 else 0.0)             # empate -> IZQ
                             + STICK_W * abs(wrap(centro - self.prev_heading)))  # histeresis
                    if mejor is None or costo < mejor[0]:
                        mejor = (costo, centro, dmin)
                i = j + 1
            else:
                i += 1
        if mejor is None:
            return None
        return mejor[1], mejor[2]

    def _enter(self, st):
        self.state = st
        self.t_state = time.time()
        if st != "DRIVE":
            self.kf_ready = False        # al volver a DRIVE, el Kalman se reinicia a la medicion

    def _log(self, msg):
        now = time.time()
        if SCAN_DEBUG and (now - self._last_log > 0.5):
            self._last_log = now
            self.get_logger().info(msg)

    def _start_turn(self):
        delta = self.turn_request
        self.turn_request = None
        if self.have_odom:
            self.turn_mode = "odom"
            self.target_yaw = wrap(self.yaw + delta)
        else:
            self.turn_mode = "time"      # fallback menos preciso
            self.turn_dir = 1.0 if delta >= 0 else -1.0
            self.turn_t_end = time.time() + abs(delta) / W_TURN_NOMINAL
        self._enter("TURN")
        self.get_logger().info(f">>> GIRO {'IZQ' if delta>0 else 'DER'} "
                               f"{abs(math.degrees(delta)):.0f}deg ({self.turn_mode})")

    # ------------------------------- Decision -------------------------------
    def decide(self, scan, dt):
        # 0) Comando de giro pendiente -> arranca giro de esquina (tiene prioridad).
        if self.turn_request is not None and self.state != "TURN":
            self._start_turn()

        # 1) GIRO de esquina en lazo cerrado por /odom (o por tiempo si no hay odom).
        if self.state == "TURN":
            if self.turn_mode == "odom":
                err = wrap(self.target_yaw - self.yaw)
                if abs(err) < math.radians(TURN_TOL_DEG):
                    self._enter("DRIVE")
                    return 0.0, 0.0
                w = clamp(KP_TURN * err, -W_TURN_MAX, W_TURN_MAX)
                if abs(w) < W_TURN_MIN:                 # piso para no estancarse
                    w = math.copysign(W_TURN_MIN, w)
                return 0.0, w
            else:
                if time.time() >= self.turn_t_end:
                    self._enter("DRIVE")
                    return 0.0, 0.0
                return 0.0, self.turn_dir * W_TURN_NOMINAL

        # 2) Emergencia / golpe: gira en el sitio a la DERECHA (no retrocede).
        emergencia = self.frente_ciego(scan) or \
                     self.sector_min(scan, 0.0, math.radians(FRONT_HALF_DEG)) < D_EMERG
        if self.bump_flag:
            self.bump_flag = False
            emergencia = True
        if emergencia:
            if self.state != "BUSCA":
                self._enter("BUSCA")
            self._log("[EMERG] obstaculo pegado/golpe -> giro derecha en el sitio")
            return 0.0, -W_SEARCH

        # 3) Elegir hueco (con histeresis).
        hueco = self.mejor_hueco(scan)
        if hueco is None:
            if self.state != "BUSCA":
                self._enter("BUSCA")
            self._log("[BUSCA] sin hueco de frente -> giro derecha en el sitio")
            return 0.0, -W_SEARCH

        if self.state != "DRIVE":
            self._enter("DRIVE")
        rumbo_z, dmin = hueco
        self.prev_heading = rumbo_z                      # memoria para la histeresis

        # 4) Kalman 1D: rumbo limpio + su velocidad (para la D sin diferenciar ruido).
        if not self.kf_ready:
            theta_f, thetadot_f = rumbo_z, 0.0
            self.kf_reset(rumbo_z)
        else:
            theta_f, thetadot_f = self.kf_step(rumbo_z, dt)

        # 5) Control PD (sin I): w = Kp*theta + Kd*thetadot. La D amortigua la oscilacion.
        w = KP_HEADING * theta_f + KD_HEADING * thetadot_f + LEFT_BIAS
        w = clamp(w, -W_MAX, W_MAX)

        # 6) Velocidad acoplada al giro: baja en curvas cerradas (curva y no se pasa).
        clear   = self.sector_min(scan, theta_f, math.radians(FRONT_HALF_DEG))
        f_clear = clamp((clear - D_EMERG) / (D_SLOW - D_EMERG), 0.0, 1.0)
        f_turn  = clamp(1.0 - abs(theta_f) / math.radians(TURN_FULL_DEG), 0.0, 1.0)
        v = LIN * f_clear * f_turn
        self._log(f"[DRIVE] z={math.degrees(rumbo_z):.0f} thf={math.degrees(theta_f):.0f} "
                  f"thd={thetadot_f:.2f} dmin={dmin:.2f} v={v:.2f} w={w:.2f}")
        return v, w

    # ------------------------------- Bucle de control -------------------------------
    def control_step(self):
        scan = self.scan
        age = time.time() - self.scan_t
        dt = 1.0 / CONTROL_HZ

        if not self.armed:
            vt, wt = 0.0, 0.0
            if scan is not None:
                ranges, amin, ainc = scan
                gmin, gang = R_MAX, None
                for i, r in enumerate(ranges):
                    if R_MIN < r < R_MAX and r < gmin:
                        gmin = r; gang = math.degrees(amin + i * ainc)
                self._log(f"[IDLE] min_global={gmin:.2f} @ {gang:.0f} deg (pon FRONT_DEG = ese angulo)")
        elif scan is None or age > DEAD_S:
            vt, wt = 0.0, 0.0
        else:
            vt, wt = self.decide(scan, dt)

        self.v_cur += clamp(vt - self.v_cur, -MAX_DV, MAX_DV)
        self.w_cur += clamp(wt - self.w_cur, -MAX_DW, MAX_DW)

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
    node = AutonomiaExplora()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.v_cur = node.w_cur = 0.0
        node.control_step()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
