#!/usr/bin/env python3
# =====================================================================================
# autonomia_v14.py -- CORRE EN LA RASPBERRY DEL ROBOT (nodo ROS2). Autonomia A BORDO.
# v14 = v13 con: (1) LIN_MAX 0.30->0.26 (un pelin menos de velocidad punta = mas control/roces suaves);
#       (2) PD de rumbo mas preciso: KP 0.9->1.1 (menos error de rumbo residual) + KD 0.30->0.40
#       (amortigua el sobrepaso que trae el KP mayor). Fallback: v13.
# v13 = v12 + modo PROBE (degradacion elegante): si lleva PROBE_AFTER seg atascado sin NINGUN hueco
#       seguro, baja el umbral a ROBOT_PASS_MIN e intenta el hueco mas justo a velocidad LENTA
#       (preciso y suave). Solo actua cuando esta atascado -> no afecta la navegacion normal. Fallback: v12.
# v12 = v11 con prueba de hueco DISTANCIA-CORRECTA (ancho fisico = cuerda 2*dmin*sin(dw/2)) en vez
#       de erosion por angulo fijo. Asi DESCUBRE pasillos angostos desde lejos y entra (antes los
#       rechazaba de lejos y retrocedia/chocaba). ROBOT_PASS=0.41 (diam real 0.346 + ~3cm/lado).
# v11 = v10 con UN cambio: SAFETY 0.05 -> 0.03 (acepta pasillos de ~4cm/lado que antes rechazaba;
#       sigue rechazando ~2-2.5cm). Solo afecta que huecos considera transitables. Fallback: v4/v10.
# v10 = v9 + LEY DE VELOCIDAD NUEVA: rapido cuando va RECTO hacia un frente despejado (aunque el
#       pasillo sea angosto), lento solo cuando GIRA/corrige. La cercania de los flancos YA NO
#       frena (solo empuja para centrar). Conserva ROTATE en curvas muy cerradas. Fallback: v4.
# v9 = v4 (curvas que funcionaban) con CAMBIOS MINIMOS:
#   - QUITA el disparo por proximidad (pegado) -> ya NO titubea en curvas.
#   - EVADE solo ante GOLPE REAL del bumper (zona del frame_id), breve, rear-safe, con enfriamiento.
#   - Repulsion lateral un poco mas fuerte (anti-roce) SIN frenar de mas.
#   Fallback: autonomia_v4.py.
# v4: DOS MODOS CLAROS Y DECIDIDOS (arregla el "izquierda-derecha-un paso" del v3).
#
# QUE ESTABA MAL EN v3 (lo que viste en pista):
#   v3 acoplaba velocidad y giro (v = LIN*f_clear*f_turn). Frente a una pared con salidas
#   a los lados pasaba: (a) el hueco saltaba entre izq y der -> w temblaba sin completar
#   giro; (b) como el angulo era moderado, f_turn NO llegaba a 0 -> seguia avanzando un
#   pelito. Resultado: zigzag + un paso, hasta clavarse en la pared, sin disparar nunca el
#   giro de busqueda. El LiDAR ya sensa 360, asi que ese baile NO sirve: es un bug.
#
# IDEA v4 (decidir, no titubear): SEPARAR en dos modos con HISTERESIS, sin acople raro.
#   * DRIVE  : el frente esta despejado (front > D_FREE). Avanza siguiendo el hueco con
#              correccion SUAVE (Kalman 1D + PD). La velocidad baja a 0 al acercarse a una
#              pared (se apaga en D_BLOCK), asi NO se mete reptando.
#   * ROTATE : el frente esta tapado (front < D_BLOCK) o golpe/ciego. Se DETIENE y gira EN
#              EL SITIO hacia el lado mas abierto, COMPROMETIDO con esa direccion (no la
#              cambia hasta despejar) -> mata el zigzag. Sale a DRIVE cuando front > D_FREE.
#   Histeresis: entra a ROTATE bajo D_BLOCK (0.55) y solo vuelve a DRIVE sobre D_FREE (0.80).
#   La franja 0.55-0.80 evita el parpadeo en el borde.
#
#   * TURN   : giro de esquina de 90 EXACTOS por /odom (lazo cerrado). Lo dispara un comando
#              UDP LEFT/RIGHT (lo usara el YOLO de senales, o pruebas manuales).
#
# Por que sigue habiendo Kalman+PD pero SOLO en DRIVE: ahi la correccion es fina y el
# suavizado ayuda; el problema nunca fue el filtro, fue el acople que hacia reptar. En
# ROTATE no se filtra: se gira decidido y punto.
#
# CALIBRA FRONT_DEG ANTES DE CONFIAR EN ESTO (usa diag_lidar.py o el log [IDLE]). Si el
# frente esta mal, el cono frontal mira a otro lado y "a veces se va de frente y choca".
#
# Signos (ROS REP-103): angular.z > 0 = IZQUIERDA (CCW); < 0 = DERECHA (CW).
# Correr:  export ROS_DOMAIN_ID=67 && python3 ~/autonomia_v14.py
# NO correr junto con otros nodos que publiquen cmd_vel.
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
USE_STAMPED = False     # ESTE robot NO fue reflasheado: se mueve con Twist en /cmd_vel_unstamped
                        # (lo escucha create3_repub, best_effort). /cmd_vel (TwistStamped) tiene 0
                        # suscriptores en este robot -> NO mueve. Verificar: ros2 topic info /cmd_vel -v
START_ARMED = False
STAGE       = 1
QR_LOG_FILE = "/home/ubuntu/checkpoints_log.txt"

# --- Velocidades ---
LIN    = 0.15            # (referencia v4)
LIN_MAX = 0.26           # v14: techo un pelin menor (mas control). Sube/baja para mas/menos velocidad.
W_ALIGN = 0.9            # v10: por encima de este giro, la velocidad cae a su piso
FLOOR_ALIGN = 0.25       # v10: fraccion minima de velocidad al girar fuerte
W_MAX  = 1.2            # tope de giro en DRIVE.
W_ROT  = 0.7            # v13: giro de escape mas suave (antes 1.0) -> no roza al salir de apretados.
LEFT_BIAS = 0.0

# --- Umbrales de modo (histeresis DRIVE <-> ROTATE) ---
D_BLOCK = 0.45          # frente < esto -> ROTATE (se detiene y gira). Antes de llegar a la pared.
D_FREE  = 0.65          # frente > esto -> vuelve a DRIVE.
D_SLOW  = 0.70          # arriba de esto, velocidad plena.
D_EMERG = 0.20          # frente < esto (o ciego) = peligro -> ROTATE inmediato.

# --- Hueco transitable ---
D_CLEAR    = 0.55       # un rayo es "libre" si supera esto.
ROBOT_PASS = 0.41       # v12: ancho minimo del hueco (diam 0.346 + ~3cm/lado). Acepta ~4cm/lado, rechaza ~2.5cm.
ROBOT_PASS_MIN = 0.37   # v13: umbral RELAJADO en modo PROBE (~1.2cm/lado). Solo si esta atascado.
PROBE_AFTER = 3.0       # v13: seg atascado en ROTATE sin hueco seguro antes de intentar el justo.
PROBE_V    = 0.06       # v13: velocidad LENTA al colarse por el hueco justo.

# --- CONCIENCIA DEL CUERPO (que no roce con los flancos) ---
ROBOT_RADIUS = 0.173    # v12: radio real (diametro medido 34.6 cm).
SAFETY       = 0.03     # v11: 3cm (antes 5). Acepta pasillos ~4cm/lado; rechaza ~2.5cm.
R_INFL       = ROBOT_RADIUS + SAFETY   # inflacion de obstaculos (espacio de configuracion).
SIDE_CENTER_DEG = 90.0  # sector lateral centrado en +-90.
SIDE_HALF_DEG   = 55.0  # ancho del sector lateral.
SIDE_AVOID      = 0.32  # v9: esquiva el flanco un pelin antes.
K_SIDE          = 1.5   # v9: repulsion lateral un poco mas fuerte (anti-roce).
BACKUP_T   = 0.6        # seg de retroceso tras un golpe (des-encajarse).
BACKUP_V   = -0.10      # velocidad de retroceso (m/s).
EVADE_T    = 0.5        # v9: duracion de evasion tras GOLPE real.
EVADE_COOLDOWN = 1.5    # v9: seg sin re-disparar evasion (evita oscilar).
FRONT_DEG       = -90.0 # CALIBRADO: caja al frente leyo ~-90 deg (verificado izq/der/frente OK).
FRONT_HALF_DEG  = 14.0  # medio cono frontal para decidir DRIVE/ROTATE.
SEARCH_HALF_DEG = 150.0 # busca huecos en +-150 deg (casi todo el frente y costados).
R_MIN, R_MAX    = 0.06, 12.0

# --- Control PD del rumbo en DRIVE (sin termino I) ---
KP_HEADING = 1.1   # v14: +precision (menos error de rumbo residual)
KD_HEADING = 0.40  # v14: +amortiguacion para el KP mayor (sin sobrepaso)
# Kalman 1D [theta, thetadot]
KF_R = 0.0030
KF_Q_TH = 0.0010
KF_Q_THD = 0.0200
STICK_W = 0.5           # histeresis al elegir hueco (pegajosidad al rumbo previo).

# --- Giro de esquina por /odom ---
TURN_STEP_DEG  = 90.0
KP_TURN        = 1.6
W_TURN_MAX     = 1.4
W_TURN_MIN     = 0.30
TURN_TOL_DEG   = 2.5
W_TURN_NOMINAL = 0.9    # fallback por tiempo si no hay /odom.

# --- Tiempos / suavizado ---
CONTROL_HZ = 20.0
MAX_DV = 0.04
MAX_DW = 0.50           # mas alto que v3: deja girar rapido (antes el giro no "arrancaba").
DEAD_S = 0.7
CMD_LISTEN_PORT = 5008
QR_HZ = 5.0
SCAN_DEBUG = True
# =====================================================================================


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class AutonomiaV14(Node):
    def __init__(self):
        super().__init__("autonomia_v14")

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
            self.create_subscription(HazardDetectionVector, "/_do_not_use/hazard_detection", self.hazard_cb, qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.qr = cv2.QRCodeDetector()

        self.scan = None
        self.scan_t = 0.0
        self.yaw = 0.0
        self.have_odom = False

        self.state = "DRIVE"             # DRIVE | ROTATE | TURN
        self.rot_dir = -1.0              # direccion COMPROMETIDA en ROTATE (-1 der, +1 izq)
        self.armed = START_ARMED
        self.v_cur = 0.0
        self.w_cur = 0.0
        self.bump_flag = False
        self._btn_prev = False
        self._last_qr = 0.0
        self._last_log = 0.0

        self.prev_heading = 0.0
        self.kf_x = [0.0, 0.0]
        self.kf_P = [[0.05, 0.0], [0.0, 0.5]]
        self.kf_ready = False

        self.turn_request = None
        self.turn_mode = None
        self.target_yaw = 0.0
        self.turn_t_end = 0.0
        self.turn_dir = 0.0
        self.t_backup = 0.0
        self.t_evade = 0.0
        self.t_evade_end = -10.0
        self.evade_dir = -1.0
        self.bump_side = 0.0
        self.t_state = 0.0

        self.checkpoints = set()
        self.create_timer(1.0 / CONTROL_HZ, self.control_step)

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(("0.0.0.0", CMD_LISTEN_PORT))
        threading.Thread(target=self._cmd_listen, daemon=True).start()

        self.get_logger().info("=== autonomia_v14 (DRIVE/ROTATE decididos + giro/odom) ===")
        self.get_logger().info(f"Estado inicial: {'ARMADO' if self.armed else 'IDLE'} | boton1 o tecla g.")

    # ------------------------------- Callbacks -------------------------------
    def scan_cb(self, msg):
        self.scan = (list(msg.ranges), msg.angle_min, msg.angle_increment)
        self.scan_t = time.time()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.have_odom = True

    def hazard_cb(self, msg):
        try:
            for d in msg.detections:
                if d.type == 1:
                    self.bump_flag = True
                    return
        except Exception:
            pass

    def img_cb(self, msg):
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
                    self.turn_request = math.radians(TURN_STEP_DEG)
                elif c in ("RIGHT", "R", "R90"):
                    self.turn_request = -math.radians(TURN_STEP_DEG)
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
        th, thd = self.kf_x
        p00, p01 = self.kf_P[0]
        p10, p11 = self.kf_P[1]
        th_p = th + thd * dt
        p00p = p00 + dt * (p10 + p01) + dt * dt * p11 + KF_Q_TH
        p01p = p01 + dt * p11
        p10p = p10 + dt * p11
        p11p = p11 + KF_Q_THD
        S = p00p + KF_R
        k0 = p00p / S
        k1 = p10p / S
        y = wrap(z - th_p)
        th_n = th_p + k0 * y
        thd_n = thd + k1 * y
        self.kf_x = [th_n, thd_n]
        self.kf_P = [[(1 - k0) * p00p, (1 - k0) * p01p],
                     [-k1 * p00p + p10p, -k1 * p01p + p11p]]
        return th_n, thd_n

    # ------------------------------- LiDAR helpers -------------------------------
    def _rel(self, a):
        return wrap(a - math.radians(FRONT_DEG))

    def sector_min(self, scan, center_rad, half_rad):
        ranges, amin, ainc = scan
        best = R_MAX
        for i, r in enumerate(ranges):
            if not (R_MIN < r < R_MAX):
                continue
            if abs(wrap(self._rel(amin + i * ainc) - center_rad)) <= half_rad and r < best:
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

    def mejor_hueco(self, scan, use_stick=True, pass_thr=None):
        # Devuelve (rumbo, dist) del mejor hueco que CABE el robot, o None.
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
        thr = ROBOT_PASS if pass_thr is None else pass_thr
        mejor = None
        n = len(pts); i = 0
        while i < n:
            if pts[i][1] > D_CLEAR:
                j = i; dmin = pts[i][1]
                while j + 1 < n and pts[j + 1][1] > D_CLEAR:
                    j += 1; dmin = min(dmin, pts[j][1])
                a_lo, a_hi = pts[i][0], pts[j][0]
                dw = a_hi - a_lo
                # ANCHO FISICO del hueco (distancia-correcto): cuerda = 2*dmin*sin(dw/2).
                # dmin (rayo libre mas cercano) ~ distancia a la BOCA del hueco -> da los metros
                # reales aunque el hueco se vea de lejos: asi lo DESCUBRE sin tener que pegarse.
                ancho = 2.0 * dmin * math.sin(max(dw, 0.0) / 2.0)
                if ancho >= thr:
                    centro = 0.5 * (a_lo + a_hi)        # apunta al centro del hueco
                    costo = abs(centro) - (0.08 if centro > 0 else 0.0)
                    if use_stick:
                        costo += STICK_W * abs(wrap(centro - self.prev_heading))
                    if mejor is None or costo < mejor[0]:
                        mejor = (costo, centro, dmin)
                i = j + 1
            else:
                i += 1
        return None if mejor is None else (mejor[1], mejor[2])

    def _enter(self, st):
        self.state = st
        self.t_state = time.time()      # v13 fix: marca cuando entro al estado (para PROBE)
        if st != "DRIVE":
            self.kf_ready = False

    def _log(self, msg):
        now = time.time()
        if SCAN_DEBUG and (now - self._last_log > 0.5):
            self._last_log = now
            self.get_logger().info(msg)

    def _enter_rotate(self, scan):
        # Decide UNA vez hacia que lado girar: hacia el mejor hueco (camino corto). Si no
        # hay hueco, gira a la derecha a buscar. Queda COMPROMETIDO hasta despejar el frente.
        h = self.mejor_hueco(scan, use_stick=False)
        if h is not None:
            self.rot_dir = 1.0 if h[0] >= 0 else -1.0
        else:
            self.rot_dir = -1.0
        self._enter("ROTATE")
        self.get_logger().info(f">>> ROTATE comprometido a {'IZQ' if self.rot_dir > 0 else 'DER'}")

    def _start_turn(self):
        delta = self.turn_request
        self.turn_request = None
        if self.have_odom:
            self.turn_mode = "odom"
            self.target_yaw = wrap(self.yaw + delta)
        else:
            self.turn_mode = "time"
            self.turn_dir = 1.0 if delta >= 0 else -1.0
            self.turn_t_end = time.time() + abs(delta) / W_TURN_NOMINAL
        self._enter("TURN")
        self.get_logger().info(f">>> GIRO {'IZQ' if delta > 0 else 'DER'} "
                               f"{abs(math.degrees(delta)):.0f}deg ({self.turn_mode})")

    # ------------------------------- Decision -------------------------------
    def decide(self, scan, dt):
        # 0) Giro de esquina por comando (prioridad).
        if self.turn_request is not None and self.state != "TURN":
            self._start_turn()
        if self.state == "TURN":
            if self.turn_mode == "odom":
                err = wrap(self.target_yaw - self.yaw)
                if abs(err) < math.radians(TURN_TOL_DEG):
                    self._enter("DRIVE"); return 0.0, 0.0
                w = clamp(KP_TURN * err, -W_TURN_MAX, W_TURN_MAX)
                if abs(w) < W_TURN_MIN:
                    w = math.copysign(W_TURN_MIN, w)
                return 0.0, w
            else:
                if time.time() >= self.turn_t_end:
                    self._enter("DRIVE"); return 0.0, 0.0
                return 0.0, self.turn_dir * W_TURN_NOMINAL

        # EVADE solo ante GOLPE REAL (no por proximidad LiDAR -> NO titubea en curvas).
        now = time.time()
        if self.bump_flag and self.state != "EVADE" and (now - self.t_evade_end) > EVADE_COOLDOWN:
            self.bump_flag = False
            if self.bump_side != 0.0:
                self.evade_dir = -self.bump_side          # gira AWAY del lado golpeado
            else:
                li = self.sector_min(scan,  math.radians(60.0), math.radians(40.0))
                ri = self.sector_min(scan, -math.radians(60.0), math.radians(40.0))
                self.evade_dir = 1.0 if li >= ri else -1.0
            self.t_evade = now
            self._enter("EVADE")
            self.get_logger().info(f"[EVADE] golpe -> gira {'IZQ' if self.evade_dir>0 else 'DER'}")
        else:
            self.bump_flag = False
        if self.state == "EVADE":
            if now - self.t_evade < EVADE_T:
                rear = self.sector_min(scan, math.radians(180.0), math.radians(40.0))
                return (BACKUP_V if rear > 0.30 else 0.0), self.evade_dir * W_ROT
            self.t_evade_end = now
            self._enter("DRIVE")

        front = self.sector_min(scan, 0.0, math.radians(FRONT_HALF_DEG))
        blind = self.frente_ciego(scan)

        # 1) Transiciones DRIVE <-> ROTATE con HISTERESIS.
        if self.state != "ROTATE" and (blind or front < D_BLOCK):
            self._enter_rotate(scan)
        elif self.state == "ROTATE" and (not blind) and front > D_FREE:
            self._enter("DRIVE")

        # 2) ROTATE: gira en el sitio, COMPROMETIDO, hasta despejar. Sin reptar.
        if self.state == "ROTATE":
            if (time.time() - self.t_state) > PROBE_AFTER and \
               self.mejor_hueco(scan, use_stick=False, pass_thr=ROBOT_PASS_MIN) is not None:
                self._enter("PROBE")        # atascado sin salida segura -> intenta el hueco justo, lento
            else:
                self._log(f"[ROTATE] front={front:.2f} dir={'IZQ' if self.rot_dir > 0 else 'DER'}")
                return 0.0, self.rot_dir * W_ROT

        if self.state == "PROBE":
            hp = self.mejor_hueco(scan, use_stick=True, pass_thr=ROBOT_PASS_MIN)
            if hp is None:
                self._enter_rotate(scan)
                return 0.0, self.rot_dir * W_ROT
            rumbo_z, dmin = hp
            self.prev_heading = rumbo_z
            if not self.kf_ready:
                theta_f, thetadot_f = rumbo_z, 0.0
                self.kf_reset(rumbo_z)
            else:
                theta_f, thetadot_f = self.kf_step(rumbo_z, dt)
            w = clamp(KP_HEADING * theta_f + KD_HEADING * thetadot_f, -W_MAX, W_MAX)
            clear_dir = self.sector_min(scan, theta_f, math.radians(FRONT_HALF_DEG))
            if clear_dir > D_FREE and abs(theta_f) < math.radians(12):
                self._enter("DRIVE")        # ya alineado y despejado -> navegacion normal
            self._log(f"[PROBE lento] rumbo={math.degrees(theta_f):.0f} clear={clear_dir:.2f} w={w:.2f}")
            return PROBE_V, w

        # 3) DRIVE: frente despejado -> avanza siguiendo el hueco con correccion suave PD.
        h = self.mejor_hueco(scan, use_stick=True)
        if h is None:
            # raro estando despejado; por seguridad, a ROTATE.
            self._enter_rotate(scan)
            return 0.0, self.rot_dir * W_ROT
        rumbo_z, dmin = h
        self.prev_heading = rumbo_z

        if not self.kf_ready:
            theta_f, thetadot_f = rumbo_z, 0.0
            self.kf_reset(rumbo_z)
        else:
            theta_f, thetadot_f = self.kf_step(rumbo_z, dt)

        # REPULSION LATERAL: empuja lejos del costado mas cercano (evita rozar con el cuerpo).
        left_min  = self.sector_min(scan,  math.radians(SIDE_CENTER_DEG), math.radians(SIDE_HALF_DEG))
        right_min = self.sector_min(scan, -math.radians(SIDE_CENTER_DEG), math.radians(SIDE_HALF_DEG))
        rep = 0.0
        if left_min  < SIDE_AVOID: rep -= K_SIDE * (SIDE_AVOID - left_min)    # algo a la IZQ -> gira DER
        if right_min < SIDE_AVOID: rep += K_SIDE * (SIDE_AVOID - right_min)   # algo a la DER -> gira IZQ
        w = clamp(KP_HEADING * theta_f + KD_HEADING * thetadot_f + rep + LEFT_BIAS, -W_MAX, W_MAX)
        # velocidad: baja con el frente Y con el costado mas cercano (mas lento si algo roza).
        # v10: velocidad por DIRECCION DE VIAJE (holgura hacia donde apunta) + esfuerzo de giro.
        clear_dir = self.sector_min(scan, theta_f, math.radians(FRONT_HALF_DEG))   # que tan lejos puedo ir asi
        f_front = clamp((clear_dir - D_BLOCK) / (D_SLOW - D_BLOCK), 0.0, 1.0)
        f_align = clamp(1.0 - abs(w) / W_ALIGN, FLOOR_ALIGN, 1.0)                   # lento si gira/corrige
        v = LIN_MAX * f_front * f_align
        self._log(f"[DRIVE] clear={clear_dir:.2f} izq={left_min:.2f} der={right_min:.2f} "
                  f"rumbo={math.degrees(theta_f):.0f} v={v:.2f} w={w:.2f}")
        return v, w

    # ------------------------------- Bucle de control -------------------------------
    def control_step(self):
        if not rclpy.ok():
            return
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
            m.header.frame_id = "base_link"
            m.twist.linear.x = self.v_cur
            m.twist.angular.z = self.w_cur
        else:
            m = Twist()
            m.linear.x = self.v_cur
            m.angular.z = self.w_cur
        try:
            self.pub.publish(m)
        except Exception:
            pass


def main():
    rclpy.init()
    node = AutonomiaV14()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        try:
            node.get_logger().error(f"spin termino: {e}")
        except Exception:
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
