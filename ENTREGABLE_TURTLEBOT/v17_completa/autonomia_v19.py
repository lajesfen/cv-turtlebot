#!/usr/bin/env python3
# =====================================================================================
# autonomia_v19.py -- CORRE EN LA RASPBERRY DEL ROBOT (nodo ROS2). Autonomia A BORDO.
# v19 = v18 + (1) senal: si solo hay via al lado CONTRARIO, gira a BUSCAR el lado de la senal (puede dar la vuelta);
#       (2) repulsion de CUARTO-FRONTAL anti-roce de esquina al girar.
# v18 = v17.13 + PUERTA REAL solo en la eleccion GEOMETRICA (mejor_hueco) y solo en arcos ANGOSTOS.
#       NO toca gaps_list ni la senal (YOLO sigue igual que v17). Objetivo: no clavarse en esquina-en-punta / mini-huecos.
# v17 = v16 + MEMORIA de trayectoria (rejilla de 'migas de pan' en /odom, anti-loop) + fixes v17.1:
#       (a) bumper lee header.frame_id -> evade al lado REAL del golpe; (b) la senal expira por
#       TIEMPO (SIGN_TTL) Y por DISTANCIA (SIGN_MAX_DIST) -> anti-fantasma, cubre el cooldown del YOLO;
#       (c) PROBE_AFTER 3.0->1.5s (rescate rapido); (d) borra memoria al registrar cada checkpoint QR.
#       Fallback: v14 (misma nav, sin memoria ni senales).
# v17.3 (merge de la otra cuenta) = consciencia de cuerpo (ROBOT_PASS 0.43, SIDE_AVOID 0.38, K_SIDE 2.0,
#       KD_HEADING 0.55, STICK_W 0.8, EVADE_T 0.8) + _check_stall (atasco por odom -> EVADE). USE_STAMPED=True.
# v16 (corregido) = EXPLORE que se MUEVE + fixes: (a) bifurcacion en Y obedece la senal en DRIVE
#       (>=2 huecos -> elige el lado de la senal); (b) EXPLORE decrece velocidad suave (no escalon);
#       (c) repulsion en EXPLORE ensancha pero NO invierte el giro comprometido. STOP de senal usa SSTOP.
# v16 = v15 + EXPLORE que se MUEVE (wall-follow lento) en vez de pivotar en el sitio.
#       Motivo: pivotar quieto NO revela huecos (el LiDAR ya ve 360 desde ese punto); para
#       asomarse tras una esquina o la TAPA de una caja hay que TRASLADARSE. Ahora, cuando el
#       frente se tapa, avanza lento rodeando (direccion comprometida + repulsion lateral),
#       con freno frontal solo si el frente esta pegadisimo (< D_EMERG). Margen minimo del
#       modo PROBE a 1.5 cm/lado (ROBOT_PASS_MIN=0.376). Fallback: v15.
# v15 = v14 + senales YOLO por BUFFER (se consultan al decidir girar, no por distancia fija)
#       + 'stop' que frena 2s solo. Fallback: v14.
# v14 = v13 con: (1) LIN_MAX 0.30->0.26; (2) PD de rumbo KP 1.1 + KD 0.40.
# v13 = v12 + modo PROBE (si atascado sin hueco seguro, baja umbral e intenta el justo, lento).
# v12 = prueba de hueco DISTANCIA-CORRECTA (ancho fisico = cuerda 2*dmin*sin(dw/2)).
# v11 = SAFETY 0.05 -> 0.03. v10 = ley de velocidad (rapido recto, lento al girar).
# v9 = v4 con EVADE solo por golpe real. v4 = DOS MODOS (DRIVE/ROTATE) con histeresis.
#
# IDEA base (v4): DRIVE (frente despejado, sigue el hueco con Kalman 1D + PD) y ROTATE
#   (frente tapado). En v16 ROTATE ya no pivota: EXPLORA moviendose.
#   TURN = giro de 90 EXACTOS por /odom, disparado por comando UDP (YOLO/manual).
#
# CALIBRA FRONT_DEG ANTES DE CONFIAR EN ESTO (usa diag_lidar.py o el log [IDLE]).
# Signos (ROS REP-103): angular.z > 0 = IZQUIERDA (CCW); < 0 = DERECHA (CW).
# Correr:  export ROS_DOMAIN_ID=67 && python3 ~/autonomia_v17.py
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
USE_STAMPED = False     # Publica Twist en /cmd_vel_unstamped -> lo escucha 'create3_repub' (puente
                        #   directo a la base). Es la ruta MAS directa/robusta y la que definimos.
                        # OJO: en el ultimo diagnostico AMBOS (/cmd_vel y /cmd_vel_unstamped) tenian 1
                        #   suscriptor, asi que NO esta 100% confirmado. VERIFICAR con la prueba de
                        #   movimiento (robot en el PISO, fuera del dock):
                        #     ros2 topic pub --rate 10 --qos-reliability best_effort /cmd_vel_unstamped \
                        #          geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
                        #   Si NO mueve, prueba /cmd_vel (TwistStamped) y pon USE_STAMPED = True.
START_ARMED = False
STAGE       = 1
QR_LOG_FILE = "/home/ubuntu/checkpoints_log.txt"

# --- Velocidades ---
LIN    = 0.15            # (referencia v4)
LIN_MAX = 0.30           # finetune recta (antes 0.26). SOLO afecta DRIVE. Techo de velocidad; frenado por proximidad/giro sigue intacto.
W_ALIGN = 1.2            # finetune recta (antes 0.9). Mas tolerante al giro de centrado -> no mata la velocidad en recta angosta.
FLOOR_ALIGN = 0.25       # v10: fraccion minima de velocidad al girar fuerte
W_MAX  = 1.2            # tope de giro en DRIVE.
W_ROT  = 0.7            # giro de escape/exploracion.
W_EXPLORE_MIN = 0.15    # v16fix: giro minimo en EXPLORE (la repulsion ensancha pero no invierte).
SIGN_W = 3.0           # (NO USADO / no es un dial): en v16/v17 la senal en bifurcacion es prioridad DURA
                       #   (elige el hueco del LADO de la senal entre los que caben), no un peso. Se deja por referencia.
EXPLORE_V = 0.08         # v16: velocidad LENTA de exploracion (wall-follow) al rodear una esquina.
LEFT_BIAS = 0.0

# --- Umbrales de modo (histeresis DRIVE <-> ROTATE/EXPLORE) ---
D_BLOCK = 0.45          # frente < esto -> EXPLORE (rodea moviendose). Antes de llegar a la pared.
D_FREE  = 0.65          # frente > esto -> vuelve a DRIVE.
D_SLOW  = 0.70          # arriba de esto, velocidad plena.
D_EMERG = 0.20          # frente < esto (o ciego) = peligro -> NO avanza en EXPLORE (solo gira).
REAR_CLEAR = 0.35       # v17.13: margen ATRAS necesario para permitir retroceso en curva (encajonado).

# --- Hueco transitable ---
D_CLEAR    = 0.55       # un rayo es "libre" si supera esto.
ROBOT_PASS = 0.41       # v17.7: bajado de 0.43 (rechazaba huecos validos). ~3.2cm/lado. Subir si roza; bajar si no entra.
ROBOT_PASS_MIN = 0.376  # v16: umbral RELAJADO en PROBE = ~1.5 cm/lado. Solo si esta atascado.
NARROW_ARC_DEG = 50     # v18: la 'puerta real' SOLO se exige en huecos angulares < esto (mini/punta). Anchos (esquina normal) pasan con la cuerda.
PROBE_AFTER = 1.0       # v17.7: entra al modo DELICADO (PROBE) mas rapido (antes 1.5) para colarse en vez de chocar.
PROBE_V    = 0.06       # velocidad LENTA al colarse por el hueco justo.

# --- CONCIENCIA DEL CUERPO (que no roce con los flancos) ---
ROBOT_RADIUS = 0.173    # v12: radio real (diametro medido 34.6 cm).
SAFETY       = 0.03     # 3cm de inflacion.
R_INFL       = ROBOT_RADIUS + SAFETY
SIDE_CENTER_DEG = 90.0  # sector lateral centrado en +-90.
SIDE_HALF_DEG   = 55.0  # ancho del sector lateral.
SIDE_AVOID      = 0.36  # v17.13: (antes 0.34). Ajuste fino anti-roce en angostos.
K_SIDE          = 1.6   # v17.12: repulsion mas suave (antes 2.0) -> menos desvio/zigzag en recta angosta (control fino con PD).
FRONT_Q_DEG   = 40      # v19: centro del sector CUARTO-FRONTAL (deg) para anti-roce de esquina al girar.
FRONT_Q_HALF  = 25      # v19: medio ancho del cuarto-frontal (cubre ~15-65 deg a cada lado; zona que el sector lateral no ve).
FRONT_AVOID   = 0.30    # v19: distancia a la que una esquina frontal-lateral empieza a ABRIR el giro.
K_FRONT       = 1.2     # v19: fuerza del anti-roce frontal.
BACKUP_T   = 0.6        # seg de retroceso tras un golpe.
BACKUP_V   = -0.10      # velocidad de retroceso (m/s).
EVADE_T    = 0.8        # v17.3 merge: recuperacion mas larga tras golpe/ATASCO (retrocede+gira a hueco NUEVO).
EVADE_COOLDOWN = 1.5    # seg sin re-disparar evasion.
EVADE_T_MAX  = 1.6      # v17.9: tope SUAVE del escape (~64 deg max). Antes 4.0 (~160) pisaba la senal YOLO. NO subir sin motivo.
EVADE_SAME_D = 0.5      # v17.8: nuevo escape a < esto (m) del anterior = MISMA trampa -> gira MAS.
EVADE_SAME_T = 4.0      # v17.8: ...y a < esto (s) del anterior.
# --- ATASCO por odometria (v17.3 merge): independiente del bumper (que no reporta) ---
STALL_T = 1.2           # v17.9: mas tolerante (antes 0.8) -> EVADE dispara MENOS en maniobras normales/señal.
STALL_D = 0.02          # m: traslacion minima para considerar "avanzo".
STALL_A = 0.10          # rad (~6 deg): rotacion minima para considerar "giro".
FRONT_DEG       = -90.0 # CALIBRADO: caja al frente leyo ~-90 deg.
FRONT_HALF_DEG  = 14.0  # medio cono frontal para decidir DRIVE/EXPLORE.
SEARCH_HALF_DEG = 150.0 # busca huecos en +-150 deg.
R_MIN, R_MAX    = 0.06, 12.0

# --- Control PD del rumbo en DRIVE (sin termino I) ---
KP_HEADING = 1.1
KD_HEADING = 0.55       # v17.3 merge: mas amortiguacion del giro (menos zigzag en recto).
# Kalman 1D [theta, thetadot]
KF_R = 0.0030
KF_Q_TH = 0.0010
KF_Q_THD = 0.0200
STICK_W = 0.8           # v17.3 merge: mas pegajoso al rumbo previo (menos zigzag).

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
MAX_DW = 0.50
DEAD_S = 0.7
CMD_LISTEN_PORT = 5008
QR_HZ = 5.0
SCAN_DEBUG = True

# --- Senales YOLO (buffer; se consultan al decidir girar) ---
SIGN_TTL          = 3.0    # v17.12: la senal se DESVANECE en 3 s (antes 10) -> no sesga una senal nueva en entorno cerrado.
SIGN_MAX_DIST     = 1.5    # v17.1: ...y por DISTANCIA (m): si avanzo mas de esto desde que la vi, se descarta (anti-fantasma / cubre el cooldown de 5s del YOLO).
SIGN_TRIGGER_DIST = 0.55   # solo para 'stop': frena cuando el frente < esto (m).
SIGN_INVERT = False    # v18: si el robot va al lado CONTRARIO de la senal (espejo LiDAR/convencion) -> ponlo True.
FWD_CONE_DEG = 100     # v18.3: al obedecer una senal, SOLO considera huecos con |centro| <= esto (hacia ADELANTE). Evita girar ~180 hacia atras.
SIGN_ALIGN_MARGIN = 45 # v19: un hueco 'cuenta' para la senal si NO es del lado CONTRARIO (|margen|). Si solo hay del contrario -> gira a buscar el lado de la senal.
STOP_HOLD_S       = 2.0    # 'stop': frena este tiempo y sigue solo.
STOP_COOLDOWN     = 5.0    # v17.7: tras un STOP, IGNORA nuevos SSTOP por este tiempo (no re-frena con el mismo stop a la vista).
META_HOLD_S       = 10.0   # META (linea de meta): parar este tiempo. PROVISIONAL -> CONFIRMAR con Rensso.

# --- MEMORIA de trayectoria (v17): rejilla de 'migas de pan' en marco /odom, anti-loop ---
USE_MEMORY    = False   # v18: APAGADA para probar (era True). Toggle LIMPIO: off=efecto cero, no rompe. Pon True para anti-loop.
MEM_CELL      = 0.35    # v17.7: celda mas grande (antes 0.25) = memoria mas robusta a la deriva del odom. Mas chico = mas fina pero sensible a la
                        #   deriva del odom. ~= radio del robot es un buen punto medio.
MEM_LOOKAHEAD = 0.60    # a que distancia adelante (m) miro la celda para juzgar un rumbo candidato.
W_VISIT       = 0.30    # v17.12: memoria mas SUAVE (antes 0.60) -> en bifurcaciones ya visitadas NO se traba; sigue eligiendo el mejor hueco.
MEM_MARK_HZ   = 4.0     # cada cuanto dejo una miga (Hz). No hace falta a 20 Hz.
MEM_CAP       = 8       # tope de conteo por celda (para que una celda muy pisada no domine todo).
# =====================================================================================


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class AutonomiaV19(Node):
    def __init__(self):
        super().__init__("autonomia_v19")

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
        self.px = 0.0
        self.py = 0.0
        self.have_odom = False
        self.visit = {}                  # v17: {(i,j): conteo} celdas visitadas (marco odom)
        self._last_mark = 0.0
        self.stall_px = 0.0; self.stall_py = 0.0; self.stall_yaw = 0.0   # v17.3: ref del detector de atasco
        self.stall_t0 = 0.0

        self.state = "DRIVE"             # DRIVE | ROTATE(=EXPLORE) | TURN | PROBE | EVADE | SIGNSTOP
        self.rot_dir = -1.0              # direccion COMPROMETIDA al rodear (-1 der, +1 izq)
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
        self.sign_dir = None          # BUFFER de senal de giro (+1 izq, -1 der); se consulta al girar
        self.sign_t = 0.0             # momento en que llego (expira por SIGN_TTL)
        self.sign_px = 0.0            # v17.1: posicion odom al recibir la senal (expira por SIGN_MAX_DIST)
        self.sign_py = 0.0
        self.pending_stop = False     # senal YOLO 'stop' pendiente
        self.signstop_end = 0.0       # fin del frenado por 'stop'
        self.pending_meta = False     # META detectada (comando UDP 'META')
        self.meta_stop_end = 0.0      # fin del paro de 10s en la meta
        self.meta_done = False        # ya se proceso la meta (no repetir)
        self.stop_cd_end = 0.0        # v17.7: cooldown tras un SSTOP
        self.t_armed = None           # v17.7: instante en que se armo (para medir tiempo a cada QR)
        self.cp_times = []            # v17.7: lineas de checkpoints con su tiempo (resumen en META)
        self.turn_mode = None
        self.target_yaw = 0.0
        self.turn_t_end = 0.0
        self.turn_dir = 0.0
        self.t_backup = 0.0
        self.t_evade = 0.0
        self.t_evade_end = -10.0
        self.evade_dir = -1.0
        self.evade_count = 0          # v17.8: nº de escapes seguidos en la MISMA trampa (para escalar el giro)
        self.evade_ref_x = 0.0
        self.evade_ref_y = 0.0
        self.evade_dur = EVADE_T      # duracion actual del escape (crece si reincide)
        self.bump_side = 0.0
        self.t_state = 0.0

        self.checkpoints = set()
        try:
            open(QR_LOG_FILE, "w").close()   # v17.8: log de checkpoints LIMPIO en cada arranque (inicia de cero)
        except Exception:
            pass
        self.create_timer(1.0 / CONTROL_HZ, self.control_step)

        self.cmd_count = 0            # DIAG: comandos UDP recibidos y parseados OK
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(("0.0.0.0", CMD_LISTEN_PORT))
        threading.Thread(target=self._cmd_listen, daemon=True).start()

        self.get_logger().info("=== autonomia_v19 (v18 + senal busca su lado aunque de la vuelta + anti-roce cuarto-frontal) ===")
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
        self.px = msg.pose.pose.position.x      # v17: posicion para las migas de pan
        self.py = msg.pose.pose.position.y
        self.have_odom = True

    def hazard_cb(self, msg):
        try:
            for d in msg.detections:
                if d.type == 1:
                    self.bump_flag = True
                    fid = (getattr(d.header, "frame_id", "") or "").lower()  # v17.1: lado REAL del golpe
                    if "left" in fid:
                        self.bump_side = 1.0     # golpe IZQ -> evade DER (evade_dir = -bump_side)
                    elif "right" in fid:
                        self.bump_side = -1.0    # golpe DER -> evade IZQ
                    else:
                        self.bump_side = 0.0     # centro/desconocido -> lo decide el LiDAR en decide()
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
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            up = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)  # v17.6: ampliar x3 (preview 250x250 NO decodifica crudo)
            data, _, _ = self.qr.detectAndDecode(up)
            if data and data not in self.checkpoints:
                self.checkpoints.add(data)
                ts = time.strftime("%Y-%m-%d %H:%M:%S")
                elapsed = (time.time() - self.t_armed) if self.t_armed else -1.0   # v17.7: tiempo desde ARM
                line = f"{ts} | STAGE {STAGE} | CHECKPOINT {len(self.checkpoints)}/3 | t=+{elapsed:5.1f}s desde ARM | QR='{data}'"
                self.cp_times.append(line)
                self.get_logger().info(f">>> {line} <<<")
                with open(QR_LOG_FILE, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
                self.visit.clear()    # v17.1: memoria fresca por tramo entre checkpoints (acota deriva del odom)
        except Exception as e:
            self.get_logger().error(f"img_cb: {e}")

    def btn_cb(self, msg):
        pressed = bool(msg.button_1.is_pressed)
        if pressed and not self._btn_prev:
            self.armed = not self.armed
            if self.armed: self.visit.clear(); self.t_armed = time.time()   # v17: memoria fresca + reloj del intento
            self.get_logger().info(f">>> BOTON 1: {'ARMADO' if self.armed else 'PAUSA'} <<<")
        self._btn_prev = pressed

    def _cmd_listen(self):
        # DIAG v17.2: protocolo opcional "CMD#seq". Si trae #seq, responde "ACK#seq" al emisor
        #   (la laptop mide RTT y perdida). "PING#seq" -> "PONG#seq" SIN efecto en la nav.
        #   cmd_count permite verificar parseo/coordinacion (cuantos comandos entraron de verdad).
        while True:
            try:
                data, addr = self.cmd_sock.recvfrom(256)
                raw = data.decode(errors="ignore").strip()
                base, _, seq = raw.partition("#")        # "LEFT#42" -> base="LEFT", seq="42"
                c = base.strip().upper()
                if c == "PING":                          # DIAG puro: latencia/perdida sin mover el robot
                    if seq:
                        self.cmd_sock.sendto(f"PONG#{seq}".encode(), addr)
                    continue
                if c in ("ARM", "GO"):
                    self.armed = True; self.visit.clear(); self.t_armed = time.time(); self._enter("DRIVE")
                elif c in ("PAUSE", "STOP", "IDLE"):
                    self.armed = False
                elif c == "TOGGLE":
                    self.armed = not self.armed
                elif c in ("LEFT", "L", "L90"):
                    self.sign_dir = 1.0; self.sign_t = time.time()      # al BUFFER (se usa al decidir girar)
                    self.sign_px = self.px; self.sign_py = self.py      # v17.1: ancla espacial
                elif c in ("RIGHT", "R", "R90"):
                    self.sign_dir = -1.0; self.sign_t = time.time()
                    self.sign_px = self.px; self.sign_py = self.py
                elif c in ("SSTOP", "SIGN_STOP"):
                    self.pending_stop = True                            # senal 'stop' (por cercania)
                elif c in ("META", "FINISH"):
                    self.pending_meta = True                            # llego a la meta (bandera a cuadros)
                else:
                    continue
                self.cmd_count += 1                       # DIAG: contador de comandos validos
                if seq:                                   # DIAG: confirma recepcion a la laptop (RTT/perdida)
                    self.cmd_sock.sendto(f"ACK#{seq}".encode(), addr)
                self.get_logger().info(f">>> CMD #{self.cmd_count} '{c}' seq={seq or '-'} de {addr[0]}")
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

    # ------------------------------- Memoria (v17) -------------------------------
    def _cell(self, x, y):
        # (x,y) del mundo -> indice de celda entero. floor para que las celdas sean estables.
        return (int(math.floor(x / MEM_CELL)), int(math.floor(y / MEM_CELL)))

    def _mark_visit(self):
        # deja una miga en la celda actual (throttle MEM_MARK_HZ). Solo con odom valido.
        if not (USE_MEMORY and self.have_odom):
            return
        now = time.time()
        if now - self._last_mark < 1.0 / MEM_MARK_HZ:
            return
        self._last_mark = now
        c = self._cell(self.px, self.py)
        self.visit[c] = min(self.visit.get(c, 0) + 1, MEM_CAP)

    def _visit_ahead(self, centro):
        # conteo de visitas de la celda ~MEM_LOOKAHEAD m adelante, en el rumbo relativo 'centro'.
        # Es el "ya estuve por alla?" que se suma al costo del hueco. 0 si no hay odom.
        if not (USE_MEMORY and self.have_odom):
            return 0.0
        wh = self.yaw + centro                       # rumbo en el mundo (odom)
        wx = self.px + MEM_LOOKAHEAD * math.cos(wh)
        wy = self.py + MEM_LOOKAHEAD * math.sin(wh)
        return float(self.visit.get(self._cell(wx, wy), 0))

    def _door_ok(self, pts, i, j, n, thr):
        # v18: PUERTA real = distancia (m) entre los DOS obstaculos que bordean el arco libre (indices i-1, j+1).
        #   door = || P_izq - P_der ||,  P = (d*cos a, d*sin a).  Si door < thr, el cuerpo NO cruza.
        # Rechaza esquinas EN PUNTA (paredes que convergen) y mini-huecos, aunque la cuerda diera >= ROBOT_PASS.
        # Si el arco toca el borde de busqueda (i<=0 o j+1>=n) -> ese lado es ABIERTO -> no limita -> True.
        if i <= 0 or j + 1 >= n:
            return True
        al, dl = pts[i - 1]
        ar, dr = pts[j + 1]
        dl = max(dl, R_MIN); dr = max(dr, R_MIN)   # ray ciego (0) = obstaculo pegadisimo
        xl, yl = dl * math.cos(al), dl * math.sin(al)
        xr, yr = dr * math.cos(ar), dr * math.sin(ar)
        return math.hypot(xl - xr, yl - yr) >= thr

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
                ancho = 2.0 * dmin * math.sin(max(dw, 0.0) / 2.0)
                door = (dw >= math.radians(NARROW_ARC_DEG)) or self._door_ok(pts, i, j, n, ROBOT_PASS_MIN)
                if ancho >= thr and door:   # v18: arco ANCHO -> cuerda basta; ANGOSTO -> exige puerta real
                    centro = 0.5 * (a_lo + a_hi)        # apunta al centro del hueco
                    costo = abs(centro) - (0.08 if centro > 0 else 0.0)
                    if use_stick:
                        costo += STICK_W * abs(wrap(centro - self.prev_heading))
                    costo += W_VISIT * self._visit_ahead(centro)   # v17: evita volver a lo ya recorrido
                    if mejor is None or costo < mejor[0]:
                        mejor = (costo, centro, dmin)
                i = j + 1
            else:
                i += 1
        return None if mejor is None else (mejor[1], mejor[2])

    def gaps_list(self, scan, pass_thr=None):
        # Lista de (centro, dmin) de TODOS los huecos que caben (para detectar bifurcaciones).
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
            return []
        pts.sort(key=lambda p: p[0])
        thr = ROBOT_PASS if pass_thr is None else pass_thr
        out = []
        n = len(pts); i = 0
        while i < n:
            if pts[i][1] > D_CLEAR:
                j = i; dmin = pts[i][1]
                while j + 1 < n and pts[j + 1][1] > D_CLEAR:
                    j += 1; dmin = min(dmin, pts[j][1])
                dw = pts[j][0] - pts[i][0]
                ancho = 2.0 * dmin * math.sin(max(dw, 0.0) / 2.0)
                if ancho >= thr:
                    out.append((0.5 * (pts[i][0] + pts[j][0]), dmin))
                i = j + 1
            else:
                i += 1
        return out

    def _enter(self, st):
        self.state = st
        self.t_state = time.time()
        if st != "DRIVE":
            self.kf_ready = False

    def _log(self, msg):
        now = time.time()
        if SCAN_DEBUG and (now - self._last_log > 0.5):
            self._last_log = now
            self.get_logger().info(msg)

    def _sign_fresh(self):
        # v17.1: la senal vale si no expiro por TIEMPO (SIGN_TTL) NI por DISTANCIA (SIGN_MAX_DIST).
        if self.sign_dir is None:
            return False
        if (time.time() - self.sign_t) > SIGN_TTL:
            return False
        if self.have_odom and math.hypot(self.px - self.sign_px, self.py - self.sign_py) > SIGN_MAX_DIST:
            return False
        return True

    def _consume_sign(self):
        # v18.2 PEEK (NO borra): devuelve la senal si es FRESCA (tiempo+distancia). Antes la borraba tras
        #   1 ciclo -> obedecia la flecha un instante y luego la GEOMETRIA lo jalaba al lado contrario.
        #   Ahora persiste y guia el giro VARIOS ciclos; se expira sola por SIGN_TTL / SIGN_MAX_DIST.
        return self.sign_dir if self._sign_fresh() else None

    def _enter_rotate(self, scan):
        # Al decidir girar (barrera detectada), CONSULTA el buffer del YOLO.
        #   - buffer con senal fresca -> gira hacia ELLA.
        #   - buffer vacio -> default: hacia el mejor hueco; si no hay, derecha.
        # Queda COMPROMETIDO con la direccion hasta despejar el frente.
        sign = self._consume_sign()
        if sign is not None:
            if SIGN_INVERT: sign = -sign                 # v18: corrige ESPEJO
            # v17.2 PROTECCION: obedece la senal SOLO si ese lado tiene un hueco transitable.
            # Si la senal apunta a una pared (falso positivo del YOLO), la ignora -> no hay forma
            # de chocar; cae al hueco seguro de abajo (re-analiza y toma otra salida).
            cand = [g for g in self.gaps_list(scan, ROBOT_PASS) if (g[0] >= 0) == (sign > 0)]
            if cand:
                self.rot_dir = sign
                self._enter("ROTATE")
                self.get_logger().info(f">>> EXPLORE por SENAL YOLO -> {'IZQ' if sign > 0 else 'DER'}")
                return
            self.get_logger().info(">>> SENAL hacia lado bloqueado -> la ignoro, tomo hueco seguro")
        h = self.mejor_hueco(scan, use_stick=False)
        if h is not None:
            self.rot_dir = 1.0 if h[0] >= 0 else -1.0
        else:
            # v17: sin hueco -> rodea hacia el lado MENOS pisado (izq +60 vs der -60).
            vl = self._visit_ahead(math.radians(60.0))
            vr = self._visit_ahead(-math.radians(60.0))
            self.rot_dir = 1.0 if vl < vr else -1.0
        self._enter("ROTATE")
        self.get_logger().info(f">>> EXPLORE (default) a {'IZQ' if self.rot_dir > 0 else 'DER'}")

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
    def _check_stall(self):
        # ATASCO (v17.3 merge): comando movimiento pero NO me traslado NI roto por STALL_T seg.
        # Independiente del bumper (que en este robot no reporta). Dispara EVADE hacia un hueco NUEVO.
        if not self.have_odom:
            return False
        now = time.time()
        moved = (math.hypot(self.px - self.stall_px, self.py - self.stall_py) > STALL_D or
                 abs(wrap(self.yaw - self.stall_yaw)) > STALL_A)
        cmd_move = (abs(self.v_cur) > 0.02 or abs(self.w_cur) > 0.15)   # estamos PIDIENDO movernos
        if moved or not cmd_move:
            self.stall_px, self.stall_py, self.stall_yaw = self.px, self.py, self.yaw
            self.stall_t0 = now
            return False
        return (now - self.stall_t0) > STALL_T

    def decide(self, scan, dt):
        self._mark_visit()   # v17: deja miga de la celda actual (throttled)
        # SIGNSTOP: 'stop' de senal -> frena STOP_HOLD_S y sigue solo.
        now0 = time.time()
        if self.state == "SIGNSTOP":
            if now0 < self.signstop_end:
                return 0.0, 0.0
            self.stop_cd_end = now0 + STOP_COOLDOWN   # v17.7: no re-frenar por SSTOP por 5s
            self._enter("DRIVE")

        # META (bandera a cuadros): PROVISIONAL. **CONFIRMAR CON RENSSO**: ¿el intento TERMINA en la meta
        # (detener y ya) o hay que seguir? Por ahora: PARAR META_HOLD_S seg y girar 180 para no quedar
        # mirando la meta y re-disparar. meta_done evita repetirlo. Si Rensso dice "terminar", en METASTOP
        # deja return 0,0 permanente (no dispares el giro).
        if self.state == "METASTOP":
            if now0 < self.meta_stop_end:
                return 0.0, 0.0
            self.turn_request = math.pi        # 180 grados por /odom (lo ejecuta el estado TURN)
            self.meta_done = True
            self._enter("DRIVE")
            return 0.0, 0.0
        if self.pending_meta and not self.meta_done and self.state not in ("TURN", "METASTOP", "SIGNSTOP"):
            self.pending_meta = False
            self.meta_stop_end = now0 + META_HOLD_S
            self._enter("METASTOP")
            total = (time.time() - self.t_armed) if self.t_armed else -1.0   # v17.7: resumen de checkpoints al llegar a META
            try:
                with open(QR_LOG_FILE, "a", encoding="utf-8") as f:
                    f.write(f"===== META (STAGE {STAGE}) | tiempo total +{total:.1f}s | {len(self.checkpoints)}/3 checkpoints =====\n")
                    for l in self.cp_times:
                        f.write("   " + l + "\n")
            except Exception:
                pass
            self.get_logger().info(f">>> META: {len(self.checkpoints)}/3 checkpoints en +{total:.1f}s. PARO 10s + giro 180 (CONFIRMAR con Rensso).")
            return 0.0, 0.0

        # v17.7: durante el cooldown, ignora cualquier SSTOP (evita re-frenar con el mismo stop a la vista).
        if now0 <= self.stop_cd_end:
            self.pending_stop = False
        # 'stop' de senal: al acercarse a la pared, frena STOP_HOLD_S y sigue solo.
        if self.state not in ("TURN", "SIGNSTOP") and self.pending_stop:
            fr_now = self.sector_min(scan, 0.0, math.radians(FRONT_HALF_DEG))
            if fr_now < SIGN_TRIGGER_DIST:
                self.pending_stop = False
                self.signstop_end = now0 + STOP_HOLD_S
                self._enter("SIGNSTOP")
                self.get_logger().info(">>> YOLO: STOP (frena 2s)")
                return 0.0, 0.0

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

        # EVADE FINO (revert v17.11): giro CORTO fijo. Ante GOLPE real, o ATASCO por odom SOLO en DRIVE
        # (NO interrumpe el enhebrado fino de PROBE/EXPLORE). Restaura el control delicado de v17.
        now = time.time()
        stalled = self._check_stall() and self.state == "DRIVE"   # el atasco solo cuenta en DRIVE
        if (self.bump_flag or stalled) and self.state != "EVADE" and (now - self.t_evade_end) > EVADE_COOLDOWN:
            self.bump_flag = False
            if self.bump_side != 0.0 and not stalled:
                self.evade_dir = -self.bump_side
            else:
                li = self.sector_min(scan,  math.radians(60.0), math.radians(40.0))
                ri = self.sector_min(scan, -math.radians(60.0), math.radians(40.0))
                self.evade_dir = 1.0 if li >= ri else -1.0
            self.stall_t0 = now
            self.t_evade = now
            self._enter("EVADE")
            self.get_logger().info(f"[EVADE] {'ATASCO' if stalled else 'golpe'} -> gira {'IZQ' if self.evade_dir>0 else 'DER'} (corto)")
        else:
            self.bump_flag = False
        if self.state == "EVADE":
            if now - self.t_evade < EVADE_T:            # giro CORTO fijo -> retrocede, gira poco y REINTENTA enhebrar
                rear = self.sector_min(scan, math.radians(180.0), math.radians(40.0))
                return (BACKUP_V if rear > 0.30 else 0.0), self.evade_dir * W_ROT
            self.t_evade_end = now
            self._enter("DRIVE")

        front = self.sector_min(scan, 0.0, math.radians(FRONT_HALF_DEG))
        blind = self.frente_ciego(scan)

        # 1) Transiciones DRIVE <-> EXPLORE con HISTERESIS.
        if self.state != "ROTATE" and (blind or front < D_BLOCK):
            self._enter_rotate(scan)
        elif self.state == "ROTATE" and (not blind) and front > D_FREE:
            self._enter("DRIVE")

        # 2) EXPLORE (v16): NO pivota en el sitio. Se TRASLADA lento rodeando la esquina
        #    (direccion comprometida + repulsion lateral anti-roce). El LiDAR ya ve 360;
        #    lo que revela huecos ocultos tras una esquina/tapa de caja es MOVERSE.
        if self.state == "ROTATE":
            relaxed = self.mejor_hueco(scan, use_stick=False, pass_thr=ROBOT_PASS_MIN)
            # v17.13: ENCAJONADO (frente pegadisimo, sin hueco para colarse) y CON margen ATRAS ->
            #   retrocede EN CURVA para ganar espacio y salir a otro lado (en vez de pivotar contra la caja).
            #   Al retroceder, 'front' crece y sale solo de este modo cuando ya tiene sitio.
            if front < D_EMERG and relaxed is None:
                rear = self.sector_min(scan, math.radians(180.0), math.radians(40.0))
                if rear > REAR_CLEAR:
                    self._log(f"[EXPLORE] ENCAJONADO front={front:.2f} rear={rear:.2f} -> retrocedo en curva ({'IZQ' if self.rot_dir>0 else 'DER'})")
                    return BACKUP_V, self.rot_dir * W_ROT
            if (time.time() - self.t_state) > PROBE_AFTER and relaxed is not None:
                self._enter("PROBE")        # sigue sin salida amplia -> intenta el hueco justo, lento
            else:
                v_ex = EXPLORE_V * clamp((front - D_EMERG) / (D_BLOCK - D_EMERG), 0.0, 1.0)  # decrece suave
                lft = self.sector_min(scan,  math.radians(SIDE_CENTER_DEG), math.radians(SIDE_HALF_DEG))
                rgt = self.sector_min(scan, -math.radians(SIDE_CENTER_DEG), math.radians(SIDE_HALF_DEG))
                rep = 0.0
                if lft < SIDE_AVOID: rep -= K_SIDE * (SIDE_AVOID - lft)
                if rgt < SIDE_AVOID: rep += K_SIDE * (SIDE_AVOID - rgt)
                w = clamp(self.rot_dir * W_ROT + rep, -W_MAX, W_MAX)
                if self.rot_dir > 0:   w = max(w,  W_EXPLORE_MIN)   # ensancha pero no invierte
                elif self.rot_dir < 0: w = min(w, -W_EXPLORE_MIN)
                self._log(f"[EXPLORE] front={front:.2f} dir={'IZQ' if self.rot_dir>0 else 'DER'} "
                          f"v={v_ex:.2f} w={w:.2f}")
                return v_ex, w

        if self.state == "PROBE":
            hp = self.mejor_hueco(scan, use_stick=True, pass_thr=ROBOT_PASS_MIN)
            if hp is None:
                self._enter_rotate(scan)
                return EXPLORE_V, self.rot_dir * W_ROT
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
                self._enter("DRIVE")
            self._log(f"[PROBE lento] rumbo={math.degrees(theta_f):.0f} clear={clear_dir:.2f} w={w:.2f}")
            return PROBE_V, w

        # 3) DRIVE: frente despejado -> sigue el hueco. En BIFURCACION (>=2 huecos) y con SENAL
        #    fresca, elige el hueco del LADO de la senal (no el mas cercano). Cubre las Y.
        gaps = self.gaps_list(scan, ROBOT_PASS)
        if not gaps:
            self._enter_rotate(scan)
            return EXPLORE_V, self.rot_dir * W_ROT
        h = None
        if self._sign_fresh():                            # obedece la senal
            lado = 1.0 if self.sign_dir > 0 else -1.0     # +1 izq (centro>0), -1 der (centro<0)
            if SIGN_INVERT: lado = -lado
            fwd = [g for g in gaps if abs(g[0]) <= math.radians(FWD_CONE_DEG)]   # solo hacia ADELANTE (no 180)
            m = math.radians(SIGN_ALIGN_MARGIN)
            if lado < 0:                                  # DERECHA: descarta lo claramente-izquierda (centro > +m)
                good = [g for g in fwd if g[0] <= m]
                h = min(good, key=lambda g: g[0]) if good else None    # el MAS a la derecha entre los de adelante
            else:                                         # IZQUIERDA: descarta lo claramente-derecha
                good = [g for g in fwd if g[0] >= -m]
                h = max(good, key=lambda g: g[0]) if good else None    # el MAS a la izquierda
            if h is not None:
                self.get_logger().info(f">>> SENAL {'IZQ' if lado>0 else 'DER'} -> hueco mas a ese lado (adelante), centro={math.degrees(h[0]):.0f} deg")
            else:
                # v19: NO hay via del lado de la senal hacia adelante (solo del lado contrario) -> NO ir al contrario;
                #   GIRA comprometido hacia el lado de la senal a BUSCAR via (puede dar la vuelta), como pediria el profe.
                self.rot_dir = -1.0 if lado < 0 else 1.0
                self._enter("ROTATE")
                self.get_logger().info(f">>> SENAL {'IZQ' if lado>0 else 'DER'} sin via adelante -> giro a buscar ese lado (puede dar la vuelta)")
                return EXPLORE_V, self.rot_dir * W_ROT
        if h is None:
            h = self.mejor_hueco(scan, use_stick=True)
            if h is None:
                self._enter_rotate(scan)
                return EXPLORE_V, self.rot_dir * W_ROT
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
        if left_min  < SIDE_AVOID: rep -= K_SIDE * (SIDE_AVOID - left_min)
        if right_min < SIDE_AVOID: rep += K_SIDE * (SIDE_AVOID - right_min)
        # v19: CUARTO-FRONTAL (15-65 deg a cada lado): la esquina interior del giro que el sector lateral no ve.
        #   Simetrico -> se cancela (centra); esquina a un solo lado -> ABRE el giro y no la roza.
        fl = self.sector_min(scan,  math.radians(FRONT_Q_DEG), math.radians(FRONT_Q_HALF))
        fr = self.sector_min(scan, -math.radians(FRONT_Q_DEG), math.radians(FRONT_Q_HALF))
        if fl < FRONT_AVOID: rep -= K_FRONT * (FRONT_AVOID - fl)
        if fr < FRONT_AVOID: rep += K_FRONT * (FRONT_AVOID - fr)
        w = clamp(KP_HEADING * theta_f + KD_HEADING * thetadot_f + rep + LEFT_BIAS, -W_MAX, W_MAX)
        clear_dir = self.sector_min(scan, theta_f, math.radians(FRONT_HALF_DEG))
        f_front = clamp((clear_dir - D_BLOCK) / (D_SLOW - D_BLOCK), 0.0, 1.0)
        f_align = clamp(1.0 - abs(w) / W_ALIGN, FLOOR_ALIGN, 1.0)
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
                if gang is not None:
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
    node = AutonomiaV19()
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
