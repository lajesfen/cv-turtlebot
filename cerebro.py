#!/usr/bin/env python3
# CORRE EN LA LAPTOP (Windows). CAPA 1: avanzar SIEMPRE, esquivar sin pararse, registrar QR.
# SIN YOLO. Anticolision por LiDAR /scan (el robot NO publica /ir_intensity).
# Arranque seguro: inicia en IDLE; 'g'=arrancar, 'p'/espacio=pausa, 'q'=salir.
import socket
import base64
import struct
import threading
import time
import math

import numpy as np
import cv2

# ==================== CONFIG (EDITA ESTO) ====================
ROBOT_IP   = "192.168.0.101"
TELEM_PORT = 6000
CONTROL_PORT = 5007
DESIRED_DOMAIN_ID   = 67
PAIRING_CODE        = "oscar"
EXPECTED_ROBOT_NAME = "turtlebotoscar"

# --- Velocidades ---
LIN   = 0.15
CREEP = 0.10
ANG   = 0.8

# --- Anticolision por LiDAR (metros) ---
D_STOP  = 0.35   # si lo mas cercano al frente < esto -> choque inminente: gira
D_SLOW  = 0.80   # a partir de aqui empieza a frenar proporcional
D_CLEAR = 0.50   # histeresis: vuelve a crucero cuando el frente supera esto
K_STEER = 0.6    # ganancia de esquive (rad/s por metro de diferencia L-R)
VMIN_FRAC = 0.4

# --- Geometria del LiDAR (CALIBRAR FRONT_DEG con SCAN_DEBUG) ---
FRONT_DEG      = 0.0    # angulo (grados) que mira al FRENTE del robot. CALIBRAR.
FRONT_HALF_DEG = 25.0   # medio ancho del sector frontal
SIDE_DEG       = 50.0   # centro de los sectores izq/der respecto al frente
SIDE_HALF_DEG  = 25.0
R_MIN, R_MAX   = 0.06, 10.0   # rangos validos (m); fuera de esto = invalido/abierto
SCAN_DEBUG     = True   # imprime min global + sectores para calibrar

# --- Tiempos de seguridad de comunicacion ---
STALE_S = 0.7
DEAD_S  = 2.0

CONTROL_HZ = 15.0

# --- Suavizado de movimiento (evita arranques/frenos bruscos) ---
MAX_DV = 0.04    # cambio maximo de v por ciclo (m/s). Mas bajo = mas suave
MAX_DW = 0.20    # cambio maximo de w por ciclo (rad/s)

# --- Logging de checkpoints (requisito de validacion) ---
STAGE       = 1                    # cambialo segun el stage que estes corriendo
QR_LOG_FILE = "checkpoints_log.txt"
# ============================================================

ctrl_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
stop_event = threading.Event()
lock = threading.Lock()

latest_scan      = None    # (ranges, angle_min, angle_inc)
latest_scan_time = 0.0
latest_frame     = None
checkpoints      = set()
qr = cv2.QRCodeDetector()

_escape_dir  = 0
_last_log    = 0.0
_dead_logged = False
armed = False
_disp = {"front": 0.0, "L": 0.0, "R": 0.0}   # para la ventana


def send_cmd(v, w):
    ctrl_sock.sendto(struct.pack("ff", float(v), float(w)), (ROBOT_IP, CONTROL_PORT))


def do_handshake(sock):
    sock.settimeout(1.0)
    print(f"[HANDSHAKE] Enviando HELLO a {ROBOT_IP}:{TELEM_PORT} ...")
    while not stop_event.is_set():
        sock.sendto(f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode(), (ROBOT_IP, TELEM_PORT))
        try:
            data, addr = sock.recvfrom(4096)
            parts = data.decode().strip().split()
            if len(parts) >= 3 and parts[0] == "ACK":
                if int(parts[1]) == DESIRED_DOMAIN_ID and " ".join(parts[2:]) == EXPECTED_ROBOT_NAME:
                    print(f"[HANDSHAKE] *** EMPAREJADO con {parts[2]} (domain {parts[1]}) ***")
                    sock.settimeout(None)
                    return
                else:
                    print(f"[HANDSHAKE] ACK no coincide: {parts}")
        except socket.timeout:
            print("[HANDSHAKE] Timeout esperando ACK, reintentando... (revisa IP/wifi/robot)")


def handle_scan(parts):
    global latest_scan, latest_scan_time
    try:
        amin = float(parts[5]); ainc = float(parts[6]); n = int(parts[7])
        ranges = [float(x) for x in parts[8:8 + n]]
    except (ValueError, IndexError):
        return
    with lock:
        latest_scan = (ranges, amin, ainc)
        latest_scan_time = time.time()


def handle_img(parts):
    global latest_frame
    try:
        jpeg = base64.b64decode(" ".join(parts[5:]))
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        data, _, _ = qr.detectAndDecode(img)
        if data and data not in checkpoints:
            checkpoints.add(data)
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            line = f"{ts} | STAGE {STAGE} | CHECKPOINT {len(checkpoints)}/3 | QR='{data}'"
            print(f"[QR] >>> {line} <<<")
            try:
                with open(QR_LOG_FILE, "a", encoding="utf-8") as flog:
                    flog.write(line + "\n")
            except Exception as e:
                print(f"[QR] no se pudo escribir el log: {e}")
        with lock:
            latest_frame = img
    except Exception as e:
        print(f"[IMG] error: {e}")


def receive_loop(sock):
    print("[RX] Hilo de recepcion iniciado.")
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(65535)
        except OSError:
            break
        parts = data.decode("utf-8", errors="ignore").split()
        if not parts:
            continue
        t = parts[0]
        if t == "SCAN":
            handle_scan(parts)
        elif t == "IMG":
            handle_img(parts)
        # IR y ODOM se ignoran en Capa 1


def _sector_min(ranges, amin, ainc, center_deg, half_deg):
    """Distancia minima valida dentro de un sector angular (en grados, relativo al frame del laser)."""
    center = math.radians(center_deg)
    half = math.radians(half_deg)
    best = R_MAX
    best_ang = None
    for i, r in enumerate(ranges):
        if not (R_MIN < r < R_MAX):
            continue
        a = amin + i * ainc
        d = math.atan2(math.sin(a - center), math.cos(a - center))  # diferencia angular envuelta
        if abs(d) <= half and r < best:
            best = r
            best_ang = math.degrees(a)
    return best, best_ang


def decide(scan):
    """Reactivo con LiDAR: avanza y curva para esquivar; gira si el frente esta muy cerca."""
    global _last_log, _escape_dir
    ranges, amin, ainc = scan

    front, _ = _sector_min(ranges, amin, ainc, FRONT_DEG, FRONT_HALF_DEG)
    left,  _ = _sector_min(ranges, amin, ainc, FRONT_DEG + SIDE_DEG, SIDE_HALF_DEG)
    right, _ = _sector_min(ranges, amin, ainc, FRONT_DEG - SIDE_DEG, SIDE_HALF_DEG)

    with lock:
        _disp["front"], _disp["L"], _disp["R"] = front, left, right

    now = time.time()
    if now - _last_log > 0.5:
        _last_log = now
        modo = "ESCAPE" if (front < D_STOP or _escape_dir != 0) else "CRUCERO"
        extra = ""
        if SCAN_DEBUG:
            gmin, gang = _sector_min(ranges, amin, ainc, FRONT_DEG, 180.0)
            extra = f"  | min_global={gmin:.2f}m @ {gang}"
        print(f"[SCAN] front={front:.2f} L={left:.2f} R={right:.2f} -> {modo}{extra}")

    # Escape comprometido (sin oscilar)
    if front < D_STOP and _escape_dir == 0:
        _escape_dir = +1 if left > right else -1   # gira hacia el lado mas despejado
    if _escape_dir != 0:
        if front > D_CLEAR:
            _escape_dir = 0
        else:
            return 0.0, _escape_dir * ANG

    # Crucero: avanza (frena proporcional al acercarse) y curva alejandose del lado mas cerrado
    frac = (front - D_STOP) / max(0.01, (D_SLOW - D_STOP))
    v = LIN * max(VMIN_FRAC, min(1.0, frac))
    w = K_STEER * (left - right)
    w = max(-ANG, min(ANG, w))
    return v, w


def control_loop():
    print("[CTRL] Hilo de control iniciado.")
    global _dead_logged, _last_log
    period = 1.0 / CONTROL_HZ
    v_cur = 0.0
    w_cur = 0.0
    while not stop_event.is_set():
        with lock:
            scan = latest_scan
            age = time.time() - latest_scan_time

        if not armed:
            vt, wt = 0.0, 0.0
            # En IDLE seguimos mostrando el LiDAR para CALIBRAR FRONT_DEG sin movernos.
            if scan is not None and SCAN_DEBUG:
                ranges, amin, ainc = scan
                f, _ = _sector_min(ranges, amin, ainc, FRONT_DEG, FRONT_HALF_DEG)
                gmin, gang = _sector_min(ranges, amin, ainc, FRONT_DEG, 180.0)
                now = time.time()
                if now - _last_log > 0.5:
                    _last_log = now
                    print(f"[IDLE/SCAN] front={f:.2f}m  min_global={gmin:.2f}m @ {gang}  (pon objeto al frente y mira el angulo -> FRONT_DEG)")
        elif age > DEAD_S or scan is None:
            vt, wt = 0.0, 0.0
            if not _dead_logged:
                _dead_logged = True
                print("[CTRL] Sin /scan -> PARADO por seguridad (revisa enviador/wifi/LiDAR).")
        elif age > STALE_S:
            vt, wt = CREEP, 0.0
            _dead_logged = False
        else:
            vt, wt = decide(scan)
            _dead_logged = False

        # Suavizado (slew-rate): acerca v,w al objetivo sin saltos -> ni arranque ni freno brusco
        v_cur += max(-MAX_DV, min(MAX_DV, vt - v_cur))
        w_cur += max(-MAX_DW, min(MAX_DW, wt - w_cur))
        send_cmd(v_cur, w_cur)
        time.sleep(period)
    send_cmd(0.0, 0.0)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    do_handshake(sock)

    threading.Thread(target=receive_loop, args=(sock,), daemon=True).start()
    threading.Thread(target=control_loop, daemon=True).start()

    print("[MAIN] Ventana abierta. 'g'=ARRANCAR  'p'/espacio=PAUSA(idle)  'q'=salir")
    global armed
    try:
        while not stop_event.is_set():
            with lock:
                frame = latest_frame
                d = dict(_disp)
            canvas = frame.copy() if frame is not None else np.zeros((240, 320, 3), np.uint8)
            estado = "ARMADO (corriendo)" if armed else "IDLE  (pulsa 'g')"
            color = (0, 200, 0) if armed else (0, 0, 255)
            cv2.putText(canvas, estado, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(canvas, f"QR {len(checkpoints)}/3  front={d['front']:.2f} L={d['L']:.2f} R={d['R']:.2f}",
                        (10, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow("cerebro", canvas)
            k = cv2.waitKey(20) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("g"):
                armed = True
                print("[MAIN] >>> ARMADO: el robot puede moverse <<<")
            elif k == ord("p") or k == ord(" "):
                armed = False
                print("[MAIN] >>> PAUSA: idle por seguridad <<<")
    except KeyboardInterrupt:
        print("\n[MAIN] Cerrando...")
    finally:
        stop_event.set()
        time.sleep(0.2)
        send_cmd(0.0, 0.0)
        sock.close()
        ctrl_sock.close()
        cv2.destroyAllWindows()
        print(f"[MAIN] Checkpoints registrados: {checkpoints}")


if __name__ == "__main__":
    main()
