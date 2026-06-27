import socket
import base64
import struct
import threading
import time
from datetime import datetime
from ultralytics import YOLO
import numpy as np
import cv2

# ========= Configuración =========

ROBOT_IP   = "192.168.0.101"  # IP del TurtleBot4
ROBOT_PORT = 6000             # Debe coincidir con el nodo de telemetría
CONTROL_PORT = 5007           # Puerto para enviar comandos

DESIRED_DOMAIN_ID = 67        # Debe coincidir con ROS_DOMAIN_ID del robot
PAIRING_CODE      = "oscar"
EXPECTED_ROBOT_NAME = "turtlebotoscar"  # por seguridad extra

# Velocidades
LIN = 0.20     # m/s
ANG = 3.00     # rad/s

ROTATION_90_TIME = (np.pi / 2) / ANG * 2  # tiempo aproximado para girar 90 grados
CONTROL_HZ = 20.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

# Visión y navegación
DETECT_IMGSZ = 640
SIGN_CONF_THRESHOLD = 0.40       # mínimo conf del bbox para considerar señal
SIGN_ARM_FRAMES = 2              # frames consecutivos antes de armar acción
OBSTACLE_STOP_M = 0.10           # nunca avanzar si algo está a menos de esto
ACTION_DISTANCE_M = 0.10         # ejecutar giro/parada de señal a esta distancia (LiDAR)
LIDAR_FRONT_HALF_RAD = np.deg2rad(35)
LIDAR_MAX_VALID_M = 12.0

# Estados
STATE_STOPPED = "STOPPED"
STATE_FORWARD = "FORWARD"
STATE_ARMED = "ARMED"
STATE_TURN_LEFT = "TURN_LEFT"
STATE_TURN_RIGHT = "TURN_RIGHT"

# ========= Variables Globales =========

control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
stop_event = threading.Event()
detector = cv2.QRCodeDetector()
model = YOLO("best.pt")
CLASSIFY_MODE = model.task == "classify"

state_lock = threading.RLock()
state = STATE_STOPPED
is_running = False
turn_deadline = None
start_time = None
checkpoints = []

pending_sign = None          # turn_left | turn_right | stop
min_front_range_m = float("inf")

_sign_streak_label = None
_sign_streak_count = 0

# ========= Helper Functions =========

def send_packet(v: float, w: float):
    control_sock.sendto(
        struct.pack("ff", float(v), float(w)),
        (ROBOT_IP, CONTROL_PORT),
    )


def set_state(new_state: str):
    global state, turn_deadline, pending_sign, start_time, is_running

    with state_lock:
        if new_state == state:
            return

        print(f"[STATE] {state} → {new_state}")

        if new_state == STATE_FORWARD and not is_running:
            is_running = True
            if start_time is None:
                start_time = time.monotonic()
                print("[STATE] Tiempo de carrera iniciado.")

        state = new_state

        if new_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT):
            turn_deadline = time.monotonic() + ROTATION_90_TIME
        else:
            turn_deadline = None

        if new_state != STATE_ARMED:
            pending_sign = None


def arm_sign(sign: str):
    """Registra señal vista a distancia; la acción se ejecuta al llegar a 10 cm."""
    global state, pending_sign
    with state_lock:
        if state not in (STATE_FORWARD, STATE_ARMED):
            return
        pending_sign = sign
        if state != STATE_ARMED:
            state = STATE_ARMED
            print(f"[ARMED] Señal '{sign}' registrada — avanzando hasta ~{ACTION_DISTANCE_M*100:.0f} cm")


def execute_pending_sign():
    """Ejecuta la acción de la señal cuando el LiDAR indica proximidad."""
    with state_lock:
        sign = pending_sign
        if sign is None:
            return

    if sign == "turn_left":
        set_state(STATE_TURN_LEFT)
    elif sign == "turn_right":
        set_state(STATE_TURN_RIGHT)
    elif sign == "stop":
        set_state(STATE_STOPPED)
    print(f"[ACTION] Ejecutando señal '{sign}' a ~{ACTION_DISTANCE_M*100:.0f} cm")


def min_front_range(ranges, angle_min: float, angle_inc: float) -> float:
    """Distancia mínima válida en el arco frontal del LiDAR."""
    best = float("inf")
    for i, raw in enumerate(ranges):
        r = float(raw)
        if not np.isfinite(r) or r <= 0.0 or r > LIDAR_MAX_VALID_M:
            continue
        angle = angle_min + i * angle_inc
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        if abs(angle) <= LIDAR_FRONT_HALF_RAD:
            best = min(best, r)
    return best


def detect_sign(img) -> tuple[str | None, float]:
    """Detección YOLO: devuelve (sign, conf) o (None, 0) si no hay bbox confiable."""
    if CLASSIFY_MODE:
        result = model.predict(source=img, imgsz=224, verbose=False)[0]
        if result.probs is None:
            return None, 0.0
        conf = float(result.probs.top1conf)
        if conf < SIGN_CONF_THRESHOLD:
            return None, conf
        return model.names[int(result.probs.top1)], conf

    result = model.predict(
        source=img, imgsz=DETECT_IMGSZ, conf=SIGN_CONF_THRESHOLD, verbose=False
    )[0]
    if len(result.boxes) == 0:
        return None, 0.0
    idx = int(result.boxes.conf.argmax())
    conf = float(result.boxes.conf[idx])
    sign = model.names[int(result.boxes.cls[idx])]
    return sign, conf


def update_sign_streak(sign: str | None) -> str | None:
    """Requiere SIGN_ARM_FRAMES consecutivos con la misma señal para armar."""
    global _sign_streak_label, _sign_streak_count
    if sign is None:
        _sign_streak_label = None
        _sign_streak_count = 0
        return None
    if sign == _sign_streak_label:
        _sign_streak_count += 1
    else:
        _sign_streak_label = sign
        _sign_streak_count = 1
    if _sign_streak_count >= SIGN_ARM_FRAMES:
        return sign
    return None


def save_checkpoints():
    try:
        if len(checkpoints) > 0:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"checkpoints-{timestamp}.txt"

            with open(filename, "w", encoding="utf-8") as f:
                f.write("data\ttime\n")
                f.write("-" * 30 + "\n")
                for c in checkpoints:
                    f.write(f"{c['data']}\t{c['time']}\n")

            print(f"[SAVE] Checkpoints guardados en {filename}")
    except Exception as e:
        print(f"[SAVE] Error: {e}")


def control_loop():
    print("[CTRL] Hilo de control iniciado.")
    while not stop_event.is_set():
        turn_finished = False
        should_execute_sign = False

        with state_lock:
            current_state = state
            deadline = turn_deadline
            front_m = min_front_range_m
            armed_sign = pending_sign

            if current_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT) and deadline is not None:
                if time.monotonic() >= deadline:
                    current_state = STATE_FORWARD
                    turn_finished = True

            obstacle_close = front_m <= OBSTACLE_STOP_M

            if obstacle_close and current_state in (STATE_FORWARD, STATE_ARMED):
                if current_state == STATE_ARMED and armed_sign is not None:
                    should_execute_sign = True
                send_packet(0.0, 0.0)
            elif current_state == STATE_FORWARD:
                send_packet(+LIN, 0.0)
            elif current_state == STATE_ARMED:
                send_packet(+LIN, 0.0)
            elif current_state == STATE_TURN_LEFT:
                send_packet(0.0, +ANG)
            elif current_state == STATE_TURN_RIGHT:
                send_packet(0.0, -ANG)
            else:
                send_packet(0.0, 0.0)

        if turn_finished:
            set_state(STATE_FORWARD)
        elif should_execute_sign:
            execute_pending_sign()

        time.sleep(CONTROL_PERIOD)
    print("[CTRL] Hilo de control terminado.")


def do_handshake(sock: socket.socket, robot_addr):
    sock.settimeout(1.0)
    print(f"[HANDSHAKE] Iniciando con {robot_addr}...")
    while not stop_event.is_set():
        msg = f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode("utf-8")
        sock.sendto(msg, robot_addr)
        try:
            data, addr = sock.recvfrom(4096)
            text = data.decode("utf-8").strip()
            parts = text.split()

            if len(parts) >= 3 and parts[0] == "ACK":
                domain_str = parts[1]
                robot_name = " ".join(parts[2:])
                print(f"[HANDSHAKE] Recibido: '{text}' desde {addr}")

                try:
                    domain_id = int(domain_str)
                except ValueError:
                    print("[HANDSHAKE] domain_id inválido, reintentando...")
                    continue

                if domain_id != DESIRED_DOMAIN_ID:
                    print(f"[HANDSHAKE] ROS_DOMAIN_ID no coincide "
                          f"(esperado={DESIRED_DOMAIN_ID}, recibido={domain_id}). Reintentando...")
                    continue

                if robot_name != EXPECTED_ROBOT_NAME:
                    print(f"[HANDSHAKE] robot_name no coincide "
                          f"(esperado={EXPECTED_ROBOT_NAME}, recibido={robot_name}). Reintentando...")
                    continue

                print(f"[HANDSHAKE] Emparejado con '{robot_name}' (domain {domain_id}).")
                sock.settimeout(None)
                return
            else:
                print(f"[HANDSHAKE] Mensaje inesperado: '{text}', reintentando...")

        except socket.timeout:
            print("[HANDSHAKE] Timeout esperando ACK, reintentando...")


def handle_scan(parts):
    """
    SCAN <domain_id> <robot_name> <sec> <nsec> <angle_min> <angle_inc> <n> r1 ... rn
    """
    global min_front_range_m
    if len(parts) < 8:
        return

    try:
        angle_min = float(parts[5])
        angle_inc = float(parts[6])
        n = int(parts[7])
        ranges_str = parts[8:]
        n_effective = min(n, len(ranges_str))
        ranges = [float(r) for r in ranges_str[:n_effective]]

        front = min_front_range(ranges, angle_min, angle_inc)
        with state_lock:
            min_front_range_m = front

        if front <= OBSTACLE_STOP_M:
            print(f"[SCAN] Obstáculo frontal a {front:.2f} m — avance bloqueado")

    except ValueError as e:
        print(f"[SCAN] Error parseando mensaje: {e}")


def handle_img(parts):
    global _sign_streak_label, _sign_streak_count

    if len(parts) < 6:
        print("[IMG] Mensaje demasiado corto.")
        return

    try:
        b64_str = " ".join(parts[5:])
        jpeg_bytes = base64.b64decode(b64_str)
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if img is None:
            print("[IMG] Error al decodificar imagen.")
            return

        with state_lock:
            current_state = state
            currently_turning = current_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT)

        # === QR ===
        qr_data, _, _ = detector.detectAndDecode(img)
        if qr_data:
            print(f"[IMG] QR detectado: '{qr_data}'")

            if qr_data == "start":
                if not currently_turning:
                    _sign_streak_label = None
                    _sign_streak_count = 0
                    set_state(STATE_FORWARD)
                    print("[QR] Carrera iniciada — avance neutro")
            elif is_running and qr_data not in [cp["data"] for cp in checkpoints]:
                elapsed = time.monotonic() - start_time if start_time else 0
                hours = int(elapsed // 3600)
                minutes = int((elapsed % 3600) // 60)
                seconds = elapsed % 60
                timestamp = f"{hours:02}:{minutes:02}:{seconds:06.3f}"
                checkpoints.append({"data": qr_data, "time": timestamp})
                print(f"[CHECKPOINT] {qr_data} @ {timestamp}")

        # === YOLO ===
        sign, conf = detect_sign(img)
        if sign:
            print(f"[YOLO] Señal detectada: {sign} ({conf:.2f})")
        else:
            print("[YOLO] Sin señal en frame")

        if not is_running or currently_turning:
            return

        if current_state == STATE_STOPPED:
            return

        confirmed = update_sign_streak(sign)
        if confirmed and current_state in (STATE_FORWARD, STATE_ARMED):
            arm_sign(confirmed)

        cv2.waitKey(1)

    except Exception as e:
        print(f"[IMG] Error manejando imagen: {e}")


def receive_loop(sock: socket.socket):
    print("[RCV] Hilo de recepción iniciado.")
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(65535)
        except OSError:
            break

        text = data.decode("utf-8", errors="ignore")
        parts = text.split()
        if not parts:
            continue

        msg_type = parts[0]
        if msg_type == "SCAN":
            handle_scan(parts)
        elif msg_type == "IMG":
            handle_img(parts)
        else:
            print(f"[RCV] Mensaje desconocido desde {addr}: '{msg_type}'")

    print("[RCV] Hilo de recepción terminado.")


# ========= Main =========

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (ROBOT_IP, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    rcv_thread = threading.Thread(target=receive_loop, args=(sock,), daemon=True)
    rcv_thread.start()

    print("[MAIN] Recibiendo telemetría. Ctrl+C para salir.")
    mode = "classify" if CLASSIFY_MODE else "detect"
    print(f"[MAIN] Modelo: {mode} | conf≥{SIGN_CONF_THRESHOLD} | obstáculo: {OBSTACLE_STOP_M*100:.0f} cm")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[MAIN] Cerrando...")
    finally:
        save_checkpoints()
        stop_event.set()
        sock.close()
        control_sock.close()
        rcv_thread.join(timeout=2.0)
        ctrl_thread.join(timeout=2.0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
