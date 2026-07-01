import socket
import base64
import struct
import threading
import time
from ultralytics import YOLO
from datetime import datetime
import time
import numpy as np
import cv2

# ========= Configuración =========

ROBOT_IP   = "192.168.0.104"  # IP del TurtleBot4
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

STATE_STOPPED = "STOPPED"
STATE_FORWARD = "FORWARD"
STATE_TURN_LEFT = "TURN_LEFT"
STATE_TURN_RIGHT = "TURN_RIGHT"

CAMERA_HORIZONTAL_FOV = np.deg2rad(69.0)   # Raspberry Pi Camera v2 ≈ 62-69°
LIDAR_DISTANCE_WINDOW = np.deg2rad(5)      # Search ±5° around the detected sign

SIGN_TRIGGER_DISTANCE = 0.30                   # meters

# ========= Variables Globales =========

control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
stop_event = threading.Event()
detector = cv2.QRCodeDetector()

print("Loading YOLO...")
model = YOLO("best.pt")
print("YOLO loaded.")

state_lock = threading.Lock()
state = STATE_STOPPED
is_running = False
turn_deadline = None
checkpoints = []
start_time = None

latest_ranges = None
latest_angle_min = None
latest_angle_inc = None

# ========= Helper Functions =========

def send_packet(v: float, w: float):
    control_sock.sendto(
        struct.pack("ff", float(v), float(w)),
        (ROBOT_IP, CONTROL_PORT),
    )

def set_state(new_state: str):
    global state, turn_deadline, start_time, is_running
    
    with state_lock:
        if new_state == state:
            return
        
        print(f"[STATE] Transitioning from {state} -> {new_state}")
        
        if new_state == STATE_FORWARD and not is_running:
            is_running = True
            if start_time is None:
                start_time = time.monotonic()
                print("[STATE] Global start time initiated.")
        
        state = new_state
        
        if new_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT):
            turn_deadline = time.monotonic() + ROTATION_90_TIME
        else:
            turn_deadline = None

def control_loop():
    print("[CTRL] Hilo de control iniciado.")
    while not stop_event.is_set():
        turn_finished = False
        with state_lock:
            current_state = state
            deadline = turn_deadline
            
            if current_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT) and deadline is not None:
                if time.monotonic() >= deadline:
                    current_state = STATE_FORWARD
                    turn_finished = True
                else:
                    turn_finished = False
            else:
                turn_finished = False

            if current_state == STATE_FORWARD:
                send_packet(+LIN, 0.0)
            elif current_state == STATE_TURN_LEFT:
                send_packet(0.0, +ANG)
            elif current_state == STATE_TURN_RIGHT:
                send_packet(0.0, -ANG)
            else:
                send_packet(0.0, 0.0)
                
        if turn_finished:
            set_state(STATE_FORWARD)
            
        time.sleep(CONTROL_PERIOD)
    print("[CTRL] Hilo de control terminado.")

def do_handshake(sock: socket.socket, robot_addr):
    sock.settimeout(1.0)
    print(f"[HANDSHAKE] Iniciando con {robot_addr}...")
    while not stop_event.is_set():
        # Enviar HELLO <domain> <pairing_code>
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

def lidar_distance_at_angle(ranges, angle_min, angle_inc, target_angle, window=LIDAR_DISTANCE_WINDOW):
    best = float("inf")
    for i, raw in enumerate(ranges):
        r = float(raw)
        if not np.isfinite(r) or r <= 0.0:
            continue

        angle = angle_min + i * angle_inc
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi

        if abs(angle - target_angle) <= window:
            best = min(best, r)
    return best

def handle_scan(parts):
    """
    parts: lista de strings del mensaje:
    SCAN <domain_id> <robot_name> <sec> <nsec> <angle_min> <angle_inc> <n> r1 ... rn
    """
    global latest_ranges, latest_angle_min, latest_angle_inc
    if len(parts) < 8:
        print("[SCAN] Mensaje demasiado corto.")
        return

    try:
        domain_id = int(parts[1])
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])
        angle_min = float(parts[5])
        angle_inc = float(parts[6])
        n = int(parts[7])
        ranges_str = parts[8:]
        if len(ranges_str) != n:
            print(f"[SCAN] n={n} pero llegaron {len(ranges_str)} rangos. Usando min(len, n).")
        n_effective = min(n, len(ranges_str))
        ranges = [float(r) for r in ranges_str[:n_effective]]

        with state_lock:
            latest_ranges = ranges
            latest_angle_min = angle_min
            latest_angle_inc = angle_inc

    except ValueError as e:
        print(f"[SCAN] Error parseando mensaje: {e}")

def handle_img(parts):
    if len(parts) < 6:
        print("[IMG] Mensaje demasiado corto.")
        return

    try:
        domain_id = int(parts[1])
        robot_name = parts[2]
        sec = int(parts[3])
        nsec = int(parts[4])

        b64_str = " ".join(parts[5:])  # el resto del mensaje
        jpeg_bytes = base64.b64decode(b64_str)

        # Decodificar JPEG a imagen OpenCV
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if img is None:
            print("[IMG] Error al decodificar imagen.")
            return
        
        with state_lock:
            current_state = state
            currently_turning = current_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT)

        # === QR Detection ===

        qr_data, vertices, _ = detector.detectAndDecode(img)
        if qr_data:
            print(f"[IMG] QR detectado: '{qr_data}'")
            
            if qr_data == "start":
                if not currently_turning:
                    set_state(STATE_FORWARD)
            else:
                if is_running and qr_data not in [cp["data"] for cp in checkpoints]:
                    elapsed = time.monotonic() - start_time if start_time else 0
                    hours = int(elapsed // 3600)
                    minutes = int((elapsed % 3600) // 60)
                    seconds = elapsed % 60
                    timestamp = f"{hours:02}:{minutes:02}:{seconds:06.3f}"
                    
                    checkpoints.append({"data": qr_data, "time": timestamp})
                    print(f"[CHECKPOINT] Saved: {qr_data} at {timestamp}")

        # === YOLO Detection ===

        sign = None
        results = model.predict(
            source=img,
            imgsz=640,
            conf=0.8,
            verbose=False
        )
        result = results[0]

        if result.boxes is not None and len(result.boxes) > 0:
            confidences = result.boxes.conf.cpu().numpy()
            idx = np.argmax(confidences)

            box = result.boxes.xyxy[idx].cpu().numpy()
            x1, y1, x2, y2 = box
            image_width = img.shape[1]
            center_x = (x1 + x2) / 2
            normalized = (center_x - image_width/2) / (image_width/2)
            target_angle = normalized * (CAMERA_HORIZONTAL_FOV / 2)

            with state_lock:
                if latest_ranges is not None:
                    sign_distance = lidar_distance_at_angle(
                        latest_ranges,
                        latest_angle_min,
                        latest_angle_inc,
                        target_angle
                    )
                else:
                    sign_distance = float("inf")

            cls_id = int(result.boxes.cls[idx])
            sign = model.names[cls_id]

            conf = confidences[idx]
            print(
                f"[YOLO] {sign} "
                f"conf={conf:.2f} "
                f"angle={np.rad2deg(target_angle):.1f}° "
                f"distance={sign_distance:.2f} m"
            )

            if not sign or currently_turning or not is_running:
                return

            if not np.isfinite(sign_distance):
                print("[YOLO] No valid LiDAR reading for detected sign.")
                return

            if sign_distance > SIGN_TRIGGER_DISTANCE:
                print(f"[YOLO] {sign} detected ({sign_distance:.2f} m) - waiting...")
                return

            print(f"[YOLO] {sign} detected ({sign_distance:.2f} m) - EXECUTING")

            if sign == "turn_right":
                set_state(STATE_TURN_RIGHT)
            elif sign == "turn_left":
                set_state(STATE_TURN_LEFT)
            elif sign == "stop":
                set_state(STATE_STOPPED)

        # if sign and not currently_turning and is_running:
        #     if sign == "turn_right":
        #         set_state(STATE_TURN_RIGHT)
        #     elif sign == "turn_left":
        #         set_state(STATE_TURN_LEFT)
        #     elif sign == "stop":
        #         set_state(STATE_STOPPED)
        
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

        text  = data.decode("utf-8", errors="ignore")
        parts = text.split()
        if not parts:
            continue

        msg_type = parts[0]
        if msg_type == "SCAN":
            handle_scan(parts)
            pass
        elif msg_type == "IMG":
            handle_img(parts)
        else:
            print(f"[RCV] Mensaje desconocido desde {addr}: '{msg_type}'")

    print("[RCV] Hilo de recepción terminado.")

def save_checkpoints():
    try:
        if len(checkpoints) > 0:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"checkpoints-{timestamp}.txt"

            with open(filename, "w") as f:
                f.write("data\ttime\n")
                f.write("-" * 30 + "\n")

                for c in checkpoints:
                    f.write(f"{c['data']}\t{c['time']}\n")

            print(f"[SAVE] Checkpoints guardados en {filename}")

    except Exception as e:
        print(f"[SAVE] Error: {e}")
        
# ========= Main =========

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # IMPORTANTE: el cliente puede usar cualquier puerto local
    # Si quieres forzar uno: sock.bind(("0.0.0.0", 6001))

    robot_addr = (ROBOT_IP, ROBOT_PORT)

    # 1) Handshake
    print("Starting handshake...")
    do_handshake(sock, robot_addr)
    print("Handshake finished.")

    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    rcv_thread = threading.Thread(target=receive_loop, args=(sock,), daemon=True)
    rcv_thread.start()

    print("[MAIN] Recibiendo telemetría. [CTRL+C] para salir.")
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