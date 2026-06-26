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

ROBOT_IP   = "192.168.0.101"  # IP del TurtleBot4
ROBOT_PORT = 6000             # Debe coincidir con el nodo de telemetría
CONTROL_PORT = 5007           # Puerto para enviar comandos

DESIRED_DOMAIN_ID = 67        # Debe coincidir con ROS_DOMAIN_ID del robot
PAIRING_CODE      = "oscar"
EXPECTED_ROBOT_NAME = "turtlebotoscar"  # por seguridad extra

# Velocidades
LIN = 0.60     # m/s
ANG = 3.00     # rad/s

ROTATION_90_TIME = (np.pi / 2) / ANG * 2  # tiempo aproximado para girar 90 grados
CONTROL_HZ = 20.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

# ========= Variables Globales =========

control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
stop_event = threading.Event()
detector = cv2.QRCodeDetector()
model = YOLO("best.pt")

state_lock = threading.Lock()
state = "STOPPED"          # STOPPED, FORWARD, TURN_LEFT, TURN_RIGHT
turn_deadline = None
checkpoints = []
start_time = None

# ========= Helper Functions =========

def send_packet(v: float, w: float):
    control_sock.sendto(
        struct.pack("ff", float(v), float(w)),
        (ROBOT_IP, CONTROL_PORT),
    )

def set_state(new_state: str):
    global state, turn_deadline, start_time

    with state_lock:
        if new_state == state:
            return
    
        if start_time is None and new_state == "FORWARD":
            start_time = time.monotonic()
            print("Set start time")

        state = new_state

        if new_state in ("TURN_LEFT", "TURN_RIGHT"):
            turn_deadline = time.monotonic() + ROTATION_90_TIME
        else:
            turn_deadline = None

def control_loop():
    print("[CTRL] Hilo de control iniciado.")
    while not stop_event.is_set():
        with state_lock:
            current_state = state
            deadline = turn_deadline

        if current_state in ("TURN_LEFT", "TURN_RIGHT") and deadline is not None and time.monotonic() >= deadline:
            set_state("STOPPED")
            with state_lock:
                current_state = state

        if current_state == "FORWARD":
            send_packet(+LIN, 0.0)
        elif current_state == "TURN_LEFT":
            send_packet(0.0, +ANG)
        elif current_state == "TURN_RIGHT":
            send_packet(0.0, -ANG)
        else:
            send_packet(0.0, 0.0)

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

def handle_scan(parts):
    """
    parts: lista de strings del mensaje:
    SCAN <domain_id> <robot_name> <sec> <nsec> <angle_min> <angle_inc> <n> r1 ... rn
    """
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

        # Aquí puedes hacer lo que quieras con el LIDAR.
        # Demo: imprimir algunos valores cada vez.
        print(f"[SCAN] robot={robot_name} domain={domain_id} "
              f"t={sec}.{nsec:09d} n={n_effective} "
              f"ejemplo={ranges[:5]}")

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

        qr_data, vertices, _ = detector.detectAndDecode(img)
        if qr_data:
            print(f"[IMG] QR detectado: '{qr_data}'")

        sign = None
        results = model.predict(
            source=img,
            imgsz=640,
            conf=0.1, # <--- De momento, para ver que detecta. Luego, subirlo para evitar falsos positivos
            verbose=False
        )
        result = results[0]

        if len(result.boxes) > 0:
            confidences = result.boxes.conf.cpu().numpy()
            idx = np.argmax(confidences)

            cls_id = int(result.boxes.cls[idx])
            sign = model.names[cls_id]

            conf = confidences[idx]
            print(f"[YOLO] Detectado: {sign} ({conf:.2f})")

        with state_lock:
            currently_turning = state in ("TURN_LEFT", "TURN_RIGHT")

        if qr_data and not currently_turning:
            if qr_data == "start" and state == "STOPPED":
                set_state("FORWARD")
            else:
                if qr_data not in [checkpoint["data"] for checkpoint in checkpoints]:
                    if start_time is not None:
                        elapsed = time.monotonic() - start_time
                    else:
                        elapsed = 0.0
                    checkpoints.append({ "data": qr_data, "time": elapsed })

        if sign and not currently_turning:
            if sign == "turn_right":
                set_state("TURN_RIGHT")
            elif sign == "turn_left":
                set_state("TURN_LEFT")
            elif sign == "stop":
                set_state("STOPPED")
        # cv2.waitKey(1)

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
            # handle_scan(parts)
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
                    f.write(f"{c['data']}\t{c['time']:.3f}\n")

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
    do_handshake(sock, robot_addr)

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
        stop_event.set()
        sock.close()
        control_sock.close()
        rcv_thread.join(timeout=2.0)
        ctrl_thread.join(timeout=2.0)
        cv2.destroyAllWindows()

        save_checkpoints()

if __name__ == "__main__":
    main()