"""
turtlebot_autonomous.py
-----------------------
Self-driving TurtleBot4 that:
  - Uses YOLOv8-nano (fine-tuned) to detect turn signs (turn_left / turn_right / stop)
  - Uses LIDAR to stay centered between walls while moving forward
  - Handles turns triggered by sign detection

Requirements:
    pip install ultralytics opencv-python numpy

To TRAIN your own model on your sign images, see train_signs.py (separate file).
If you already have a trained model, point YOLO_MODEL_PATH to your best.pt.
"""

import socket
import base64
import struct
import threading
import time

import numpy as np
import cv2
from ultralytics import YOLO

# ─────────────────────────── Configuration ───────────────────────────

ROBOT_IP     = "192.168.0.101"
ROBOT_PORT   = 6000
CONTROL_PORT = 5007

DESIRED_DOMAIN_ID   = 67
PAIRING_CODE        = "oscar"
EXPECTED_ROBOT_NAME = "turtlebotoscar"

# Speeds
LIN = 0.25          # m/s  — keep it slow for reliability
ANG = 1.20          # rad/s

ROTATION_90_TIME = (np.pi / 2) / ANG   # seconds to spin ~90°

# YOLO
YOLO_MODEL_PATH = "signs_model/weights/best.pt"   # swap with your trained model
YOLO_CONF_THRESHOLD = 0.70   # only act on high-confidence detections

# LIDAR wall-centering
# Indices into the ranges array for left / right side beams.
# For a standard 360-ray scan: index 90 ≈ left, index 270 ≈ right.
# Adjust these to match your robot's LIDAR orientation.
LIDAR_LEFT_IDX  = 90
LIDAR_RIGHT_IDX = 270
LIDAR_MAX_RANGE = 3.0     # metres — clamp inf/NaN to this
CENTER_GAIN     = 0.4     # how aggressively to correct lateral drift (P-controller)

# ─────────────────────────── Shared State ────────────────────────────

control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_lock = threading.Lock()

stop_event   = threading.Event()   # set this to shut everything down

# Latest LIDAR snapshot — written by receive_loop, read by drive_loop
latest_scan      = None
latest_scan_lock = threading.Lock()

# Sign command queue — written by vision, consumed by drive_loop
pending_turn      = None    # "LEFT" | "RIGHT" | "STOP" | None
pending_turn_lock = threading.Lock()

# ─────────────────────────── Motion ──────────────────────────────────

def send_packet(v: float, w: float):
    with control_lock:
        control_sock.sendto(
            struct.pack("ff", float(v), float(w)),
            (ROBOT_IP, CONTROL_PORT),
        )

def move(v: float, w: float): send_packet(v, w)
def stop_robot():              send_packet(0.0, 0.0)

# ─────────────────────────── Handshake ───────────────────────────────

def do_handshake(sock: socket.socket, robot_addr):
    sock.settimeout(1.0)
    print(f"[HANDSHAKE] Connecting to {robot_addr}...")
    while not stop_event.is_set():
        sock.sendto(f"HELLO {DESIRED_DOMAIN_ID} {PAIRING_CODE}".encode(), robot_addr)
        try:
            data, addr = sock.recvfrom(4096)
            parts = data.decode().strip().split()
            if len(parts) >= 3 and parts[0] == "ACK":
                try:
                    domain_id = int(parts[1])
                except ValueError:
                    continue
                robot_name = " ".join(parts[2:])
                if domain_id != DESIRED_DOMAIN_ID or robot_name != EXPECTED_ROBOT_NAME:
                    continue
                print(f"[HANDSHAKE] Paired with '{robot_name}' (domain {domain_id}).")
                sock.settimeout(None)
                return
        except socket.timeout:
            print("[HANDSHAKE] Timeout, retrying...")

# ─────────────────────────── LIDAR Handler ───────────────────────────

def handle_scan(parts):
    """Parse SCAN message and store the range array for drive_loop."""
    if len(parts) < 8:
        return
    try:
        n = int(parts[7])
        ranges_raw = parts[8 : 8 + n]
        ranges = []
        for r in ranges_raw:
            v = float(r)
            # Replace inf / NaN with a safe max distance
            ranges.append(v if np.isfinite(v) and v > 0 else LIDAR_MAX_RANGE)

        global latest_scan
        with latest_scan_lock:
            latest_scan = ranges
    except ValueError:
        pass

# ─────────────────────────── Vision Handler ──────────────────────────

model = YOLO(YOLO_MODEL_PATH)

def handle_img(parts):
    """Decode image, run YOLO, write detected command to pending_turn."""
    if len(parts) < 6:
        return
    try:
        jpeg_bytes = base64.b64decode(" ".join(parts[5:]))
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        results = model(img, conf=YOLO_CONF_THRESHOLD, verbose=False)[0]

        best_label = None
        best_conf  = 0.0

        for box in results.boxes:
            conf  = float(box.conf[0])
            label = model.names[int(box.cls[0])]   # e.g. "turn_left", "turn_right", "stop"
            if conf > best_conf:
                best_conf  = conf
                best_label = label

        global pending_turn
        if best_label:
            print(f"[VISION] Detected: {best_label} ({best_conf:.2f})")
            with pending_turn_lock:
                # Map your model's class names to internal commands
                mapping = {
                    "turn_left":  "LEFT",
                    "turn_right": "RIGHT",
                    "stop":       "STOP",
                }
                pending_turn = mapping.get(best_label)

        # Draw boxes and show window
        annotated = results.plot()
        cv2.imshow("TurtleBot Camera", annotated)
        cv2.waitKey(1)

    except Exception as e:
        print(f"[VISION] Error: {e}")

# ─────────────────────────── Receive Thread ──────────────────────────

def receive_loop(sock: socket.socket):
    print("[RX] Receive thread started.")
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(65535)
        except OSError:
            break
        parts = data.decode("utf-8", errors="ignore").split()
        if not parts:
            continue
        if parts[0] == "SCAN":
            handle_scan(parts)
        elif parts[0] == "IMG":
            handle_img(parts)
    print("[RX] Receive thread exiting.")

# ─────────────────────────── Drive Loop ──────────────────────────────

def drive_loop():
    """
    Main autonomy loop — runs in its own thread.

    Strategy:
      1. If a turn command is pending, execute it (blocking within this thread).
      2. Otherwise, drive forward while using LIDAR to stay centered.
    """
    print("[DRIVE] Drive loop started.")

    def do_turn(direction: str):
        print(f"[DRIVE] Executing turn: {direction}")
        stop_robot()
        time.sleep(0.2)

        if direction == "LEFT":
            move(0.0, +ANG)
        elif direction == "RIGHT":
            move(0.0, -ANG)
        elif direction == "STOP":
            stop_robot()
            stop_event.set()   # end the run
            return

        time.sleep(ROTATION_90_TIME)
        stop_robot()
        time.sleep(0.2)

    while not stop_event.is_set():

        # ── Check for a pending sign command ──
        global pending_turn
        with pending_turn_lock:
            command      = pending_turn
            pending_turn = None   # consume it

        if command:
            do_turn(command)
            continue   # re-check immediately after turn

        # ── LIDAR wall-centering ──
        with latest_scan_lock:
            scan = latest_scan

        correction = 0.0
        if scan and len(scan) > max(LIDAR_LEFT_IDX, LIDAR_RIGHT_IDX):
            d_left  = min(scan[LIDAR_LEFT_IDX],  LIDAR_MAX_RANGE)
            d_right = min(scan[LIDAR_RIGHT_IDX], LIDAR_MAX_RANGE)

            # Positive error → too close to right wall → steer left (positive w)
            error      = d_left - d_right
            correction = CENTER_GAIN * error

        move(LIN, correction)
        time.sleep(0.05)   # 20 Hz control loop

    stop_robot()
    print("[DRIVE] Drive loop exiting.")

# ─────────────────────────── Main ────────────────────────────────────

def main():
    sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    robot_addr = (ROBOT_IP, ROBOT_PORT)

    do_handshake(sock, robot_addr)

    rx_thread    = threading.Thread(target=receive_loop, args=(sock,),  daemon=True, name="RX")
    drive_thread = threading.Thread(target=drive_loop,                  daemon=True, name="DRIVE")

    rx_thread.start()
    drive_thread.start()

    print("[MAIN] Running. Ctrl+C to stop.")
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
    finally:
        stop_event.set()
        sock.close()
        control_sock.close()
        rx_thread.join(timeout=2.0)
        drive_thread.join(timeout=2.0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()