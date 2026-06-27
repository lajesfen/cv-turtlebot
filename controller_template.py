import socket
import struct
import time
import msvcrt

ROBOT_IP = "192.168.0.101"
ROBOT_PORT = 5007

SEND_HZ = 60  # un poco más alto, igual tu receiver publica a 50Hz

# Velocidades (sube si quieres, pero con cuidado)
LIN = 0.80     # m/s  (rápido)
ANG = 3.00     # rad/s (rápido)

def pack_cmd(v, w):
    return struct.pack('ff', float(v), float(w))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    v = 0.0
    w = 0.0

    period = 1.0 / SEND_HZ
    last_send = 0.0

    print("=== UDP Teleop FAST (Windows) ===")
    print(f"Target: {ROBOT_IP}:{ROBOT_PORT} @ {SEND_HZ} Hz")
    print("W: forward | S: back | A: left | D: right | X: stop | Q: quit")
    print("---------------------------------")

    try:
        while True:
            # lee todas las teclas pendientes
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):
                    _ = msvcrt.getch()  # ignora flechas
                    continue

                key = ch.decode(errors="ignore").lower()

                if key == "w":
                    v, w = +LIN, 0.0
                elif key == "s":
                    v, w = -LIN, 0.0
                elif key == "a":
                    v, w = 0.0, +ANG
                elif key == "d":
                    v, w = 0.0, -ANG
                elif key == "x":
                    v, w = 0.0, 0.0
                elif key == "q":
                    sock.sendto(pack_cmd(0.0, 0.0), (ROBOT_IP, ROBOT_PORT))
                    print("\nSaliendo (STOP enviado).")
                    return

                print(f"\r v={v:+.2f} m/s | w={w:+.2f} rad/s     ", end="")

            now = time.time()
            if now - last_send >= period:
                sock.sendto(pack_cmd(v, w), (ROBOT_IP, ROBOT_PORT))
                last_send = now

            time.sleep(0.001)

    finally:
        sock.close()

if __name__ == "__main__":
    main()
