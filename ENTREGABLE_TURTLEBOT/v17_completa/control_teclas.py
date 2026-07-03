#!/usr/bin/env python3
# CORRE EN LA LAPTOP (Windows). Manda ARM/PAUSE/STOP al nodo a bordo por tecla.
# La autonomia sigue corriendo en el robot; esto es solo el "interruptor" remoto.
import socket
import msvcrt

ROBOT_IP = "192.168.0.104"   # <-- IP actual del robot
CMD_PORT = 5008              # debe = CMD_LISTEN_PORT de autonomia_robot.py

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send(cmd):
    sock.sendto(cmd.encode(), (ROBOT_IP, CMD_PORT))
    print(f"-> {cmd}")

print("=== Control por teclas (laptop -> robot) ===")
print(f"Destino: {ROBOT_IP}:{CMD_PORT}")
print("g = ARMAR | p o espacio = PAUSA | t = TOGGLE | q = STOP y salir")
print("(esta ventana debe estar enfocada para que registre las teclas)")

while True:
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):
        msvcrt.getch()
        continue
    k = ch.decode(errors="ignore").lower()
    if k == "g":
        send("ARM")
    elif k == "p" or k == " ":
        send("PAUSE")
    elif k == "t":
        send("TOGGLE")
    elif k == "q":
        send("STOP")
        print("Saliendo.")
        break

sock.close()
