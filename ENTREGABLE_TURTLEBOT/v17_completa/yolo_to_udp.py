#!/usr/bin/env python3
# =====================================================================================
# yolo_to_udp.py  -- CORRE EN LA LAPTOP (no en la Pi: torch es pesado para la Raspberry).
#
# QUE HACE: mira la camara del robot por ROS, corre el YOLOv8 de senales (best.pt) y, cuando
# ve una senal REAL y confiable, manda un comando UDP a autonomia_v4 (puerto 5008):
#     turn_left  -> "LEFT"   (v4 hace giro 90 por odometria)
#     turn_right -> "RIGHT"
#     stop       -> "STOP" (pausa) ... y tras STOP_PAUSE seg manda "GO" (reanuda)
#
# ARQUITECTURA (subsuncion): el LiDAR/v4 evita obstaculos SIEMPRE; el YOLO solo AÑADE
# decisiones de giro al ver una senal. Un falso positivo del YOLO NO puede causar choque
# (la seguridad la decide el LiDAR), a lo sumo un giro equivocado. Por eso ademas filtramos
# fuerte: confianza alta + persistencia en varios frames + tamano minimo + enfriamiento.
#
# REQUISITOS (en la laptop, una vez):
#     pip install ultralytics
#   y tener ROS 2 (jazzy) con cv_bridge:  sudo apt install ros-jazzy-cv-bridge
#
# CORRER (en la laptop, misma red y dominio que el robot):
#     export ROS_DOMAIN_ID=67
#     python3 yolo_to_udp.py --robot-ip 192.168.0.104 --model best.pt
# =====================================================================================
import argparse
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

# ----- Mapa de clase del modelo -> comando que entiende autonomia_v4 -----
CLASS_TO_CMD = {
    "turn_left":  "LEFT",
    "turn_right": "RIGHT",
    "stop":       "SSTOP",   # v15: 'stop' de senal = SSTOP (el robot frena 2s solo). "STOP" queda para PAUSA manual.
}


class YoloToUdp(Node):
    def __init__(self, args):
        super().__init__("yolo_to_udp")
        self.args = args
        self.model = YOLO(args.model)            # carga el best.pt entrenado (deteccion)
        self.bridge = CvBridge()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (args.robot_ip, args.port)

        # estado del filtro temporal
        self.last_class = None
        self.streak = 0                          # frames seguidos viendo la MISMA clase
        self.last_fire = 0.0                     # cuando disparamos el ultimo comando
        self.last_infer = 0.0

        self.create_subscription(Image, args.topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"YOLO->UDP listo | modelo={args.model} | topic={args.topic} | "
            f"destino={args.robot_ip}:{args.port} | conf>={args.conf} | "
            f"persistencia={args.consec} frames | area_min={args.min_area} | cooldown={args.cooldown}s")

    def send(self, cmd):
        self.sock.sendto(cmd.encode(), self.dest)
        self.get_logger().info(f">>> ENVIADO '{cmd}' a {self.dest[0]}:{self.dest[1]}")

    def cb(self, msg: Image):
        now = time.time()
        # throttle de inferencia (cuida CPU; no necesitamos 30 Hz)
        if now - self.last_infer < 1.0 / self.args.hz:
            return
        self.last_infer = now

        # enfriamiento: tras disparar, ignora detecciones un rato (evita spam en la misma senal)
        if now - self.last_fire < self.args.cooldown:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = img.shape[:2]
        area_img = float(h * w)

        # inferencia YOLO (deteccion). imgsz 640 = como se entreno.
        res = self.model.predict(img, imgsz=self.args.imgsz, conf=self.args.conf, verbose=False)[0]

        # elegir la deteccion mas GRANDE que pase confianza y tamano minimo (= senal cercana)
        best = None  # (area_frac, clase, conf)
        for b in res.boxes:
            conf = float(b.conf)
            if conf < self.args.conf:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            area_frac = ((x2 - x1) * (y2 - y1)) / area_img
            if area_frac < self.args.min_area:        # muy chica -> lejos/espuria -> ignora
                continue
            name = self.model.names[int(b.cls)]
            if name not in CLASS_TO_CMD:
                continue
            if best is None or area_frac > best[0]:
                best = (area_frac, name, conf)

        # filtro de PERSISTENCIA: la misma clase debe verse N frames seguidos para actuar
        if best is None:
            self.last_class = None
            self.streak = 0
            return

        area_frac, name, conf = best
        if name == self.last_class:
            self.streak += 1
        else:
            self.last_class = name
            self.streak = 1
        self.get_logger().info(f"[ve] {name} conf={conf:.2f} area={area_frac:.3f} streak={self.streak}")

        if self.streak >= self.args.consec:
            cmd = CLASS_TO_CMD[name]
            self.send(cmd)
            self.last_fire = now
            self.streak = 0
            self.last_class = None
            # 'stop' -> SSTOP: el nodo autonomia_v16 hace la pausa de 2s y reanuda solo (no mandamos GO).

    def _resume_once(self):
        # timer de un solo uso: reanuda tras un STOP y se autodestruye
        self.send("GO")
        for t in list(self.timers):
            self.destroy_timer(t)


def main():
    ap = argparse.ArgumentParser(description="Puente YOLO senales -> comandos UDP a autonomia_v4")
    ap.add_argument("--robot-ip", required=True, help="IP del robot (mismo que usas para ssh)")
    ap.add_argument("--port", type=int, default=5008)
    ap.add_argument("--model", default="best.pt")
    ap.add_argument("--topic", default="/oakd/rgb/preview/image_raw")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="tamano de inferencia. EN LA PI usa 256-320 (mas rapido).")
    ap.add_argument("--conf", type=float, default=0.60, help="confianza minima")
    ap.add_argument("--min-area", type=float, default=0.03,
                    help="fraccion minima del frame que debe ocupar la senal (cercania)")
    ap.add_argument("--consec", type=int, default=4, help="frames seguidos con la misma clase")
    ap.add_argument("--cooldown", type=float, default=5.0, help="seg de enfriamiento tras actuar")
    ap.add_argument("--hz", type=float, default=5.0, help="frecuencia de inferencia")
    ap.add_argument("--stop-pause", type=float, default=2.0,
                    help="seg en pausa ante un 'stop' antes de reanudar (0 = no reanudar)")
    args = ap.parse_args()

    rclpy.init()
    node = YoloToUdp(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
