#!/usr/bin/env python3
# yolo_win.py -- CORRE EN WINDOWS (o cualquier PC), SIN ROS ni cv_bridge.
# Lee los frames del STREAM MJPEG que sirve el robot (ver_y_capturar.py en :8000), corre el YOLO
# (best.pt) y manda por UDP la senal a autonomia_v16 (LEFT / RIGHT / SSTOP). Mismo "puente" que
# yolo_to_udp.py pero la fuente de frames es el stream web, no ROS.
#
# REQUISITOS (una vez, en Windows):   pip install ultralytics opencv-python
# EN EL ROBOT (para que exista el stream):
#     ros2 service call /oakd/start_camera std_srvs/srv/Trigger
#     export ROS_DOMAIN_ID=67 && nohup python3 ~/ver_y_capturar.py stream > ~/stream.log 2>&1 &
# EN WINDOWS:
#     python yolo_win.py --stream http://192.168.0.104:8000/stream.mjpg --robot-ip 192.168.0.104 --model best.pt
import argparse, socket, time
import cv2
from ultralytics import YOLO

CLASS_TO_CMD = {"turn_left": "LEFT", "turn_right": "RIGHT", "stop": "SSTOP"}

def main():
    ap = argparse.ArgumentParser(description="YOLO (Windows, sin ROS) -> UDP a autonomia_v16")
    ap.add_argument("--stream", required=True, help="URL del MJPEG del robot, p.ej. http://IP:8000/stream.mjpg")
    ap.add_argument("--robot-ip", required=True, help="IP del robot (destino UDP)")
    ap.add_argument("--port", type=int, default=5008)
    ap.add_argument("--model", default="best.pt")
    ap.add_argument("--conf", type=float, default=0.60)
    ap.add_argument("--min-area", type=float, default=0.03, help="fraccion minima del frame (cercania)")
    ap.add_argument("--consec", type=int, default=4, help="frames seguidos con la misma clase")
    ap.add_argument("--cooldown", type=float, default=5.0)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--show", action="store_true", help="muestra la ventana con las detecciones")
    args = ap.parse_args()

    model = YOLO(args.model)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.robot_ip, args.port)
    cap = cv2.VideoCapture(args.stream)     # OpenCV lee MJPEG por HTTP
    if not cap.isOpened():
        print(f"NO pude abrir el stream: {args.stream}  (¿corre ver_y_capturar en el robot? ¿IP/puerto ok?)")
        return
    print(f"YOLO Win listo | stream={args.stream} | destino={dest} | conf>={args.conf}")

    last_class, streak, last_fire = None, 0, 0.0
    while True:
        ok, img = cap.read()
        if not ok:
            time.sleep(0.05); continue
        now = time.time()
        if now - last_fire < args.cooldown:      # enfriamiento tras actuar
            if args.show: cv2.imshow("yolo", img); cv2.waitKey(1)
            continue
        h, w = img.shape[:2]; area_img = float(h * w)
        res = model.predict(img, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        best = None
        for b in res.boxes:
            conf = float(b.conf)
            if conf < args.conf: continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            area_frac = ((x2 - x1) * (y2 - y1)) / area_img
            if area_frac < args.min_area: continue     # muy chica -> lejos/espuria
            name = model.names[int(b.cls)]
            if name not in CLASS_TO_CMD: continue
            if best is None or area_frac > best[0]:
                best = (area_frac, name, conf)
        if args.show:
            cv2.imshow("yolo", res.plot()); cv2.waitKey(1)
        if best is None:
            last_class, streak = None, 0
            continue
        area_frac, name, conf = best
        if name == last_class: streak += 1
        else: last_class, streak = name, 1
        print(f"[ve] {name} conf={conf:.2f} area={area_frac:.3f} streak={streak}")
        if streak >= args.consec:
            cmd = CLASS_TO_CMD[name]
            sock.sendto(cmd.encode(), dest)
            print(f">>> ENVIADO '{cmd}' a {dest[0]}:{dest[1]}")
            last_fire, streak, last_class = now, 0, None

if __name__ == "__main__":
    main()
