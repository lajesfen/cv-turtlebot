#!/usr/bin/env python3
# ver_y_capturar.py -- CORRE EN EL ROBOT (es Ubuntu, esta bien). Ves la camara en el NAVEGADOR de tu
# laptop Windows (sin ROS) y GUARDAS con un boton en la misma pagina. Sobrevive caidas de SSH (nohup).
#
# En el robot:
#   ros2 service call /oakd/start_camera std_srvs/srv/Trigger
#   export ROS_DOMAIN_ID=67
#   nohup python3 ~/ver_y_capturar.py turn_left > ~/ver.log 2>&1 &
# En la laptop (navegador):  http://<IP_DEL_ROBOT>:8000
#   -> ves en vivo; click "Guardar foto" para capturar. Se guardan en ~/sign_frames/turn_left/
# Para parar:  pkill -f ver_y_capturar.py
# Bajar fotos a la PC:  scp -r ubuntu@<IP>:~/sign_frames .\sign_frames
import os, sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CLASE = sys.argv[1] if len(sys.argv) > 1 else "misc"
OUT = os.path.expanduser(f"~/sign_frames/{CLASE}")
PORT = 8000
_lock = threading.Lock()
_jpeg = [None]      # ultimo frame jpeg (navegador)
_bgr  = [None]      # ultimo frame crudo (guardar)
_saved = [0]

PAGE = (b"<html><body style='margin:0;background:#111;text-align:center;color:#eee;font-family:sans-serif'>"
        b"<p>Clase: <b>" + CLASE.encode() + b"</b> &nbsp; guardadas: <span id='c'>0</span></p>"
        b"<button style='font-size:22px;padding:10px 24px' "
        b"onclick=\"fetch('/save').then(r=>r.text()).then(t=>document.getElementById('c').innerText=t)\">"
        b"Guardar foto</button><br><br>"
        b"<img src='/stream.mjpg' style='width:80vw;image-rendering:pixelated'></body></html>")

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path == "/":
            self.send_response(200); self.send_header("Content-Type","text/html"); self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/save":
            with _lock:
                img = None if _bgr[0] is None else _bgr[0].copy()
            if img is not None:
                fn = os.path.join(OUT, f"{CLASE}_{int(time.time()*1000)}.jpg")
                cv2.imwrite(fn, img); _saved[0] += 1
            self.send_response(200)
            self.send_header("Content-Type","text/plain")
            self.send_header("Cache-Control","no-store")
            self.end_headers()
            self.wfile.write(str(_saved[0]).encode())
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type","multipart/x-mixed-replace; boundary=frame"); self.end_headers()
            try:
                while True:
                    with _lock: jp = _jpeg[0]
                    if jp is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"+jp+b"\r\n")
                    time.sleep(0.05)
            except Exception:
                pass
        else:
            self.send_response(404); self.end_headers()

class Cam(Node):
    def __init__(self):
        super().__init__("ver_y_capturar")
        os.makedirs(OUT, exist_ok=True)
        self.bridge = CvBridge()
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.cb, qos_profile_sensor_data)
    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", img)
        if ok:
            with _lock:
                _jpeg[0] = buf.tobytes(); _bgr[0] = img

def main():
    rclpy.init(); node = Cam()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"Servidor en http://<IP>:{PORT}  | clase '{CLASE}' -> {OUT}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
