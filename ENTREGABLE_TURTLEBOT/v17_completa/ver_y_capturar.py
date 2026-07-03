#!/usr/bin/env python3
# ver_y_capturar.py -- CORRE EN EL ROBOT (Ubuntu). Ves la camara en el NAVEGADOR de tu laptop (sin ROS)
# y guardas fotos para el dataset del YOLO. Sobrevive caidas de SSH (nohup). v17.2: capa de DIAGNOSTICO
# + captura optimizada.
#
# NUEVO v17.2:
#   - NITIDEZ (varianza del Laplaciano): mide si el frame esta borroso -> evita guardar fotos movidas
#     (el mayor veneno del dataset). Se muestra en vivo y va en el nombre del archivo.
#   - RAFAGA: guarda varias fotos seguidas de una clase (dataset mas rapido) sin bloquear el stream.
#   - STATS: FPS de la camara (ROS) y FPS servido al navegador, y EDAD del ultimo frame (latencia local).
#
# En el robot:
#   ros2 service call /oakd/start_camera std_srvs/srv/Trigger
#   export ROS_DOMAIN_ID=67
#   nohup python3 ~/ver_y_capturar.py turn_left > ~/ver.log 2>&1 &     # <-- clase asignada
#   (usa 'stream' como clase si solo quieres servir el video para el YOLO)
# En la laptop (navegador):  http://<IP_DEL_ROBOT>:8000   -> "Guardar foto" / "Rafaga"
#   Fotos en ~/sign_frames/<clase>/ .  Bajar:  scp -r ubuntu@<IP>:~/sign_frames .\sign_frames
# Parar:  pkill -f ver_y_capturar.py
import os, sys, time, json, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

CLASE = sys.argv[1] if len(sys.argv) > 1 else "misc"
OUT = os.path.expanduser(f"~/sign_frames/{CLASE}")
PORT = 8000
BLUR_WARN = 120.0        # varianza del Laplaciano por debajo de esto = probablemente borroso (referencia)

_lock = threading.Lock()
_jpeg = [None]           # ultimo frame jpeg (navegador)
_bgr = [None]            # ultimo frame crudo (guardar)
_sharp = [0.0]           # nitidez del ultimo frame
_saved = [0]
_cam_n = [0]; _cam_t = [0.0]     # contador y hora de frames de la camara (ROS)
_srv_n = [0]                     # contador de frames servidos al navegador
_prev = {"t": time.time(), "cam": 0, "srv": 0}   # para FPS por diferencia


def _save_one():
    with _lock:
        img = None if _bgr[0] is None else _bgr[0].copy()
        sh = _sharp[0]
    if img is None:
        return False
    fn = os.path.join(OUT, f"{CLASE}_{int(time.time()*1000)}_s{int(sh)}.jpg")
    cv2.imwrite(fn, img)
    with _lock:
        _saved[0] += 1
    return True


PAGE = ("""<html><head><meta charset='utf-8'></head>
<body style='margin:0;background:#111;text-align:center;color:#eee;font-family:sans-serif'>
<p>Clase: <b>%s</b> &nbsp; guardadas: <span id='c'>0</span></p>
<p>nitidez: <b id='sh'>-</b> &nbsp;|&nbsp; camFPS: <b id='cf'>-</b> &nbsp; srvFPS: <b id='sf'>-</b>
   &nbsp; edad: <b id='ag'>-</b> ms</p>
<button style='font-size:22px;padding:10px 24px' onclick="save()">Guardar foto</button>
<button style='font-size:22px;padding:10px 24px' onclick="burst()">Rafaga x8</button><br><br>
<img src='/stream.mjpg' style='width:80vw;image-rendering:pixelated'>
<script>
function save(){fetch('/save').then(r=>r.text()).then(t=>document.getElementById('c').innerText=t)}
function burst(){fetch('/burst?n=8').then(r=>r.text()).then(t=>document.getElementById('c').innerText=t)}
setInterval(()=>fetch('/stats').then(r=>r.json()).then(s=>{
  let sh=document.getElementById('sh'); sh.innerText=s.sharp.toFixed(0);
  sh.style.color = s.sharp<%d ? '#f66' : '#6f6';
  document.getElementById('cf').innerText=s.cam_fps.toFixed(1);
  document.getElementById('sf').innerText=s.srv_fps.toFixed(1);
  document.getElementById('ag').innerText=s.age_ms.toFixed(0);
  document.getElementById('c').innerText=s.saved;
}), 700);
</script></body></html>""" % (CLASE, BLUR_WARN)).encode()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _txt(self, body, ctype="text/plain"):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._txt(PAGE, "text/html")
        elif path == "/save":
            _save_one()
            with _lock: self._txt(str(_saved[0]).encode())
        elif path == "/burst":
            q = parse_qs(urlparse(self.path).query)
            n = int(q.get("n", ["8"])[0])
            threading.Thread(target=self._burst, args=(n,), daemon=True).start()
            with _lock: self._txt(str(_saved[0]).encode())
        elif path == "/stats":
            now = time.time()
            with _lock:
                cam, srv, sh, saved, camt = _cam_n[0], _srv_n[0], _sharp[0], _saved[0], _cam_t[0]
            dt = max(now - _prev["t"], 1e-3)
            cam_fps = (cam - _prev["cam"]) / dt
            srv_fps = (srv - _prev["srv"]) / dt
            _prev["t"], _prev["cam"], _prev["srv"] = now, cam, srv
            age = (now - camt) * 1000 if camt else -1
            self._txt(json.dumps({"sharp": sh, "cam_fps": cam_fps, "srv_fps": srv_fps,
                                  "age_ms": age, "saved": saved}).encode(), "application/json")
        elif path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
            try:
                while True:
                    with _lock: jp = _jpeg[0]
                    if jp is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jp + b"\r\n")
                        with _lock: _srv_n[0] += 1
                    time.sleep(0.05)
            except Exception:
                pass
        else:
            self.send_response(404); self.end_headers()

    def _burst(self, n):
        for _ in range(max(1, min(n, 40))):
            _save_one()
            time.sleep(0.12)     # deja que llegue un frame nuevo entre capturas


class Cam(Node):
    def __init__(self):
        super().__init__("ver_y_capturar")
        os.makedirs(OUT, exist_ok=True)
        self.bridge = CvBridge()
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())    # alto = nitido; bajo = borroso/movido
        ok, buf = cv2.imencode(".jpg", img)
        if ok:
            with _lock:
                _jpeg[0] = buf.tobytes(); _bgr[0] = img; _sharp[0] = sharp
                _cam_n[0] += 1; _cam_t[0] = time.time()


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
