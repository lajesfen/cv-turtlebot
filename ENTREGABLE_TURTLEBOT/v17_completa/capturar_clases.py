#!/usr/bin/env python3
# capturar_clases.py -- CORRE EN EL ROBOT. Sirve la camara en http://<IP>:8000 con BOTONES de clase
# (turn_left / turn_right / stop / meta) y captura para el dataset. v17.5: captura MASIVA optimizada.
#
# NOVEDADES v17.5 (para juntar MUCHAS fotos y subir el confidence):
#   - MODO AUTO: eliges clase, das "AUTO ▶" y guarda una foto cada ~0.4 s mientras mueves/giras la senal
#     (barres distancia, angulo, fondo). En 30-60 s por clase juntas 75-150 fotos variadas. "⏸" para parar.
#   - ANTI-BORROSO: en AUTO descarta frames movidos (nitidez < BLUR_MIN) -> no ensucia el dataset.
#   - ANTI-DUPLICADO: en AUTO descarta frames casi identicos al ultimo guardado -> variedad real, no 50 clones.
#   - NOMBRE SECUENCIAL por clase: {clase}_0001.jpg, 0002... (RETOMA donde quedo, no pisa).
#   - Nitidez en vivo (verde/rojo) y conteo por clase en cada boton.
#
# En el robot:
#   ros2 service call /oakd/start_camera std_srvs/srv/Trigger
#   export ROS_DOMAIN_ID=67 && nohup python3 ~/capturar_clases.py > ~/cap.log 2>&1 &
# En la laptop (navegador):  http://<IP_ROBOT>:8000
# Bajar TODO al terminar:    scp -r ubuntu@<IP>:~/sign_frames "C:\ruta\dataset_nuevo"
# Parar:  pkill -f capturar_clases
import os, time, threading
from urllib.parse import urlparse, parse_qs
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CLASSES = ["turn_left", "turn_right", "stop", "meta", "mixed"]   # 'meta'=bandera cuadros; 'mixed'=2 senales en una foto (NO es clase de entreno, solo carpeta)
BASE = os.path.expanduser("~/sign_frames")
PORT = 8000
BLUR_MIN = 120.0        # nitidez minima para guardar en AUTO (por debajo = borrosa, se descarta)
DIFF_MIN = 6.0          # diferencia media minima vs la ultima guardada (evita casi-duplicados en AUTO)
AUTO_MS = 400           # cada cuanto guarda en AUTO (ms)

_lock = threading.Lock()
_jpeg = [None]
_bgr = [None]
_sharp = [0.0]
_cur = ["turn_left"]
_auto = [False]
_last_gray = [None]     # gray 64x64 de la ultima foto guardada (anti-duplicado)
_count = {c: 0 for c in CLASSES}

for c in CLASSES:
    os.makedirs(os.path.join(BASE, c), exist_ok=True)
    try:
        _count[c] = len([f for f in os.listdir(os.path.join(BASE, c)) if f.endswith(".jpg")])
    except Exception:
        pass


def next_path(c):
    n = _count[c] + 1
    while True:
        fn = os.path.join(BASE, c, f"{c}_{n:04d}.jpg")
        if not os.path.exists(fn):
            return fn, n
        n += 1


def _save(img, c):
    fn, n = next_path(c)
    cv2.imwrite(fn, img)
    _count[c] = n
    _last_gray[0] = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (64, 64))
    return n


def _auto_loop():
    # Hilo de captura automatica: mientras AUTO este ON, guarda cada AUTO_MS si nitida y NO duplicada.
    while True:
        if not _auto[0]:
            time.sleep(0.1); continue
        time.sleep(AUTO_MS / 1000.0)
        with _lock:
            img = None if _bgr[0] is None else _bgr[0].copy()
            sh = _sharp[0]
        if img is None or sh < BLUR_MIN:
            continue                                   # sin frame o borrosa -> no guarda
        g = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (64, 64)).astype(np.float32)
        if _last_gray[0] is not None:
            if float(np.mean(np.abs(g - _last_gray[0].astype(np.float32)))) < DIFF_MIN:
                continue                               # casi identica a la anterior -> no guarda
        with _lock:
            _save(img, _cur[0])


def page():
    btns = ""
    for c in CLASSES:
        bg = "#2a2" if c == _cur[0] else "#444"
        btns += (f"<button style='font-size:18px;padding:8px 14px;margin:4px;background:{bg};"
                 f"color:#fff;border:0;border-radius:6px' "
                 f"onclick=\"fetch('/set?c={c}').then(()=>location.reload())\">{c} (<span class='n_{c}'>{_count[c]}</span>)</button>")
    html = (
        "<html><head><meta charset='utf-8'></head>"
        "<body style='margin:0;background:#111;text-align:center;color:#eee;font-family:sans-serif'>"
        f"<p style='font-size:20px'>Clase actual: <b style='color:#4f4'>{_cur[0]}</b></p>"
        f"<div>{btns}</div>"
        "<button style='font-size:22px;padding:12px 28px;margin:8px;background:#06c;color:#fff;border:0;border-radius:8px' "
        "onclick=\"fetch('/save').then(r=>r.text()).then(t=>msg(t))\">Guardar 1</button>"
        "<button style='font-size:22px;padding:12px 28px;margin:8px;background:#666;color:#fff;border:0;border-radius:8px' "
        "onclick=\"fetch('/burst?n=8').then(r=>r.text()).then(t=>msg(t))\">Rafaga x8</button>"
        "<button id='auto' style='font-size:22px;padding:12px 28px;margin:8px;background:#c40;color:#fff;border:0;border-radius:8px' "
        "onclick=\"toggleAuto()\">AUTO &#9654;</button>"
        "<p id='m' style='font-size:18px;color:#4cf'></p>"
        "<p>nitidez: <b id='sh'>-</b> (rojo=borrosa) &nbsp;|&nbsp; AUTO: <b id='ast'>OFF</b></p>"
        "<img src='/stream.mjpg' style='width:70vw;image-rendering:pixelated'>"
        "<script>"
        "function msg(t){document.getElementById('m').innerText=t;}"
        "let autoOn=false;"
        "function toggleAuto(){autoOn=!autoOn;fetch('/auto?on='+(autoOn?1:0));"
        "  let b=document.getElementById('auto');"
        "  b.innerHTML=autoOn?'AUTO &#9208;':'AUTO &#9654;';b.style.background=autoOn?'#0a0':'#c40';}"
        "setInterval(()=>fetch('/stats').then(r=>r.json()).then(s=>{"
        "  let e=document.getElementById('sh');e.innerText=s.sharp.toFixed(0);"
        "  e.style.color=s.sharp<120?'#f66':'#6f6';"
        "  document.getElementById('ast').innerText=s.auto?'ON (guardando)':'OFF';"
        "  for(const k in s.count){let el=document.querySelector('.n_'+k);if(el)el.innerText=s.count[k];}"
        "}),500);"
        "</script></body></html>")
    return html.encode()


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _txt(self, body, ctype="text/plain"):
        self.send_response(200); self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store"); self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self._txt(page(), "text/html")
        elif u.path == "/set":
            c = parse_qs(u.query).get("c", ["turn_left"])[0]
            if c in CLASSES: _cur[0] = c
            self._txt(b"ok")
        elif u.path == "/save":
            with _lock:
                img = None if _bgr[0] is None else _bgr[0].copy()
                sh = _sharp[0]; c = _cur[0]
            if img is not None:
                with _lock: n = _save(img, c)
                aviso = "  (BORROSA)" if sh < BLUR_MIN else ""
                self._txt(f"Guardada {c} #{n} | nitidez {sh:.0f}{aviso}".encode())
            else:
                self._txt(b"aun no llega frame")
        elif u.path == "/burst":
            n = int(parse_qs(u.query).get("n", ["8"])[0])
            threading.Thread(target=self._burst, args=(n,), daemon=True).start()
            self._txt(f"rafaga de {n}...".encode())
        elif u.path == "/auto":
            _auto[0] = parse_qs(u.query).get("on", ["0"])[0] == "1"
            self._txt(b"ok")
        elif u.path == "/stats":
            import json
            with _lock: sh = _sharp[0]; cnt = dict(_count); au = _auto[0]
            self._txt(json.dumps({"sharp": sh, "count": cnt, "auto": au}).encode(), "application/json")
        elif u.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
            try:
                while True:
                    with _lock: jp = _jpeg[0]
                    if jp is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jp + b"\r\n")
                    time.sleep(0.05)
            except Exception:
                pass
        else:
            self.send_response(404); self.end_headers()

    def _burst(self, n):
        for _ in range(max(1, min(n, 40))):
            with _lock:
                img = None if _bgr[0] is None else _bgr[0].copy(); c = _cur[0]
            if img is not None:
                with _lock: _save(img, c)
            time.sleep(0.12)


class Cam(Node):
    def __init__(self):
        super().__init__("capturar_clases")
        self.bridge = CvBridge()
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        ok, buf = cv2.imencode(".jpg", img)
        if ok:
            with _lock:
                _jpeg[0] = buf.tobytes(); _bgr[0] = img; _sharp[0] = sharp


def main():
    rclpy.init(); node = Cam()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=_auto_loop, daemon=True).start()
    print(f"Captura por clases en http://<IP>:{PORT}  ->  {BASE}/<clase>/")
    print(f"Clases: {CLASSES} | AUTO cada {AUTO_MS}ms, nitidez>={BLUR_MIN}, anti-dup>={DIFF_MIN}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
