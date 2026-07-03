#!/usr/bin/env python3
# yolo_win.py -- CORRE EN WINDOWS (o cualquier PC), SIN ROS.
# Lee el MJPEG del robot (ver_y_capturar.py :8000), corre YOLO (best.pt) y manda por UDP la senal a
# autonomia_v17 (LEFT/RIGHT/SSTOP). Trae CAPAS DE DIAGNOSTICO (v17.2):
#   - Hilo LECTOR que se queda con el ULTIMO frame. Arregla la latencia que CRECIA sola: cv2.VideoCapture
#     bufferea y cap.read() devolvia el frame mas VIEJO -> el robot obedecia una senal vista hace 1-2 s.
#   - VENTANA DE VOTACION: en vez de exigir N frames identicos seguidos, gana la clase con mas votos
#     entre los ultimos VOTE_WINDOW frames -> amortigua falsos positivos (un frame borroso NO dispara).
#   - Cada comando lleva "#seq"; el robot responde "ACK#seq" -> mide RTT y % de paquetes perdidos.
#   - Resumen periodico: FPS del stream, ms de inferencia, latencia extremo-a-extremo, RTT, perdida.
#   - Modo --ping N: sonda UDP pura (sin YOLO) para latencia/perdida de red.
#
# REQUISITOS (una vez):  pip install ultralytics opencv-python
# EN EL ROBOT:  ver_y_capturar.py sirviendo el stream + autonomia_v17.py corriendo.
# EN WINDOWS:
#   python yolo_win.py --stream http://192.168.0.104:8000/stream.mjpg --robot-ip 192.168.0.104 --model best.pt --show
#   python yolo_win.py --stream x --robot-ip 192.168.0.104 --ping 100      # solo sonda de red (sin YOLO)
import argparse, socket, time, threading, csv
from collections import deque, Counter
import cv2

CLASS_TO_CMD = {"turn_left": "LEFT", "turn_right": "RIGHT", "stop": "SSTOP", "meta": "META"}


class FrameGrabber:
    """Hilo que lee el stream SIN parar y guarda SOLO el ultimo frame (mata la latencia acumulada)."""
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # pide buffer chico (algunos backends lo ignoran;
        except Exception:                              #   por eso ademas leemos siempre el ultimo abajo)
            pass
        self.lock = threading.Lock()
        self.frame = None
        self.idx = 0            # numero de frame; sirve para saber si llego uno NUEVO
        self.t_frame = 0.0      # instante en que se leyo (para la latencia extremo-a-extremo)
        self.grabbed = 0        # total leidos (para FPS del stream)
        self.fail = 0
        self.run = True
        self.opened = self.cap.isOpened()
        if self.opened:
            threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.run:
            ok, img = self.cap.read()          # drena el stream lo mas rapido que puede
            if not ok:
                self.fail += 1; time.sleep(0.02); continue
            with self.lock:
                self.frame = img; self.idx += 1
                self.t_frame = time.time(); self.grabbed += 1

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, -1, 0.0
            return self.frame, self.idx, self.t_frame

    def stop(self):
        self.run = False
        try: self.cap.release()
        except Exception: pass


def udp_ping(sock, dest, n, interval):
    """Sonda de red pura: manda n PING#seq, escucha PONG#seq, reporta RTT y perdida (el robot hace de eco)."""
    got, t_send, stop = {}, {}, [False]

    def listen():
        sock.settimeout(0.3)
        while not stop[0]:
            try:
                data, _ = sock.recvfrom(256)
                r = data.decode(errors="ignore").strip()
                if r.startswith("PONG#"):
                    seq = r.split("#", 1)[1]
                    if seq in got and got[seq] is None:
                        got[seq] = time.time()
            except socket.timeout:
                continue
            except Exception:
                break
    threading.Thread(target=listen, daemon=True).start()

    for i in range(n):
        seq = str(i); got[seq] = None; t_send[seq] = time.time()
        sock.sendto(f"PING#{seq}".encode(), dest)
        time.sleep(interval)
    time.sleep(0.5); stop[0] = True

    rtts = sorted((got[s] - t_send[s]) * 1000 for s in got if got[s] is not None)
    lost = sum(1 for s in got if got[s] is None)
    print(f"\n=== SONDA UDP a {dest[0]}:{dest[1]}  ({n} PING) ===")
    if rtts:
        p = lambda q: rtts[min(len(rtts) - 1, int(q * len(rtts)))]
        print(f"RTT ms: min={rtts[0]:.1f}  avg={sum(rtts)/len(rtts):.1f}  "
              f"p50={p(0.5):.1f}  p95={p(0.95):.1f}  max={rtts[-1]:.1f}")
    else:
        print("RTT ms: (ninguna respuesta -> el robot no escucha, IP/puerto malos, o red caida)")
    flag = "  <- REVISAR RED/robot" if lost else "  OK"
    print(f"Perdida: {lost}/{n} = {100*lost/n:.1f}%{flag}")


def main():
    ap = argparse.ArgumentParser(description="YOLO Win + diagnostico (latencia/perdida/votacion)")
    ap.add_argument("--stream", required=True)
    ap.add_argument("--robot-ip", required=True)
    ap.add_argument("--port", type=int, default=5008)
    ap.add_argument("--model", default="best.pt")
    ap.add_argument("--conf", type=float, default=0.60)
    ap.add_argument("--min-area", type=float, default=0.03, help="fraccion minima del frame (cercania)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--cooldown", type=float, default=5.0)
    ap.add_argument("--vote-window", type=int, default=6, help="frames recientes en la ventana de votacion")
    ap.add_argument("--vote-min", type=int, default=4, help="votos minimos de la clase ganadora para disparar")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--log", default=None, help="CSV con cada deteccion (para analizar despues)")
    ap.add_argument("--ping", type=int, default=0, help="modo SONDA de red: manda N PING y sale (sin YOLO)")
    ap.add_argument("--stats-every", type=float, default=2.0, help="seg entre resumenes de diagnostico")
    ap.add_argument("--test", action="store_true", help="PRUEBA del modelo: log numerado [N] clase, NO manda al robot")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.robot_ip, args.port)

    if args.ping > 0:                       # ---- modo sonda de red, sin cargar YOLO ----
        udp_ping(sock, dest, args.ping, 0.02)
        return

    from ultralytics import YOLO           # se importa aqui para que --ping no lo necesite
    model = YOLO(args.model)
    print(f"clases del modelo: {model.names}")   # p.ej. {0:'turn_left',1:'turn_right',2:'stop',3:'meta'}
    if args.test: print("=== MODO PRUEBA: log numerado, NO se manda nada al robot ===")
    show = args.show   # se apaga solo si opencv es headless (sin ventana)
    def _key(k):
        # teclas DESDE la ventana del YOLO: g=ARM, p/espacio=PAUSA, q=salir. Asi 1 solo .py en la laptop.
        if k == ord("g"): sock.sendto(b"ARM", dest); print(">>> tecla g -> ARM")
        elif k in (ord("p"), ord(" ")): sock.sendto(b"PAUSE", dest); print(">>> tecla p -> PAUSA")
        elif k == ord("q"): print(">>> tecla q -> salir"); return True
        return False
    if show: print("Ventana YOLO: g=ARMAR | p/espacio=PAUSA | q=salir  (enfoca la ventana)")
    grab = FrameGrabber(args.stream)
    if not grab.opened:
        print(f"NO pude abrir el stream: {args.stream}  (¿ver_y_capturar corriendo? ¿IP/puerto?)")
        return
    print(f"YOLO Win+diag | stream={args.stream} | destino={dest} | conf>={args.conf} "
          f"| votacion {args.vote_min}/{args.vote_window}")

    # --- ACK listener: mide RTT y perdida de los comandos REALES (LEFT/RIGHT/SSTOP) ---
    ack_rtt = deque(maxlen=50); pend = {}; sent = [0]; acked = [0]

    def ack_listen():
        sock.settimeout(0.3)
        while True:
            try:
                data, _ = sock.recvfrom(256)
                r = data.decode(errors="ignore").strip()
                if r.startswith("ACK#"):
                    seq = r.split("#", 1)[1]
                    if seq in pend:
                        ack_rtt.append((time.time() - pend.pop(seq)) * 1000); acked[0] += 1
            except socket.timeout:
                continue
            except Exception:
                break
    threading.Thread(target=ack_listen, daemon=True).start()

    logw = None
    if args.log:
        logf = open(args.log, "w", newline="")
        logw = csv.writer(logf)
        logw.writerow(["t", "class", "conf", "area", "infer_ms", "e2e_ms", "fired"])

    votes = deque(maxlen=args.vote_window)      # clase (o None) de cada frame reciente
    last_idx, last_fire = -1, 0.0
    infer_ms, e2e_ms = deque(maxlen=60), deque(maxlen=60)
    t_stat, grabbed0, processed = time.time(), 0, 0

    while True:
        now = time.time()
        img, idx, t_frame = grab.latest()
        if img is not None and idx != last_idx:     # SOLO procesa frames NUEVOS (descarta viejos = baja latencia)
            last_idx = idx; processed += 1
            t0 = time.time()
            res = model.predict(img, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
            infer_ms.append((time.time() - t0) * 1000)
            h, w = img.shape[:2]; area_img = float(h * w)
            best = None
            for b in res.boxes:
                cf = float(b.conf)
                if cf < args.conf: continue
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                af = ((x2 - x1) * (y2 - y1)) / area_img
                if af < args.min_area: continue        # muy chica -> lejos/espuria
                nm = model.names[int(b.cls)]
                if nm not in CLASS_TO_CMD: continue
                if best is None or af > best[0]: best = (af, nm, cf)
            if show:
                try:
                    cv2.imshow("yolo", res.plot())
                    if _key(cv2.waitKey(1) & 0xFF): break
                except Exception:
                    show = False; print("[aviso] --show no disponible (opencv sin GUI): sigo SIN ventana, el diagnostico igual corre.")
            e2e = (time.time() - t_frame) * 1000       # frame LEIDO -> decision (latencia real de percepcion)
            e2e_ms.append(e2e)
            cls = best[1] if best else None
            votes.append(cls)
            fired = ""
            if best:
                af, nm, cf = best
                print(f"[ve] {nm} conf={cf:.2f} area={af:.3f} e2e={e2e:.0f}ms "
                      f"votos={dict(Counter(v for v in votes if v))}")
            # VOTACION: gana la clase con >= vote_min apariciones en la ventana (amortigua falso positivo)
            if now - last_fire >= args.cooldown:
                cnt = Counter(v for v in votes if v is not None)
                if cnt:
                    win, nv = cnt.most_common(1)[0]
                    if nv >= args.vote_min:
                        sent[0] += 1
                        if args.test:                       # PRUEBA: solo log numerado, NO manda al robot
                            cf = f", conf~{best[2]:.2f}" if best else ""
                            print(f"[{sent[0]}] {win}   ({nv}/{len(votes)} votos{cf})")
                        else:
                            cmd = CLASS_TO_CMD.get(win, win); seq = str(sent[0])
                            pend[seq] = time.time()
                            sock.sendto(f"{cmd}#{seq}".encode(), dest)
                            print(f">>> ENVIADO '{cmd}#{seq}' ({nv}/{len(votes)} votos) a {dest[0]}")
                            fired = cmd
                        last_fire = now; votes.clear()
            if logw:
                logw.writerow([f"{now:.3f}", cls or "", f"{best[2]:.3f}" if best else "",
                               f"{best[0]:.3f}" if best else "", f"{infer_ms[-1]:.1f}", f"{e2e:.1f}", fired])
        else:
            if show and img is not None:
                try:
                    cv2.imshow("yolo", img)
                    if _key(cv2.waitKey(1) & 0xFF): break
                except Exception:
                    show = False; print("[aviso] --show no disponible (opencv sin GUI): sigo SIN ventana.")
            time.sleep(0.005)

        # --- resumen de diagnostico cada stats_every segundos ---
        if now - t_stat >= args.stats_every:
            dt = now - t_stat
            fps_recv = (grab.grabbed - grabbed0) / dt
            im = sum(infer_ms) / len(infer_ms) if infer_ms else 0
            ee = sum(e2e_ms) / len(e2e_ms) if e2e_ms else 0
            rtt = (sum(ack_rtt) / len(ack_rtt)) if ack_rtt else -1
            loss = 100 * (sent[0] - acked[0]) / sent[0] if sent[0] else 0
            drop = fps_recv - processed / dt                       # frames viejos descartados (bueno: baja latencia)
            print(f"--- DIAG | streamFPS={fps_recv:.1f} procFPS={processed/dt:.1f} "
                  f"(descarta {max(drop,0):.1f}/s viejos) | infer={im:.0f}ms e2e={ee:.0f}ms "
                  f"| cmds={sent[0]} ACKrtt={rtt:.0f}ms perdida={loss:.0f}% | fails_lectura={grab.fail}")
            t_stat = now; grabbed0 = grab.grabbed; processed = 0


if __name__ == "__main__":
    main()
