#!/usr/bin/env python3
# diag_qr.py -- DIAGNOSTICO de QR (robot, robot QUIETO). No mueve nada.
# Corre:  export ROS_DOMAIN_ID=67 && python3 ~/diag_qr.py [topic]
#   [topic] opcional. Default /oakd/rgb/preview/image_raw (chico). Si hay uno mas grande, pruebalo:
#     ros2 topic list | grep oakd     # busca p.ej. /oakd/rgb/image_raw
#     python3 ~/diag_qr.py /oakd/rgb/image_raw
#
# Prueba VARIOS metodos (OpenCV gris+ampliado, multi, y pyzbar si esta) y GUARDA ~/qr_test.jpg para
# que lo bajes y veas si el QR se ve nitido y grande. pyzbar es MUCHO mas robusto:
#     sudo apt-get install -y libzbar0 && pip3 install pyzbar --break-system-packages
import sys, os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

try:
    from pyzbar.pyzbar import decode as zbar_decode
    HAS_ZBAR = True
except Exception:
    HAS_ZBAR = False

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/oakd/rgb/preview/image_raw"


class DiagQR(Node):
    def __init__(self):
        super().__init__("diag_qr")
        self.bridge = CvBridge()
        self.qr = cv2.QRCodeDetector()
        self.frames = self.hits = self.saved = 0
        self.printed = False
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)
        print(f"Escuchando {TOPIC} | pyzbar={'SI' if HAS_ZBAR else 'NO -> instala: sudo apt install -y libzbar0 && pip3 install pyzbar --break-system-packages'}")
        print("Muestra un QR CERCA y grande, plano y bien iluminado. Guardo ~/qr_test.jpg. (Ctrl+C sale)")

    def try_decode(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        up = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)  # ampliar ayuda con preview chico
        out = []
        d, _, _ = self.qr.detectAndDecode(up)
        if d:
            out.append(("cv2/gray3x", d))
        try:
            res = self.qr.detectAndDecodeMulti(up)
            if res[0]:
                for t in res[1]:
                    if t:
                        out.append(("cv2/multi", t))
        except Exception:
            pass
        if HAS_ZBAR:
            for o in zbar_decode(gray):
                out.append(("pyzbar", o.data.decode("utf-8", "ignore")))
        return out

    def cb(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = bgr.shape[:2]
        if not self.printed:
            print(f">>> resolucion de {TOPIC}: {w}x{h}  (si es <=250, el QR debe estar MUY cerca/grande)")
            self.printed = True
        self.frames += 1
        cands = self.try_decode(bgr)
        if cands:
            self.hits += 1
            for m, t in cands:
                print(f"[OK/{m}] QR='{t}'  (frame {w}x{h})")
        if self.frames % 30 == 0:
            fn = os.path.expanduser("~/qr_test.jpg")
            cv2.imwrite(fn, bgr); self.saved += 1
            if not cands:
                print(f"... sin QR (frames={self.frames}, aciertos={self.hits}) | guarde {fn} #{self.saved} "
                      f"-> bajalo (scp) y mira si el QR se ve nitido y ocupa buena parte del frame")


def main():
    rclpy.init(); n = DiagQR()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
