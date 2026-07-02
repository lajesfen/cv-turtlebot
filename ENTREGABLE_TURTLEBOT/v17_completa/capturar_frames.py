#!/usr/bin/env python3
# capturar_frames.py -- CORRE EN EL ROBOT (Raspberry, headless). Captura frames de la camara para
# entrenar el YOLO con el DOMINIO REAL (250x250, borroso, luz de pista) -> baja la varianza.
#
# Uso:
#   ros2 service call /oakd/start_camera std_srvs/srv/Trigger        (encender camara antes)
#   export ROS_DOMAIN_ID=67
#   python3 ~/capturar_frames.py turn_left      # guarda en ~/sign_frames/turn_left/
#   python3 ~/capturar_frames.py turn_right
#   python3 ~/capturar_frames.py stop
#   python3 ~/capturar_frames.py fondo          # tomas SIN senal (negativos)
#
# Mueve la senal por distintos fondos/distancias/angulos/luz mientras corre. Ctrl+C para parar.
# Luego baja todo a la PC:  scp -r ubuntu@<IP>:~/sign_frames ./   y revisa/borra las malas.
import os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CLASE = sys.argv[1] if len(sys.argv) > 1 else "misc"
OUT = os.path.expanduser(f"~/sign_frames/{CLASE}")
SAVE_EVERY = 4          # guarda 1 de cada 4 frames (~7/seg). Sube el numero si quieres menos.

class Cap(Node):
    def __init__(self):
        super().__init__("capturar_frames")
        os.makedirs(OUT, exist_ok=True)
        self.bridge = CvBridge(); self.n = 0; self.saved = 0
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.cb, qos_profile_sensor_data)
        print(f"Clase '{CLASE}' -> guardando en {OUT} (1 cada {SAVE_EVERY}). Mueve la senal. Ctrl+C para parar.")
    def cb(self, msg):
        self.n += 1
        if self.n % SAVE_EVERY: return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(os.path.join(OUT, f"{CLASE}_{int(time.time()*1000)}.jpg"), img)
        self.saved += 1
        if self.saved % 10 == 0:
            print(f"  {CLASE}: {self.saved} guardadas")

def main():
    rclpy.init(); n=Cap()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    print(f"TOTAL '{CLASE}': {n.saved} en {OUT}")
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
