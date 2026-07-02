#!/usr/bin/env python3
# diag_qr.py -- DIAGNOSTICO de detección de QR. SOLO LEE la cámara, NO mueve el robot.
# Corre en el ROBOT:  export ROS_DOMAIN_ID=67 && python3 ~/diag_qr.py
# (antes: ros2 service call /oakd/start_camera std_srvs/srv/Trigger)
#
# Sirve para saber, ANTES de la pista: ¿el QR se decodifica? ¿a qué tamaño/cercanía?
# Acerca/aleja un QR impreso y mira el % del frame que ocupa cuando recién lo lee.
import time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

QR_HZ = 5.0

class DiagQR(Node):
    def __init__(self):
        super().__init__("diag_qr")
        self.bridge = CvBridge()
        self.qr = cv2.QRCodeDetector()
        self.last = 0.0
        self.frames = 0
        self.hits = 0
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.cb, qos_profile_sensor_data)
        print("Escuchando la cámara... muestra un QR. (Ctrl+C para salir)")

    def cb(self, msg):
        now = time.time()
        if now - self.last < 1.0 / QR_HZ:
            return
        self.last = now
        self.frames += 1
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = img.shape[:2]
        data, pts, _ = self.qr.detectAndDecode(img)
        if data:
            self.hits += 1
            # tamaño del QR en el frame (área del cuadrilátero detectado)
            frac = 0.0
            if pts is not None:
                p = pts.reshape(-1, 2)
                area = cv2.contourArea(p.astype("float32"))
                frac = area / float(w * h)
            print(f"[OK]  QR='{data}'  ocupa {frac*100:4.1f}% del frame  (frame {w}x{h})  "
                  f"aciertos={self.hits}/{self.frames}")
        else:
            if self.frames % 10 == 0:
                print(f"... sin QR  (frames={self.frames}, aciertos={self.hits})")

def main():
    rclpy.init(); n = DiagQR()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
