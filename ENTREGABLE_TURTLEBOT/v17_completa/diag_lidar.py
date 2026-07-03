#!/usr/bin/env python3
# diag_lidar.py -- DIAGNOSTICO del LiDAR. SOLO LEE /scan, NO mueve el robot.
# Corre:  export ROS_DOMAIN_ID=67 && python3 ~/diag_lidar.py
# Pon el robot quieto con un objeto/pared justo AL FRENTE y mira los numeros.
# Sirve para: (1) calibrar FRONT_DEG, (2) ver si el LiDAR esta sano (invalidos),
#             (3) confirmar si el codigo encuentra un hueco transitable.
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

FRONT_DEG = 0.0          # el angulo que CREES que es el frente; el script te dice si acertaste
R_MIN, R_MAX = 0.06, 12.0
D_CLEAR = 0.70           # un rayo es "libre" si supera esto (igual que el nodo)
ROBOT_PASS = 0.42        # ancho que debe caber el robot
SEARCH_HALF_DEG = 120.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Diag(Node):
    def __init__(self):
        super().__init__('diag_lidar')
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.last = 0.0
        print("Escuchando /scan ... (Ctrl+C para salir)")

    def sect(self, rr, amin, ainc, center_deg, half_deg=15.0):
        c = math.radians(center_deg + FRONT_DEG)
        h = math.radians(half_deg)
        best = R_MAX
        for i, r in enumerate(rr):
            if R_MIN < r < R_MAX and abs(wrap(amin + i * ainc - c)) <= h:
                best = min(best, r)
        return best

    def best_gap(self, rr, amin, ainc):
        fr = math.radians(FRONT_DEG)
        search = math.radians(SEARCH_HALF_DEG)
        pts = []
        for i, r in enumerate(rr):
            rel = wrap(amin + i * ainc - fr)
            if abs(rel) > search:
                continue
            if (r != r) or math.isinf(r):
                d = R_MAX
            elif r <= R_MIN:
                d = 0.0
            else:
                d = r
            pts.append((rel, d))
        if not pts:
            return None
        pts.sort()
        best = None
        n = len(pts); i = 0
        while i < n:
            if pts[i][1] > D_CLEAR:
                j = i; dmin = pts[i][1]
                while j + 1 < n and pts[j + 1][1] > D_CLEAR:
                    j += 1; dmin = min(dmin, pts[j][1])
                dw = pts[j][0] - pts[i][0]
                if 2 * dmin * math.sin(max(dw, 0.0) / 2.0) >= ROBOT_PASS:
                    c = 0.5 * (pts[i][0] + pts[j][0])
                    if best is None or abs(c) < abs(best[0]):
                        best = (c, dmin, math.degrees(dw))
                i = j + 1
            else:
                i += 1
        return best

    def cb(self, m):
        now = time.time()
        if now - self.last < 1.0:
            return
        self.last = now
        rr = list(m.ranges); n = len(rr)
        amin, ainc = m.angle_min, m.angle_increment
        inf = sum(1 for r in rr if (r != r) or math.isinf(r))
        zero = sum(1 for r in rr if r <= R_MIN)
        valid = [(amin + i * ainc, r) for i, r in enumerate(rr) if R_MIN < r < R_MAX]
        if valid:
            ang_min, d_min = min(valid, key=lambda t: t[1])
        else:
            ang_min, d_min = 0.0, float('nan')
        front = self.sect(rr, amin, ainc, 0)
        left  = self.sect(rr, amin, ainc, 90)
        right = self.sect(rr, amin, ainc, -90)
        back  = self.sect(rr, amin, ainc, 180)
        gap = self.best_gap(rr, amin, ainc)

        print("=" * 64)
        print(f"rayos={n}  ang=[{math.degrees(amin):.0f},{math.degrees(m.angle_max):.0f}]deg "
              f"inc={math.degrees(ainc):.2f}deg  rango=[{m.range_min:.2f},{m.range_max:.1f}]m")
        print(f"invalidos: inf/nan={inf} ({100*inf/n:.0f}%)  cero/<min={zero} ({100*zero/n:.0f}%)")
        print(f"MIN GLOBAL: {d_min:.2f} m @ {math.degrees(ang_min):.0f} deg")
        print(f"   -> si tienes el objeto AL FRENTE, pon  FRONT_DEG = {math.degrees(ang_min):.0f}")
        print(f"con FRONT_DEG={FRONT_DEG:.0f}:  FRENTE={front:.2f}  IZQ(+90)={left:.2f}  "
              f"DER(-90)={right:.2f}  ATRAS(180)={back:.2f}  (m)")
        if gap is None:
            print("MEJOR HUECO: NINGUNO pasa el ancho del robot en +-120deg "
                  "(o todo bloqueado, o ROBOT_PASS muy alto)")
        else:
            print(f"MEJOR HUECO: rumbo={math.degrees(gap[0]):.0f}deg  dist={gap[1]:.2f}m  ancho={gap[2]:.0f}deg")


def main():
    rclpy.init()
    node = Diag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
