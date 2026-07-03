# RECETARIO — Probar QR + correr v17 (estado actual)  ·  sin pista

> Objetivo de HOY: sin circuito, verificar las 3 piezas por separado y luego juntas:
> **(1) QR** se decodifica, **(2) las 4 señales** las clasifica el YOLO, **(3) el robot se MUEVE**
> y esquiva (con tus manos / cajas sueltas). El día de la pista solo repites la Parte 4.

Convención de terminales:
- **ROBOT-#** = una pestaña SSH abierta hacia el robot (todas exportan `ROS_DOMAIN_ID=67`).
- **LAPTOP-#** = una ventana PowerShell/CMD en tu Windows, dentro de la carpeta `v17_completa`.

---

## PARTE 0 — Preparación (haz esto SIEMPRE, una vez por sesión)

### 0.1 Físico
- Robot **FUERA del dock** (en el piso, no en el aire: si lo levantas, el Create-3 corta motores por seguridad "wheel-drop").
- Batería **≥ 30 %** (con menos, a veces no arranca la nav).
- Laptop y robot en la **misma WiFi** (tu router TP-Link).

### 0.2 Averigua la IP del robot (cambia según la red)
En una pestaña SSH o en la pantallita del robot:
```bash
hostname -I        # toma la 1ra IP, ej. 192.168.0.104
```
Anótala. La usarás en 3 sitios: `control_teclas.py`, `--robot-ip` y `--stream` del YOLO.

### 0.3 Conéctate por SSH (abre varias pestañas hacia el robot)
En cada LAPTOP terminal que necesite ser ROBOT:
```bash
ssh ubuntu@<IP_ROBOT>        # ej. ssh ubuntu@192.168.0.104
# escribe la contraseña del robot
export ROS_DOMAIN_ID=67      # <-- OBLIGATORIO en CADA pestaña SSH nueva
```
> Si te sale "no encuentra tópicos" el 90 % de las veces es que **olvidaste el `export ROS_DOMAIN_ID=67`** en esa pestaña.

### 0.4 Sube los archivos al robot (solo si los cambiaste)
Desde LAPTOP (carpeta `v17_completa`):
```bash
scp autonomia_v17.py ubuntu@<IP_ROBOT>:~/
scp ver_y_capturar.py ubuntu@<IP_ROBOT>:~/
scp diag_qr.py        ubuntu@<IP_ROBOT>:~/
scp capturar_clases.py ubuntu@<IP_ROBOT>:~/     # opcional (solo para tomar dataset)
```

### 0.5 Enciende la cámara (una sola vez por encendido del robot)
En **ROBOT-1**:
```bash
ros2 service call /oakd/start_camera std_srvs/srv/Trigger
```
Debe responder `success=True`. La cámara ahora publica en `/oakd/rgb/preview/image_raw`.
Verifícalo:
```bash
ros2 topic hz /oakd/rgb/preview/image_raw      # debe imprimir ~10-30 Hz. Ctrl+C para salir.
```

---

## PARTE 1 — Probar el QR SOLO (sin mover el robot)

`diag_qr.py` solo lee la cámara e imprime cuándo decodifica un QR y **qué % del frame ocupa**
(así sabes a qué distancia máxima lo lee → cuánto acercarte en la pista).

En **ROBOT-1**:
```bash
export ROS_DOMAIN_ID=67
python3 ~/diag_qr.py
```
Acerca/aleja tu QR impreso frente a la cámara. Verás:
```
[OK]  QR='checkpoint_1'  ocupa  7.3% del frame  (frame 250x250)  aciertos=12/40
```
- **Anota el % mínimo** al que aún lo lee (típico: lo agarra desde ~5-8 % del frame).
- Si NUNCA sale `[OK]`: el QR está muy chico/lejos, borroso, o con reflejo. Imprímelo más grande
  (lado ≥ 8-10 cm) y mátalo de frente sin brillo.
- Salir: `Ctrl+C`.

> **Cómo lo detecta v17 (mismo código):** en `img_cb`, cada `1/QR_HZ` seg hace
> `data,_,_ = self.qr.detectAndDecode(img)`. Si `data` es **nuevo** (no repetido), lo suma a
> `self.checkpoints` y lo escribe en `checkpoints_log.txt` con timestamp. Solo cuenta **QRs distintos**
> (los 3 checkpoints deben tener texto diferente). Volver a ver el mismo QR NO suma otra vez.

El núcleo, por si lo quieres explicar:
```python
qr = cv2.QRCodeDetector()
data, pts, _ = qr.detectAndDecode(img)   # data = "" si no hay QR
if data and data not in vistos:
    vistos.add(data)                     # cada checkpoint una sola vez
```

---

## PARTE 2 — Probar las 4 señales SOLO (YOLO en modo prueba, no manda nada al robot)

Aquí necesitas el **stream de la cámara** corriendo en el robot y el **YOLO** en la laptop en `--test`.

**ROBOT-2** (sirve el video en el puerto 8000, sin ROS del lado laptop):
```bash
export ROS_DOMAIN_ID=67
nohup python3 ~/ver_y_capturar.py stream > ~/ver.log 2>&1 &
```
Comprueba desde el navegador de la laptop: abre `http://<IP_ROBOT>:8000` → debes ver el video.

**LAPTOP-1** (YOLO en prueba: imprime `[N] clase conf`, NO envía al robot):
```bash
cd ...\ENTREGABLE_TURTLEBOT\v17_completa
python yolo_win.py --stream http://<IP_ROBOT>:8000/stream.mjpg --robot-ip <IP_ROBOT> --model best.pt --test
```
Muéstrale una por una tus 4 señales (turn_left, turn_right, stop, meta). En consola verás algo como:
```
clases del modelo: {0:'turn_left',1:'turn_right',2:'stop',3:'meta'}
[1] turn_left  conf=0.71  area=6.2%
[2] stop       conf=0.83  area=9.1%
```
Qué mirar:
- **¿Acierta la clase?** (ojo con el histórico problema left↔right).
- **¿Confianza ≥ 0.60?** (es el `--conf` por defecto). Si tus señales dan 0.3-0.5, hay que
  re-entrenar con fotos reales del pipeline (usa `capturar_clases.py`, Parte 5).
- Guarda un CSV para revisar después: agrega `--log prueba_senales.csv`.

> `--test` NO manda comandos, así que puedes calibrar tranquilo. Cuando pases a la Parte 4
> quitas `--test` y ahí sí envía LEFT/RIGHT/SSTOP/META al robot.

---

## PARTE 3 — Probar que el robot SE MUEVE (confirmar el tópico de cmd_vel)

Esto es lo más importante y lo que quedó pendiente: confirmar que `USE_STAMPED=False`
(publicar `Twist` en `/cmd_vel_unstamped`) es el camino correcto en TU robot.

**Con el robot en el piso, espacio libre delante.** En **ROBOT-3**:
```bash
export ROS_DOMAIN_ID=67
ros2 topic pub --rate 10 --qos-reliability best_effort /cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
- **Si avanza** → `USE_STAMPED=False` es correcto (ya está así). Perfecto. `Ctrl+C` para parar.
- **Si NO avanza**, prueba el otro camino:
  ```bash
  ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1}}}"
  ```
  Si con ESTE sí avanza → edita `autonomia_v17.py` línea 65: pon `USE_STAMPED = True`, vuelve a
  hacer `scp`, y usa ese de ahora en adelante.

> Deja el robot listo para frenar (Ctrl+C corta el `pub` y el robot para). No lo dejes yendo hacia una pared.

---

## PARTE 4 — TODO junto (autonomía + QR + señales + control remoto)

4 procesos: **2 en el robot**, **2 en la laptop**. Enciéndelos en este orden.

### ROBOT-1 — la autonomía (nav + QR a bordo)
```bash
export ROS_DOMAIN_ID=67
nohup python3 ~/autonomia_v17.py > ~/auto.log 2>&1 &
pgrep -af autonomia_v17        # confirma que quedó corriendo (debe imprimir un PID)
```
Mira su log en vivo (aquí verás los checkpoints de QR y el modo de navegación):
```bash
tail -f ~/auto.log
# verás lineas [IDLE]/[DRIVE]/... y, al ver un QR:  >>> ... CHECKPOINT 1/3 | QR='...' <<<
```

### ROBOT-2 — el stream para el YOLO
```bash
export ROS_DOMAIN_ID=67
nohup python3 ~/ver_y_capturar.py stream > ~/ver.log 2>&1 &
```
(Si ya lo dejaste de la Parte 2, no lo dupliques: `pgrep -af ver_y_capturar` para chequear.)

### LAPTOP-1 — el YOLO real (ahora SÍ manda señales al robot)
```bash
cd ...\ENTREGABLE_TURTLEBOT\v17_completa
python yolo_win.py --stream http://<IP_ROBOT>:8000/stream.mjpg --robot-ip <IP_ROBOT> --model best.pt
```
> Sin `--show` (tu OpenCV es headless y `--show` revienta). Te basta la consola con las líneas `[ve]`
> y el navegador en `http://<IP_ROBOT>:8000` para ver lo que ve la cámara.

### LAPTOP-2 — el interruptor (armar/pausar por tecla)
1. Edita `control_teclas.py` **línea 7** → `ROBOT_IP = "<IP_ROBOT>"` (la de hoy).
2. Corre:
```bash
python control_teclas.py
```
Teclas (esta ventana debe estar **enfocada**):
- **g** = ARMAR (el robot empieza a navegar)
- **p** o **espacio** = PAUSA
- **t** = alterna
- **q** = STOP y salir

### Secuencia de prueba
1. Todo lo de arriba corriendo. Robot en el piso.
2. En **LAPTOP-2** pulsa **g** → el robot arranca en modo DRIVE (avanza si tiene frente libre).
3. **Esquiva:** pon tu mano o una caja delante/al costado → debe frenar/rodear (modo EXPLORE/PROBE),
   sin chocar. Es lo que vas a afinar en pista.
4. **QR:** acércale el QR a la cámara → en `~/auto.log` sale `>>> ... CHECKPOINT 1/3 | QR='...' <<<`.
   Compruébalo también con:
   ```bash
   cat ~/checkpoints_log.txt
   ```
5. **Señales:** muéstrale turn_left/right/stop/meta a la cámara. En **LAPTOP-1** verás que el YOLO
   envía `LEFT/RIGHT/SSTOP/META`; en `~/auto.log` verás que la nav lo recibe y, al llegar a un cruce,
   gira hacia ese lado (la señal va a un **buffer**: se consulta cuando toca decidir un giro, no gira al
   instante). `stop` frena ~2 s. `meta` para ~10 s y gira 180° (provisional — confirmar con Rensso si el
   intento **termina** en la meta).
6. Para todo: **q** en LAPTOP-2, y en el robot:
   ```bash
   pkill -f autonomia_v17 ; pkill -f ver_y_capturar
   ```

---

## PARTE 5 — (opcional hoy) Tomar dataset de las 4 señales para re-entrenar

Si en la Parte 2 las confianzas salieron bajas o confunde left/right, junta fotos REALES con la
misma cámara. `capturar_clases.py` sirve una web con botones por clase y modo AUTO (una foto cada ~0.4 s).

**ROBOT-2** (para el stream normal antes, para no chocar el puerto 8000):
```bash
pkill -f ver_y_capturar
export ROS_DOMAIN_ID=67
nohup python3 ~/capturar_clases.py > ~/cap.log 2>&1 &
```
En la laptop: abre `http://<IP_ROBOT>:8000`. Elige clase (turn_left/turn_right/stop/meta), dale
**AUTO ▶** y mueve/gira la señal 30-60 s por clase (varía distancia, ángulo, fondo). Descarta solo las
borrosas. Objetivo: ~70+ fotos/clase.

Bajar el dataset a la laptop:
```bash
scp -r ubuntu@<IP_ROBOT>:~/sign_frames "C:\ruta\dataset_nuevo"
```
Parar la captura: `pkill -f capturar_clases`.

---

## Chuleta de puertos / rutas
| Cosa | Valor |
|---|---|
| ROS_DOMAIN_ID | **67** (en cada pestaña SSH) |
| Puerto UDP comandos (laptop→robot) | **5008** |
| Puerto stream cámara (navegador/YOLO) | **8000** → `http://<IP>:8000/stream.mjpg` |
| Tópico de movimiento | `/cmd_vel_unstamped` (Twist) con `USE_STAMPED=False` |
| Log de checkpoints QR | `/home/ubuntu/checkpoints_log.txt` |
| Log de la autonomía | `~/auto.log` |
| Comandos que entiende la nav | `ARM/GO`, `PAUSE/STOP`, `TOGGLE`, `LEFT`, `RIGHT`, `SSTOP`, `META`, `PING` |

## "Si no funciona, corre estos"
```bash
# ¿el nodo está vivo?
pgrep -af autonomia_v17
# ¿llegan tópicos? (¿pusiste el export en ESTA pestaña?)
ros2 topic list | grep -E "scan|odom|image_raw|cmd_vel"
# ¿la cámara publica?
ros2 topic hz /oakd/rgb/preview/image_raw
# ¿el robot está en dock / batería? (no arranca en dock)
ros2 topic echo /dock --once ; ros2 topic echo /battery_state --once
# ¿el YOLO alcanza al robot por UDP? (sonda de red pura, sin YOLO)
python yolo_win.py --stream x --robot-ip <IP_ROBOT> --ping 50
# matar todo en el robot
pkill -f autonomia_v17 ; pkill -f ver_y_capturar ; pkill -f capturar_clases
```
