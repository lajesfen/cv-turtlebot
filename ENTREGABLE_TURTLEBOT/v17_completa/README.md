# v17_completa — Navegación + señales YOLO + memoria anti-loop + QR (VERSIÓN DE COMPETENCIA)

La versión más completa. v16 + **memoria de trayectoria** (rejilla de "migas de pan" en `/odom`
que penaliza volver por donde ya pasó) + fixes v17.1: bumper por lado real del golpe, señal que
expira por tiempo Y distancia (anti-fantasma), rescate rápido (PROBE 1.5 s), y borra la memoria
al registrar cada checkpoint QR.

> Para comportarte EXACTO como v16 (sin memoria anti-loop) pon `USE_MEMORY = False` en la
> línea 145 de `autonomia_v17.py`.

## Qué corre dónde
| Archivo | Máquina | Rol |
|---|---|---|
| `autonomia_v17.py` | ROBOT (Pi) | Navegación + recibe señales YOLO + memoria + log QR |
| `ver_y_capturar.py` | ROBOT | Sirve el video en `http://<IP>:8000` para que la laptop lo lea |
| `yolo_win.py` + `best.pt` | LAPTOP (Windows) | Lee el stream MJPEG, corre YOLO, manda LEFT/RIGHT/SSTOP por UDP |
| `control_teclas.py` | LAPTOP (Windows) | ARM / PAUSA / STOP por UDP 5008 |
| `diag_lidar.py`, `diag_qr.py`, `capturar_frames.py` | ROBOT | OPCIONALES |

**No subas `yolo_win.py` ni `best.pt` a la Pi.** La laptop lee el video por HTTP (sin ROS).

---

## Comandos paso a paso

Necesitarás **3 ventanas** en la laptop: una SSH al robot, una para YOLO, una para control_teclas.

### Paso 0 — Verificar IP y editar si cambió
La IP del robot está hardcodeada como `192.168.0.104`.
Si cambió, edítala en:
- `control_teclas.py` línea 7 (`ROBOT_IP`)
- el argumento `--robot-ip` del comando de `yolo_win.py`
- los comandos `scp` y `ssh` de abajo

Para encontrar la IP actual del robot:
```bash
# (en el robot, por SSH o serial):
hostname -I   # anota la primera IP
```

### Paso 1 — Enviar los archivos al robot
Desde PowerShell en la laptop, dentro de la carpeta `v17_completa/`:
```powershell
scp autonomia_v17.py ver_y_capturar.py ubuntu@192.168.0.104:~/
# contraseña SSH: turtlebot4
```
**No envíes `yolo_win.py` ni `best.pt` al robot.** Se quedan en la laptop.

### Paso 2 — Conectarse al robot por SSH (ventana 1)
```powershell
ssh ubuntu@192.168.0.104
# contraseña: turtlebot4
```

### Paso 3 — Lanzar los dos procesos en el robot
Dentro de la sesión SSH (todo en el robot):
```bash
export ROS_DOMAIN_ID=67
pkill -f autonomia_ ; pkill -f ver_y_capturar ; pkill -f teleop ; sleep 2
ros2 service call /oakd/start_camera std_srvs/srv/Trigger
nohup python3 ~/autonomia_v17.py         > ~/auto.log    2>&1 &
nohup python3 ~/ver_y_capturar.py stream > ~/stream.log  2>&1 &
tail -f ~/auto.log
```
Cuando veas `[IDLE] min_global=0.XX @ YY deg`, el nodo de autonomía está activo.
Deja esta terminal abierta (muestra el log en tiempo real).

### Paso 4 — Verificar el stream de video (laptop, navegador)
Abre en cualquier navegador de la laptop:
```
http://192.168.0.104:8000
```
Debes ver la imagen en vivo de la cámara. **Si no carga, no lances el YOLO todavía.**
Si no carga: revisa `cat ~/stream.log` en el robot para ver el error.

### Paso 5 — Lanzar YOLO en la laptop (ventana 2, nueva PowerShell)
Abre una segunda PowerShell, ve a la carpeta `v17_completa/` y ejecuta:
```powershell
pip install ultralytics opencv-python     # solo la primera vez
python yolo_win.py --stream http://192.168.0.104:8000/stream.mjpg --robot-ip 192.168.0.104 --model best.pt --show
```
La ventana `--show` muestra lo que detecta en tiempo real. Verifica que las 3 clases
(`turn_left`, `turn_right`, `stop`) se detectan correctamente **antes** de armar el robot.

### Paso 6 — Armar el robot (ventana 3, nueva PowerShell)
Abre una **tercera ventana PowerShell**, ve a la carpeta `v17_completa/` y ejecuta:
```powershell
python control_teclas.py
```

Teclas disponibles:
| Tecla | Acción |
|---|---|
| `g` | **ARMAR** — el robot empieza a moverse |
| `p` o `espacio` | **PAUSA** — el robot se detiene (puede reanudarse con `g`) |
| `t` | TOGGLE arm/pause |
| `q` | **STOP** y cerrar el control |

> **Desacopla el robot del dock antes de mandar `g`.**
> **Tras armar, NO vuelvas a tocar teclas** (intervención = intento terminado).

---

## Ver log y detener
Desde la sesión SSH en el robot:
```bash
# Checkpoints QR registrados:
cat ~/checkpoints_log.txt

# Log de la autonomía en tiempo real:
tail -f ~/auto.log

# Detener todo:
pkill -f autonomia_ ; pkill -f ver_y_capturar
```

---

## Checklist antes de competir
- [ ] `STAGE = 1/2/3` en `autonomia_v17.py` (editar antes del `scp`).
- [ ] Stream carga en `http://192.168.0.104:8000`.
- [ ] YOLO detecta las 3 señales en la ventana `--show` (con el robot quieto apuntando hacia carteles de prueba).
- [ ] `diag_qr.py` lee bien los QR de los checkpoints (opcional si hay tiempo).
- [ ] Robot desacoplado del dock.

---

## Calibración antes de competir (opcional)

### Calibrar FRONT_DEG (offset del LiDAR)
```bash
export ROS_DOMAIN_ID=67
python3 ~/diag_lidar.py
```
Pon un objeto plano justo al frente del robot. Anota el valor sugerido y ponlo en `FRONT_DEG`
(línea 107 de `autonomia_v17.py`). **Valor calibrado actual: `-90.0`**.

### Verificar lectura de QR
```bash
export ROS_DOMAIN_ID=67
ros2 service call /oakd/start_camera std_srvs/srv/Trigger
python3 ~/diag_qr.py
```

---

## Diales (ajuste fino en pista)

### En el robot (`autonomia_v17.py`)
| Variable | Valor actual | Qué hace |
|---|---|---|
| `STAGE` | 1 | Pon 1, 2 o 3 según el stage (etiqueta el log del QR) |
| `LIN_MAX` | 0.26 | Velocidad máxima (m/s) |
| `D_BLOCK` | 0.45 | Distancia (m) a la que frena. No bajar de 0.40 |
| `W_VISIT` | 0.60 | Peso del castigo anti-loop. Bájalo si zigzaguea por deriva del odom |
| `MEM_CELL` | 0.25 | Tamaño de celda de la memoria (m) |
| `K_SIDE` | 1.5 | Repulsión lateral anti-roce |
| `PROBE_AFTER` | 1.5 | Segundos atascado antes del rescate lento |
| `USE_MEMORY` | True | En `False` se comporta exacto como v16 (sin memoria anti-loop) |
| `FRONT_DEG` | -90.0 | Offset del LiDAR. Calibrado con `diag_lidar.py` |

### En la laptop (`yolo_win.py`)
| Argumento | Valor por defecto | Qué hace |
|---|---|---|
| `--conf` | 0.60 | Confianza mínima de detección. Baja a 0.50 si no detecta bien |
| `--consec` | 4 | Frames seguidos con la misma clase antes de actuar. Baja a 2 si reacciona tarde |
| `--min-area` | 0.03 | Fracción mínima del frame (filtra señales lejanas/pequeñas) |
| `--cooldown` | 5.0 | Segundos de espera entre señales consecutivas |

> **`SIGN_W` NO es un dial** (no tiene efecto en v17; la señal en bifurcación es prioridad dura).

---

## Si el YOLO/laptop falla el día de la competencia
Cae a **v14** (misma navegación, sin depender de la laptop). En el robot:
```bash
pkill -f autonomia_ ; pkill -f ver_y_capturar ; sleep 2
nohup python3 ~/autonomia_v14.py > ~/auto.log 2>&1 &
tail -f ~/auto.log
```
En la laptop (una sola PowerShell, carpeta `v14_joyita/`):
```powershell
python control_teclas.py
```
Recuerda enviar `autonomia_v14.py` al robot antes con `scp` (Paso 1 del README de v14).

---

## Troubleshooting

| Síntoma | Causa probable | Solución |
|---|---|---|
| `control_teclas.py` no conecta | IP incorrecta | Edita `ROBOT_IP` en línea 7 de `control_teclas.py` |
| Robot no se mueve al `g` | `ROS_DOMAIN_ID` distinto o robot en dock | Verifica `export ROS_DOMAIN_ID=67` en el robot; desacopla del dock |
| Stream no carga en el navegador | `ver_y_capturar.py` no lanzado o cámara apagada | Revisa `cat ~/stream.log`; ejecuta el `ros2 service call /oakd/start_camera ...` primero |
| YOLO no abre el stream | URL incorrecta o stream caído | Confirma que el navegador carga `http://192.168.0.104:8000` primero |
| Robot no gira ante señales | YOLO no detecta o `--consec` muy alto | Baja `--conf 0.50 --consec 2` en `yolo_win.py`; verifica la ventana `--show` |
| Robot se va a un lado siempre | `FRONT_DEG` mal calibrado | Usa `diag_lidar.py` para recalibrar |
| No lee QR / log vacío | Cámara apagada | Ejecuta `ros2 service call /oakd/start_camera std_srvs/srv/Trigger` antes de lanzar |
| `python control_teclas.py` falla en Linux/Mac | `msvcrt` solo existe en Windows | Corre `control_teclas.py` únicamente en Windows |
| YOLO envía señal pero robot no gira | Señal ya expiró (TTL o distancia) | Acerca más el robot a la señal; `SIGN_TTL=10s`, `SIGN_MAX_DIST=1.5m` |
