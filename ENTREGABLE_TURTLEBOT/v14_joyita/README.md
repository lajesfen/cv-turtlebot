# v14_joyita — Mejor navegación probada (RECOMENDADA para nav pura / Stage 1 / fallback)

Base v12 + **PROBE** (se asoma antes de comprometer el giro) + **ley de velocidad** (acelera en
recto, frena progresivo en curva) + **PD afinado** (`KP=1.1`, `KD=0.40`, `LIN_MAX=0.26`). Menos
zigzag, más firme. La que mejor se portó en pista. Úsala en Stage 1 (sprint) y cuando el YOLO no
esté listo. Registra QR automáticamente.

## Qué corre dónde
| Archivo | Máquina | Rol |
|---|---|---|
| `autonomia_v14.py` | ROBOT (Pi) | Navegación + log de QR |
| `control_teclas.py` | LAPTOP (Windows) | ARM / PAUSA / STOP por UDP 5008 |
| `diag_lidar.py` | ROBOT | OPCIONAL: calibra FRONT_DEG |
| `diag_qr.py` | ROBOT | OPCIONAL: verifica lectura de QR |

---

## Comandos paso a paso

### Paso 0 — Verificar IP y editar si cambió
La IP del robot está hardcodeada como `192.168.0.104` (línea 7 de `control_teclas.py`).
Si cambió, edítala **antes de continuar**. Para encontrarla:
```bash
# en el robot (SSH o serial):
hostname -I   # anota la primera IP
```

### Paso 1 — Enviar el script al robot
Desde PowerShell en la laptop, dentro de la carpeta `v14_joyita/`:
```powershell
scp autonomia_v14.py ubuntu@192.168.0.104:~/
# contraseña SSH: turtlebot4
```

### Paso 2 — Conectarse al robot por SSH
```powershell
ssh ubuntu@192.168.0.104
# contraseña: turtlebot4
```

### Paso 3 — Lanzar la autonomía en el robot
Ejecuta esto dentro de la sesión SSH (todo en el robot):
```bash
export ROS_DOMAIN_ID=67
pkill -f autonomia_ ; pkill -f teleop ; sleep 2
ros2 service call /oakd/start_camera std_srvs/srv/Trigger
nohup python3 ~/autonomia_v14.py > ~/auto.log 2>&1 &
tail -f ~/auto.log
```

Cuando veas líneas como `[IDLE] min_global=0.XX @ YY deg`, el nodo está activo y esperando ARM.
Deja esta terminal abierta (muestra el log en tiempo real).

### Paso 4 — Armar el robot
Abre **una segunda ventana PowerShell** (sin cerrar la SSH), ve a la carpeta `v14_joyita/` y ejecuta:
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
> **Tras armar, NO toques más teclas** hasta que termine el intento (intervención = intento terminado).

---

## Ver log y detener
Desde la sesión SSH en el robot:
```bash
# Log de la autonomía en tiempo real (ya está corriendo con tail -f):
tail -f ~/auto.log

# Checkpoints QR registrados:
cat ~/checkpoints_log.txt

# Detener la autonomía:
pkill -f autonomia_
```

---

## Calibración antes de competir (opcional pero recomendada)

### Calibrar FRONT_DEG (offset del LiDAR)
Si el robot se va siempre hacia el mismo lado sin razón, el offset del LiDAR está mal.
En el robot:
```bash
export ROS_DOMAIN_ID=67
python3 ~/diag_lidar.py
```
Pon un objeto plano justo al frente del robot. La salida indica el valor a poner en `FRONT_DEG`
(línea 125 de `autonomia_v14.py`). **Valor calibrado actual: `-90.0`** (ya está correcto para
este robot; cámbialo solo si el robot choca siempre hacia el mismo lado).

### Verificar lectura de QR
```bash
export ROS_DOMAIN_ID=67
ros2 service call /oakd/start_camera std_srvs/srv/Trigger
python3 ~/diag_qr.py
```
Acerca un QR impreso. Debe aparecer `[OK] QR='...'` cuando lo detecte.

---

## Diales (ajuste fino en pista)

Edita al principio de `autonomia_v14.py` antes de subir al robot:

| Variable | Valor actual | Qué hace |
|---|---|---|
| `STAGE` | 1 | Pon 1, 2 o 3 según el stage (solo etiqueta el log del QR) |
| `LIN_MAX` | 0.26 | Velocidad máxima (m/s). Sube para ir más rápido, baja si hay muchos rozamientos |
| `D_BLOCK` | 0.45 | Distancia (m) a la que frena y entra a modo ROTATE. No bajar de 0.38 |
| `D_FREE` | 0.65 | Distancia (m) a la que vuelve a DRIVE. Debe ser > `D_BLOCK` |
| `KP_HEADING` | 1.1 | Agresividad del giro. Sube si el robot se va curvo; baja si zigzaguea |
| `KD_HEADING` | 0.40 | Amortiguación del giro. Sube si hay sobreoscilación |
| `FRONT_DEG` | -90.0 | Offset del LiDAR en grados. Calibrado con `diag_lidar.py` |
| `PROBE_AFTER` | 3.0 | Segundos atascado en ROTATE antes del intento de hueco justo (PROBE) |

---

## Troubleshooting

| Síntoma | Causa probable | Solución |
|---|---|---|
| `control_teclas.py` no conecta / no manda | IP incorrecta | Edita `ROBOT_IP` en línea 7 de `control_teclas.py` |
| Robot no se mueve al mandar `g` | `ROS_DOMAIN_ID` distinto, o robot aún en dock | Verifica `export ROS_DOMAIN_ID=67` en el robot; desacopla del dock |
| `tail -f ~/auto.log` no muestra nada | La autonomía no arrancó | Revisa `cat ~/auto.log` para ver el error de Python |
| Robot se va siempre a un lado | `FRONT_DEG` mal calibrado | Usa `diag_lidar.py` para recalibrar |
| No lee QR / `checkpoints_log.txt` vacío | Cámara apagada | Ejecuta `ros2 service call /oakd/start_camera std_srvs/srv/Trigger` antes de lanzar la autonomía |
| Robot toca la pared suavemente y sigue | Normal en PROBE (mode de escape) | Es comportamiento esperado; si choca fuerte, baja `LIN_MAX` |
| `python control_teclas.py` falla en Linux/Mac | `msvcrt` solo existe en Windows | Corre `control_teclas.py` únicamente en Windows |
