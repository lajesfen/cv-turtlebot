# TurtleBot4 — Autonomous Time Attack (UTEC, Visión por Computador)

Navegación autónoma con LiDAR + lectura de QR (checkpoints) + obediencia de señales (YOLO).
Cada carpeta es **autocontenida** con su `README.md` y comandos copia-pega, para que cualquiera
del grupo corra el robot sin depender de nadie.

## Reglas de oro
1. **`export ROS_DOMAIN_ID=67`** SIEMPRE en el robot (antes de lanzar cualquier script). Si no
   coincide con la laptop, el robot ignora todos los comandos UDP y no responde.
2. **Qué corre dónde:** la navegación (`autonomia_vNN.py`) corre EN EL ROBOT (Pi); el control por
   teclas (`control_teclas.py`) y el YOLO (`yolo_win.py`, solo v17) corren EN LA LAPTOP (Windows).
3. El robot debe estar **desacoplado del dock** antes de mandar `g`.
4. **Una vez armado (`g`), NO toques nada** hasta el final: tocar el robot/entorno o mandar teclas
   a mitad del intento = intervención humana = **intento terminado** (rúbrica).
5. **Confiabilidad > velocidad.** Llegar a los 3 checkpoints da más puntos que ir rápido;
   6 choques = intento perdido.

## Qué versión usar
| Carpeta | Qué es | Cuándo usarla |
|---|---|---|
| `v17_completa/` | **Competencia.** Nav + señales YOLO + memoria anti-loop + QR. Requiere la laptop con `yolo_win.py`. | Stages 2 y 3 (señaléticas). También funciona para Stage 1. |
| `v14_joyita/` | **Fallback.** La mejor navegación pura probada, sin señales, sin laptop. | Stage 1 (sprint) o si el YOLO/laptop falla en competencia. |

> Día de competencia: probar v17 en warm-up; si el YOLO da problemas, caer a v14 (misma nav, sin
> laptop). Ambas registran los QR con timestamp en `~/checkpoints_log.txt` en el robot.

---

## Prerequisitos (UNA VEZ antes de la primera prueba)

### En la laptop (Windows) — solo para v17
```powershell
pip install ultralytics opencv-python
```

### Red WiFi del laboratorio
- **SSID:** `Lab_Computech_5G`
- **Contraseña:** `Computech2025!`

La laptop y el robot deben estar en la **misma red** para que se comuniquen.

### IP del robot en la red del lab
**IP configurada en los archivos: `192.168.0.104`**

Si la IP cambió (ej. alguien reinició el router), encuéntrala con:
```bash
# (en el robot, por SSH o serial)
hostname -I
# anota la primera IP que aparece
```
Luego edita `ROBOT_IP` en las líneas indicadas de:
- `v14_joyita/control_teclas.py` — línea 7
- `v17_completa/control_teclas.py` — línea 7
- el argumento `--robot-ip` en el comando de `yolo_win.py` (v17)

### SSH al robot desde Windows
```powershell
ssh ubuntu@192.168.0.104
# contraseña: turtlebot4
```

---

## Estructura de archivos

```
v14_joyita/
  autonomia_v14.py    ← ROBOT: navegación + log de QR
  control_teclas.py   ← LAPTOP Windows: ARM / PAUSA / STOP
  diag_lidar.py       ← ROBOT: opcional, calibra FRONT_DEG
  diag_qr.py          ← ROBOT: opcional, verifica lectura de QR

v17_completa/
  autonomia_v17.py    ← ROBOT: navegación + señales YOLO + memoria + log de QR
  ver_y_capturar.py   ← ROBOT: sirve el video de la cámara en http://<IP>:8000
  yolo_win.py         ← LAPTOP Windows: corre YOLO y manda señales por UDP
  best.pt             ← LAPTOP Windows: modelo YOLO entrenado (NO subir al robot)
  control_teclas.py   ← LAPTOP Windows: ARM / PAUSA / STOP
  diag_lidar.py       ← ROBOT: opcional
  diag_qr.py          ← ROBOT: opcional
  capturar_frames.py  ← ROBOT: opcional (captura frames para entrenar YOLO)
```

Lee el `README.md` dentro de la carpeta que uses para los comandos completos.
