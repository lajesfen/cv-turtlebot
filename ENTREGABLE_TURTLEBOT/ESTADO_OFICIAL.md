# ESTADO OFICIAL — sincronización entre cuentas de Claude (TurtleBot4, Autonomous Time Attack)

> Documento vivo. Lo leen ambas cuentas de Claude para continuar sin perder contexto. Última
> actualización: **v17.6 (YOLO REENTRENADO mAP50 0.987 · meta · velocidad recta · ventana+teclas en 1 solo .py)**. **PRIORIDAD ABSOLUTA: NO CHOCAR** (choque = penalidad fuerte; 6 = intento perdido).
> El robot **nunca debió chocar**: cualquier roce es un bug a corregir, no algo a "recuperar y ya".

---

## 1. Estado actual

- **Versión de competencia:** `v17_completa/autonomia_v17.py` (corre a bordo en la Pi).
- **Versión de nav pura / fallback:** `v14_joyita/autonomia_v14.py`.
- **Robot:** TurtleBot4 (Create 3 + RPi4 + RPLIDAR + OAK-D). `ROS_DOMAIN_ID=67`. IP DHCP (~192.168.0.104).
- **`USE_STAMPED = False` — RECONCILIADO y CONFIRMADO.** Felipe corrió v17 con False y **sí mueve** (`Twist` en
  `/cmd_vel_unstamped`, lo escucha `create3_repub`). Código y este doc coinciden en False.
- **YOLO — REENTRENADO (yolo26n, mAP50 0.987 / mAP50-95 0.876).** Izq/der **RESUELTO** (turn_right P=1.0,
  turn_left P≈0.95, stop 0.98, meta 1.0). Latencia y comms diagnosticadas **SANAS** (UDP 0% pérdida ~5 ms;
  e2e cámara→decisión 60-90 ms) → el problema NUNCA fue la red, era el modelo (ya arreglado). Riesgo residual
  menor: algún falso positivo en fondo → mitigado por conf 0.5-0.6 + votación + más negativos. Bumper **NO reporta** (por eso `_check_stall`).
- **Nav probada en pista (parcial):** esquiva obstáculos; ojo con obstáculos que el LiDAR 2D no ve (tela/pie
  bajo el plano ~18-20 cm) — en competencia son cajas rígidas altas, sí las verá.

---

## 2. Arquitectura (resumen)

- **A bordo (Pi):** `autonomia_v17.py` = nav por LiDAR (DRIVE follow-gap Kalman+PD / EXPLORE que se mueve /
  PROBE / EVADE) + memoria anti-loop (rejilla odom) + registro QR + recepción de señales YOLO por UDP 5008.
- **Stream de cámara:** `ver_y_capturar.py` (robot) sirve MJPEG en `http://<IP>:8000`.
- **Laptop (Windows, SIN ROS): UN solo `.py`** = `yolo_win.py --show`: lee el MJPEG, corre `best.pt`, muestra
  ventana con **cajas+clase+conf**, manda `LEFT/RIGHT/SSTOP/META` por UDP 5008, y **captura las teclas desde la
  ventana** (g=ARM, p=PAUSA, q=salir). `control_teclas.py` queda solo de respaldo (headless). Para que abra la
  ventana: `pip uninstall opencv-python-headless -y`. PyTorch usa los 16 núcleos solo (no hay que paralelizar a mano).
- **Señales (subsunción):** el LiDAR SIEMPRE evita chocar; el YOLO solo AÑADE decisiones de giro. Un falso
  positivo NO debe causar choque. Las señales van a un **buffer** (última señal, caduca por tiempo Y distancia)
  y se consultan al decidir girar; se obedecen **solo si ese lado tiene hueco transitable**.

---

## 3. Config actual y diales (valores v17.6, FINE-TUNE en pista)

| Dial | Valor | Qué hace / dirección |
|---|---|---|
| `USE_STAMPED` | **False** ✅ | RECONCILIADO. Felipe corrió v17 con False y SÍ mueve (`Twist` en `/cmd_vel_unstamped`). |
| `LIN_MAX` | **0.30** | Techo de velocidad (SOLO DRIVE; antes 0.26). Bajar a 0.28 si roza en curvas rápidas. |
| `W_ALIGN` | **1.2** | Tolerancia al giro antes de frenar (antes 0.9). Más alto = mantiene velocidad en recta angosta. |
| `META_HOLD_S` | 10.0 | Meta: parar este tiempo, luego girar 180°. PROVISIONAL → confirmar con Rensso. |
| `EXPLORE_V` | 0.08 | Velocidad al rodear esquina. Bajar (0.05) si clipea al entrar. |
| `D_BLOCK` | 0.45 | Frente < esto → EXPLORE. Subir (0.55) = rodea antes, más espacio. |
| `ROBOT_PASS` | **0.43** | Ancho mín. de hueco que intenta. Subir si roza flancos; bajar si rechaza válidos. |
| `ROBOT_PASS_MIN` | 0.376 | Umbral relajado en PROBE (~1.5 cm/lado). Último recurso; no bajar. |
| `SIDE_AVOID` | **0.38** | Repele del flanco a esta distancia. Subir = se aleja más de las paredes. |
| `K_SIDE` | **2.0** | Fuerza de repulsión lateral. Muy alto = zigzag. |
| `KD_HEADING` | **0.55** | Amortiguación del giro (menos zigzag en recto). |
| `STICK_W` | **0.8** | Pegajosidad al rumbo previo (menos zigzag). |
| `EVADE_T` | **0.8** | Duración de la recuperación tras golpe/atasco. |
| `STALL_T` | 0.8 | Seg comandando movimiento sin trasladar NI rotar → declara atasco (EVADE). |
| `W_VISIT` | 0.60 | Peso anti-loop de la memoria. `USE_MEMORY=False` la apaga. |
| `FRONT_DEG` | -90.0 | Offset del LiDAR (calibrado). NO cambiar sin `diag_lidar`. |

---

## 4. Problemas conocidos + estado

1. **CHOQUE con el lateral superior + reincide (el más grave, en pista de laboratorio).**
   Causa raíz: el LiDAR es un **corte 2D a una altura**; el robot es 3D. Golpea con la parte alta algo que el
   LiDAR (más abajo) ve "libre" → re-planea optimista y reincide. Fix con LiDAR 2D = inflar el cuerpo efectivo
   (`SIDE_AVOID/K_SIDE/ROBOT_PASS`) + recuperación que gire a un hueco NUEVO. **Nota:** con tela/pie bajo el plano
   el sensor NO lo ve; en competencia son **cajas rígidas altas** (buen retorno LiDAR) → se ven de lejos. Fine-tune con cajas reales.
2. ✅ **RESUELTO — YOLO izq/der.** Reentrenado (yolo26n; dominio real + `fliplr=0` + balance + negativos).
   Matriz: turn_right P=1.0, turn_left P≈0.95, stop 0.98, meta 1.0. mAP50 0.987. Residual: algún FP en fondo → conf 0.5-0.6 + más `junk`.
3. ✅ **RESUELTO — latencia/comms.** UDP 0% pérdida, RTT ~5 ms; e2e 60-90 ms (se arregló el buffer de
   `VideoCapture` con hilo lector). NO era comms; era el modelo (ya reentrenado).
4. **Bumper NO reporta** golpes → se añadió detección de atasco por ODOM (`_check_stall`); ambos disparan EVADE.
5. **Batería:** a 15% el Create 3 va flojo. Correr con ≥30%. Docked = no maneja.

---

## 5. PENDIENTES priorizados (para ambas cuentas)

1. **[CRÍTICO] Que NO choque.** Fine-tune `SIDE_AVOID/K_SIDE/ROBOT_PASS/D_BLOCK/EXPLORE_V` con cajas reales.
   Verificar que `_check_stall` dispara (log `[EVADE] ATASCO`).
2. ✅ **[YOLO] Delay/pérdida — HECHO.** Red sana (0% pérdida, RTT ~5 ms), e2e 60-90 ms. Herramientas en `yolo_win.py`: `--ping N` y la línea `--- DIAG`.
3. ✅ **[YOLO] Reentrenado — HECHO.** yolo26n, mAP50 0.987. Notebook `entrenar_yolo26n.ipynb` (Colab: baja de Drive con `gdown`, **fuerza el class id por carpeta**, ingiere el set de Oscar **sin remapear**, negativos `junk` con label vacío). Dataset: ~1000 propias + 108 de Oscar + junk.
4. ✅ **[meta] Lógica HECHA (provisional).** Al recibir `META`: para `META_HOLD_S`(10 s) y gira 180° (estado `METASTOP`, no repite por `meta_done`). ⚠️ **CONFIRMAR con Rensso**: ¿el intento TERMINA en la meta o solo se detiene? (si termina, dejar `return 0,0` permanente en METASTOP). `yolo_win` ya mapea `meta→META`.
5. **[finetune v17 en pista]** con cajas reales: `SIDE_AVOID/K_SIDE/ROBOT_PASS/D_BLOCK/LIN_MAX/W_ALIGN`.
6. **[upgrades — análisis]** YOLO ya casi **saturado** (0.987): ganancia marginal salvo (a) más **NEGATIVOS**
   (matar FP en fondo) y (b) fotos del **RECINTO real** (domain shift). v17 está **maduro**: gana por finetune de
   diales + OPCIONAL escape escalado (si EVADE se repite, girar cada vez más) / modo sprint para Stage 1. Ambos opcionales, probar con caja rígida primero.
7. **[limpieza post-competencia]** El estado `TURN` (giro por /odom) ahora solo lo usa META (180°); `SIGN_W` sin uso. Revisar DESPUÉS de competir.

---

## 6. Reglas de oro
- **NO CHOCAR > todo.** Confiabilidad > velocidad. Nunca sacrificar seguridad por tiempo.
- `USE_STAMPED=False`, `ROS_DOMAIN_ID=67`, robot fuera del dock y batería ≥30%.
- Correr comandos **una línea a la vez**. Verificar `pgrep -af autonomia_v17`.
- Laptop: **1 solo** `.py` (`yolo_win.py --show`), teclas desde la ventana. `control_teclas.py` = respaldo.
- Editar SIEMPRE la copia del `ENTREGABLE_TURTLEBOT/v17_completa/` (es la que se sube/`scp`), no la de la raíz.

---

## 7. Log de cambios recientes

- **v17.6 (sesión de diagnóstico+YOLO):** **YOLO reentrenado** (yolo26n, mAP50 0.987, izq/der resuelto) con notebook
  `entrenar_yolo26n.ipynb`. **Diagnóstico cerrado**: red y latencia sanas, el fallo era el modelo. **META** implementada
  (`METASTOP`: para 10 s + gira 180°, ⚠️ confirmar regla con Rensso; `yolo_win` mapea `meta→META`). **Velocidad recta**:
  `LIN_MAX` 0.26→0.30, `W_ALIGN` 0.9→1.2 (solo DRIVE, con valor viejo comentado). **Laptop = 1 solo `.py`**: `yolo_win.py --show`
  muestra cajas+clase+conf y captura teclas (g/p/q) desde la ventana; `control_teclas.py` = respaldo. `capturar_clases.py`
  con **modo AUTO** (captura masiva anti-borroso/anti-duplicado), numeración secuencial y carpeta `mixed`. `USE_STAMPED=False` confirmado (mueve).
- **v17.5 (FUSIÓN):** `_check_stall` (atasco por odom → EVADE) + diales anti-choque (ROBOT_PASS 0.43, SIDE_AVOID 0.38,
  K_SIDE 2.0, KD_HEADING 0.55, STICK_W 0.8, EVADE_T 0.8) sobre el v17 con memoria+diagnóstico. Entregable curado
  (solo `v17_completa/` + `v14_joyita/`). `yolo_win.py` con votación + #seq/ACK + --ping + --test.
- **v17.1/17.2:** memoria anti-loop; señal expira por tiempo Y distancia; PROBE rápido; borra memoria por checkpoint;
  `_enter_rotate` obedece la señal solo si ese lado tiene hueco.

---

## 8. Herramientas de diagnóstico y dataset (v17.6)

- **`yolo_win.py`** (laptop, 1 solo .py): hilo lector (mata la latencia del buffer MJPEG), **votación por ventana**
  (`--vote-min/--vote-window`), protocolo `#seq`+ACK (RTT/pérdida reales), `--ping N` (sonda de red), `--test`
  (log numerado `[N] clase`, no manda), `--show` (ventana con cajas+clase+conf **+ teclas g/p/q**). `--conf`/`--min-area`/`--cooldown` para filtrar.
- **`autonomia_v17.py`** (robot): `_cmd_listen` responde `ACK#seq`/`PONG#seq` y lleva `cmd_count`. Comandos: ARM/PAUSE/
  LEFT/RIGHT/SSTOP/**META**. Estados: DRIVE/EXPLORE/PROBE/EVADE/SIGNSTOP/**METASTOP**/TURN.
- **`ver_y_capturar.py`** (robot): sirve el stream + mide nitidez/FPS/edad + ráfaga.
- **`capturar_clases.py`** (robot): botones de clase (turn_left/turn_right/stop/meta/**mixed**), **modo AUTO** (guarda
  cada ~0.4 s, descarta borrosas y casi-duplicadas), nombre secuencial `{clase}_{NNNN}.jpg`, nitidez en vivo.
- **`entrenar_yolo26n.ipynb`** (Colab, 1 T4): baja de Drive con `gdown`, fuerza class id por carpeta, ingiere Oscar
  sin remapear, negativos `junk`, split 80/20, entrena con `fliplr=0`, matriz de confusión, baja `best.pt`.
- **Modelo:** yolo26n, 4 clases (turn_left/turn_right/stop/meta). Reemplazar `best.pt` en `v17_completa/` tras reentrenar.
