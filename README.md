# CV TurtleBot

Proyecto experimental para controlar un TurtleBot 4 desde una computadora y
procesar la cámara y el LiDAR. Incluye dos clientes de control por UDP y un
script para entrenar un detector de señales con Ultralytics YOLO.

> [!IMPORTANT]
> Este repositorio contiene únicamente el software del lado de la computadora.
> Falta el nodo o servidor que debe ejecutarse en el TurtleBot para publicar la
> telemetría y recibir comandos en los puertos UDP `6000` y `5007`. Por ese
> motivo, el proyecto todavía no funciona de extremo a extremo por sí solo.

## Estructura del proyecto

```text
cv-turtlebot/
├── README.md
├── .gitignore
├── turtlebot.py
├── turtlebot_autonomous.py
├── train_signs.py
└── sample_images/
    └── output_*.png
```

### `README.md`

Es este documento. Describe el contenido del repositorio, cómo se relacionan
sus componentes y qué hace falta antes de conectarlo a un robot físico.

### `.gitignore`

Evita que Git agregue archivos locales que no deben formar parte del proyecto:

- `.DS_Store`: metadatos creados por macOS.
- `.venv`: entorno virtual local de Python.

Actualmente no excluye datasets, resultados de entrenamiento ni pesos de
modelos. Conviene agregarlos si son grandes y no se almacenarán en Git.

### `turtlebot.py`

Cliente manual basado en Python, OpenCV y sockets UDP. No utiliza directamente
la API de ROS 2.

Sus responsabilidades son:

1. Contactar al servidor del robot en `ROBOT_IP:6000`.
2. Enviar el mensaje de emparejamiento:

   ```text
   HELLO <domain_id> <pairing_code>
   ```

3. Esperar una respuesta con este formato:

   ```text
   ACK <domain_id> <robot_name>
   ```

4. Recibir mediciones del LiDAR como mensajes de texto `SCAN`.
5. Recibir imágenes JPEG codificadas en Base64 como mensajes `IMG`.
6. Detectar códigos QR con `cv2.QRCodeDetector`.
7. Enviar velocidades lineales y angulares al puerto UDP `5007`.

Los códigos QR reconocidos son:

- `TURN_LEFT`: gira aproximadamente 90 grados a la izquierda.
- `TURN_RIGHT`: gira aproximadamente 90 grados a la derecha.

El giro se calcula por tiempo, no con odometría. Por ello, el ángulo real puede
variar según la batería, el piso y la respuesta del robot.

Configuración que debe revisarse antes de ejecutarlo:

```python
ROBOT_IP = "192.168.0.101"
ROBOT_PORT = 6000
CONTROL_PORT = 5007
DESIRED_DOMAIN_ID = 67
PAIRING_CODE = "oscar"
EXPECTED_ROBOT_NAME = "turtlebotoscar"
```

### `turtlebot_autonomous.py`

Cliente de conducción autónoma. Reutiliza el mismo protocolo UDP de
`turtlebot.py`, pero reemplaza los códigos QR por detección de señales con un
modelo YOLO.

El programa ejecuta tres tareas principales:

- Recepción de imágenes y mediciones del LiDAR.
- Detección de `turn_left`, `turn_right` y `stop` con YOLO.
- Movimiento hacia adelante con un controlador proporcional para intentar
  mantener el robot centrado entre dos paredes.

El controlador compara dos posiciones del arreglo del LiDAR:

```python
LIDAR_LEFT_IDX = 90
LIDAR_RIGHT_IDX = 270
```

Estos índices sólo son correctos si la orientación y cantidad de mediciones del
LiDAR coinciden con lo asumido por el programa. Deben validarse con datos reales.

El modelo se carga desde:

```text
signs_model/weights/best.pt
```

Ese archivo no está incluido en el repositorio. Además, antes de usar este modo
en un robot físico deben implementarse como mínimo:

- Detención ante obstáculos frontales.
- Detención si dejan de llegar datos del LiDAR o de la red.
- Límite y validación de velocidades.
- Confirmación de comandos y un mecanismo de parada de emergencia.
- Control de giro mediante odometría en lugar de temporización.

### `train_signs.py`

Entrena un modelo YOLO para reconocer señales de giro y parada.

Las clases configuradas son:

```text
0: turn_left
1: turn_right
2: stop
```

El script espera esta estructura, que todavía no está incluida:

```text
dataset/
├── images/
│   ├── train/
│   └── val/
└── labels/
    ├── train/
    └── val/
```

Cada imagen necesita un archivo `.txt` con anotaciones en formato YOLO:

```text
<class_id> <x_center> <y_center> <width> <height>
```

Todos los valores de posición y tamaño deben estar normalizados entre `0` y
`1`. El entrenamiento usa 60 épocas, imágenes de `416 x 416` y un batch de 16.

El código inicia el entrenamiento con:

```python
YOLO("yolo26n.pt")
```

Los resultados se generan dentro de `signs_model/`. Con la configuración
actual, el mejor peso puede quedar en:

```text
signs_model/weights/weights/best.pt
```

Si esto ocurre, se debe mover el archivo o corregir `YOLO_MODEL_PATH` en
`turtlebot_autonomous.py`.

### `sample_images/`

Contiene doce imágenes PNG de muestra. No son consumidas automáticamente por
ninguno de los scripts actuales; sirven como evidencia visual, pruebas manuales
o material inicial para preparar un dataset.

Para utilizarlas en el entrenamiento es necesario:

1. Separarlas entre `dataset/images/train` y `dataset/images/val`.
2. Dibujar las cajas de cada señal.
3. Crear los archivos de etiquetas correspondientes.

## Flujo de comunicación esperado

```text
TurtleBot 4                              Laptop
┌──────────────────────────────┐          ┌──────────────────────────────┐
│ Servidor UDP no incluido     │          │ turtlebot.py                 │
│                              │  IMG --> │ o                           │
│ ROS 2 -> cámara y LiDAR      │ SCAN --> │ turtlebot_autonomous.py      │
│                              │ <-- v,w │                              │
│ UDP 6000 / UDP 5007          │          │ OpenCV / NumPy / YOLO        │
└──────────────────────────────┘          └──────────────────────────────┘
```

`ROS_DOMAIN_ID` aparece en el protocolo de emparejamiento, pero los clientes no
participan en DDS ni crean nodos ROS 2. El servidor faltante es quien tendría que
suscribirse y publicar en los tópicos reales del TurtleBot.

## Requisitos de Python

Para `turtlebot.py`:

```text
numpy
opencv-python
```

Para `turtlebot_autonomous.py` y `train_signs.py` también se requiere:

```text
ultralytics
PyYAML
```

Instalación local en macOS o Ubuntu:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy opencv-python ultralytics PyYAML
```

## Ejecución

Antes de ejecutar cualquier cliente deben cumplirse estas condiciones:

- La laptop y el TurtleBot están en la misma red.
- `ROBOT_IP` contiene la IP actual de la Raspberry Pi del robot.
- El servidor UDP faltante está instalado y activo en el robot.
- Los puertos UDP `6000` y `5007` no están bloqueados.
- El dominio, código de emparejamiento y nombre coinciden en ambos lados.

Cliente con códigos QR:

```bash
source .venv/bin/activate
python turtlebot.py
```

Entrenamiento:

```bash
source .venv/bin/activate
python train_signs.py
```

Cliente autónomo, sólo después de generar y configurar los pesos:

```bash
source .venv/bin/activate
python turtlebot_autonomous.py
```

## Estado actual

- [x] Cliente de cámara y LiDAR por UDP.
- [x] Lectura de códigos QR.
- [x] Entrenamiento y detección con YOLO.
- [x] Control proporcional básico entre paredes.
- [ ] Servidor ROS 2/UDP para el TurtleBot.
- [ ] Archivo reproducible de dependencias.
- [ ] Configuración mediante variables de entorno o argumentos.
- [ ] Dataset y pesos entrenados.
- [ ] Pruebas automatizadas.
- [ ] Mecanismos de seguridad para el robot físico.

## Advertencia de seguridad

Prueba primero con el robot elevado, las ruedas sin contacto con el suelo y una
persona preparada para cortar la alimentación. El modo autónomo actual es un
prototipo y no debe operar sin supervisión.
