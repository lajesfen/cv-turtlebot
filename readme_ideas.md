# Ideas de visión por computador para CV TurtleBot

## Recomendación principal

La solución más adecuada para este proyecto no consiste en utilizar un único
modelo de deep learning, sino una arquitectura híbrida:

- YOLO para detectar señales de tránsito.
- OpenCV para detectar y decodificar los códigos QR.
- LiDAR y Nav2 para navegación y seguridad.
- Odometría o IMU para controlar los giros.
- Una máquina de estados o un Behavior Tree para tomar decisiones.

Esta combinación aprovecha el tipo de información que cada sensor proporciona
mejor. La cámara reconoce elementos semánticos, mientras que el LiDAR mide
distancias físicas de forma más fiable para prevenir colisiones.

## Análisis del proyecto actual

El repositorio ya contiene una base para la solución:

- `train_signs.py` entrena un detector YOLO para las clases `turn_left`,
  `turn_right` y `stop`.
- `turtlebot_autonomous.py` ejecuta el modelo YOLO sobre las imágenes recibidas.
- `turtlebot.py` utiliza `cv2.QRCodeDetector` para leer códigos QR.
- `turtlebot_autonomous.py` implementa un controlador proporcional básico para
  intentar mantener el robot centrado entre dos paredes usando el LiDAR.

Sin embargo, el modo autónomo todavía no satisface completamente los requisitos
del reto descrito en `Computer_vision_proyecto_final.pdf`:

1. No detecta ni registra los tres códigos QR de los checkpoints.
2. Sólo utiliza dos mediciones individuales del LiDAR para navegar.
3. No comprueba si existen obstáculos delante del robot.
4. Los giros de 90 grados se calculan por tiempo y no mediante odometría.
5. Una misma señal puede producir comandos repetidos en fotogramas consecutivos.
6. No se guarda información sobre las zonas o rutas visitadas.
7. Si dejan de llegar datos del LiDAR, el robot puede continuar avanzando.
8. El programa autónomo falla al iniciar si no encuentra el archivo `best.pt`.
9. El repositorio sólo contiene el software de la computadora; falta el nodo o
   servidor que debe ejecutarse en el TurtleBot para comunicar ROS 2 con UDP.

## Arquitectura propuesta

```text
Cámara ──> Detector QR con OpenCV ──┐
       └─> Detector de señales YOLO ┤
                                    ├─> Máquina de estados / Behavior Tree
LiDAR ──> Obstáculos y costmap ─────┤                  │
Odometría ─> posición y orientación ┘                  ▼
                                                    Nav2
                                                      │
                                           Monitor de colisión
                                                      │
                                                  TurtleBot
```

La percepción no debería enviar velocidades directamente. Cada detector debe
producir eventos, por ejemplo `SIGN_LEFT`, `CHECKPOINT_2` u
`OBSTACLE_FRONT`. Una capa de decisión procesa esos eventos y el controlador de
navegación genera finalmente las velocidades del robot.

## Comparación de modelos y algoritmos

| Problema | Opción recomendada | Alternativa |
|---|---|---|
| Señales de tránsito | YOLO26n ajustado al dataset | YOLO11n o MobileNet-SSD |
| Códigos QR | `cv2.QRCodeDetector` | `QRCodeDetectorAruco` |
| Obstáculos | LiDAR y costmap de Nav2 | Segmentación de imagen |
| Navegación desconocida | SLAM Toolbox y Nav2 | Navegación reactiva con LiDAR |
| Control de giro | Odometría o IMU | Giro temporizado |
| Decisiones | Behavior Tree | Máquina de estados finitos |
| Superficie transitable | Fast-SCNN o BiSeNetV2 | HSV y contornos |
| Señales con visión clásica | HSV, HOG y SVM | ORB o template matching |

## 1. Detección de señales con YOLO26n

Mantendría como primera opción el modelo configurado actualmente:

```python
model = YOLO("yolo26n.pt")
```

Un detector de la familia YOLO es apropiado porque debe encontrar la señal y
clasificarla dentro de una imagen completa. La variante `nano` favorece la
velocidad de inferencia y puede exportarse posteriormente a formatos como ONNX,
OpenVINO o CoreML.

El dataset debería incluir:

- Entre 200 y 500 imágenes por clase como punto de partida.
- Imágenes sin señales como ejemplos negativos.
- Diferentes distancias, tamaños, inclinaciones y posiciones.
- Cambios de iluminación, sombras y desenfoque por movimiento.
- Señales parcialmente ocultas.
- Imágenes capturadas desde la cámara real del TurtleBot.
- Escenarios parecidos al laboratorio de la competencia.

No se deben colocar fotogramas casi idénticos de una misma grabación tanto en
entrenamiento como en validación. Es preferible separar el dataset por sesiones
de captura para medir mejor la capacidad de generalización.

Las métricas principales serían:

- Precisión y recall por clase.
- `mAP50` y `mAP50-95`.
- Matriz de confusión.
- Latencia de inferencia y FPS en el equipo de ejecución.
- Porcentaje de señales que producen la acción correcta durante una prueba real.

### Confirmación temporal

Una señal no debería ejecutarse inmediatamente después de una sola detección.
Se puede exigir que aparezca durante, por ejemplo, tres de los últimos cinco
fotogramas. Después de ejecutar la acción debe existir un periodo de cooldown o
debe verificarse que la señal desapareció antes de aceptar el mismo comando.

Esto reduce acciones incorrectas producidas por falsos positivos y evita girar
varias veces frente a la misma señal.

## 2. Detección de checkpoints QR con OpenCV

No es necesario entrenar una red neuronal para reconocer los checkpoints. Los
QR ya están diseñados para ser localizados y decodificados mediante algoritmos
clásicos.

Se puede usar:

```python
detector = cv2.QRCodeDetector()
retval, decoded_info, points, straight_qrcode = detector.detectAndDecodeMulti(frame)
```

El programa debe mantener un conjunto de checkpoints registrados:

```python
visited_checkpoints = set()

if qr_value not in visited_checkpoints:
    visited_checkpoints.add(qr_value)
    register_checkpoint(qr_value)
```

De esta forma, cada checkpoint se cuenta una única vez, como exige el proyecto.
También conviene guardar el instante, la posición estimada y una imagen de
evidencia de cada detección.

## 3. Detección de obstáculos mediante LiDAR

No usaría deep learning como mecanismo principal para evitar obstáculos. El
LiDAR proporciona directamente distancias métricas y normalmente será más
fiable que inferir profundidad desde una imagen RGB.

En lugar de consultar únicamente los índices 90 y 270, se deben procesar
sectores completos:

- Sector frontal: aproximadamente de -20 a +20 grados.
- Sector frontal izquierdo.
- Sector frontal derecho.
- Sector lateral izquierdo.
- Sector lateral derecho.
- Sector trasero para maniobras de recuperación.

Para cada sector se puede calcular la mediana o un percentil bajo de las
distancias válidas. Esto es más robusto que utilizar el mínimo absoluto, que
puede verse afectado por una medición aislada.

Una política inicial podría ser:

- Distancia frontal menor que 0.30 m: parada inmediata.
- Distancia entre 0.30 y 0.60 m: reducción de velocidad.
- Distancia segura: navegación normal.

Nav2 ofrece un componente llamado Collision Monitor que permite definir una
zona frontal de reducción de velocidad y otra zona más cercana de parada. Esta
capa de seguridad trabaja por debajo del planificador y puede filtrar los
comandos `cmd_vel` antes de enviarlos al robot.

## 4. SLAM y navegación con Nav2

Como el circuito final será desconocido y se penaliza recorrer repetidamente la
misma ruta, resulta útil construir un mapa 2D durante la ejecución.

SLAM Toolbox puede combinar:

- Escaneos del LiDAR.
- Odometría del TurtleBot.
- Transformaciones de ROS 2.

El sistema puede producir un mapa de ocupación y una estimación de la pose del
robot. Esto permitiría:

- Conocer qué zonas ya fueron visitadas.
- Detectar y evitar ciclos.
- Guardar la posición aproximada de cada checkpoint.
- Planificar alrededor de cajas y obstáculos.
- Recuperarse después de encontrar un camino bloqueado.
- Mostrar el mapa y la trayectoria como resultado experimental.

Nav2 puede encargarse de planificación, seguimiento de trayectorias,
replanificación y comportamientos de recuperación. Para este proyecto se puede
personalizar un Behavior Tree que combine navegación, búsqueda de QR y reacción
ante señales.

## 5. Control de los giros mediante odometría

Los giros actuales dependen de:

```python
ROTATION_90_TIME = (np.pi / 2) / ANG
```

Este método cambia su resultado según la batería, el suelo, el deslizamiento y
la respuesta de la red. La alternativa adecuada es:

1. Leer el yaw inicial desde odometría o IMU.
2. Definir un yaw objetivo de `+90` o `-90` grados.
3. Aplicar velocidad angular proporcional al error.
4. Reducir la velocidad al acercarse al objetivo.
5. Detener el robot al entrar en una tolerancia, por ejemplo de 2 a 4 grados.

Esto produce giros más repetibles y facilita justificar técnicamente el
controlador en el informe.

## 6. Segmentación semántica como mejora opcional

Si el suelo o la zona transitable tiene una apariencia consistente, se puede
entrenar un modelo ligero de segmentación como:

- Fast-SCNN.
- BiSeNetV2.
- DeepLabV3 con backbone MobileNet.

Clases posibles:

```text
floor
wall
box
sign
unknown
```

La máscara resultante permitiría estimar el espacio transitable y complementar
el LiDAR. No debería sustituirlo como mecanismo de seguridad.

Esta alternativa requiere etiquetar polígonos o máscaras por píxel, por lo que
implica bastante más trabajo que anotar bounding boxes. Por esa razón debería
implementarse sólo después de que QR, YOLO, LiDAR y odometría funcionen de forma
estable.

## 7. Alternativa basada en visión clásica

Si las señales tienen colores y formas bien definidos, se puede construir un
baseline sin deep learning:

1. Convertir la imagen de BGR a HSV.
2. Segmentar el color dominante de la señal.
3. Aplicar apertura y cierre morfológico.
4. Extraer contornos.
5. Filtrar por área, relación de aspecto y forma.
6. Corregir perspectiva mediante una homografía.
7. Clasificar con HOG y SVM o mediante comparación de plantillas.

Ventajas:

- Inferencia rápida.
- No requiere un dataset grande.
- Fácil de explicar y depurar.

Desventajas:

- Sensibilidad a cambios de iluminación.
- Menor tolerancia a rotaciones y oclusiones.
- Requiere ajustar umbrales al entorno.
- Generaliza peor que un detector entrenado correctamente.

Esta solución sería especialmente útil como baseline experimental para comparar
su precisión y velocidad con YOLO en el informe final.

## 8. Máquina de estados o Behavior Tree

Una máquina de estados inicial podría contener:

```text
INITIALIZING
SEARCHING
NAVIGATING
APPROACHING_SIGN
TURNING
CHECKPOINT_DETECTED
OBSTACLE_RECOVERY
FINISHED
EMERGENCY_STOP
```

Reglas importantes:

- La seguridad siempre tiene prioridad sobre una señal.
- Un QR se registra sólo después de confirmarlo en varios fotogramas.
- Una señal se ejecuta sólo si su confianza y persistencia son suficientes.
- El robot se detiene si la cámara, el LiDAR o la odometría quedan obsoletos.
- Después de un giro se ignora temporalmente la señal que lo produjo.
- Una ruta sin progreso durante cierto tiempo activa una recuperación.
- La detección de los tres checkpoints produce el estado `FINISHED`.

Para una primera versión, una máquina de estados en Python será más sencilla.
Cuando la navegación se integre con ROS 2 y Nav2, un Behavior Tree permitirá
representar mejor los reintentos, recuperaciones y prioridades.

## Modelos que no priorizaría

### Detección monocular de profundidad

Modelos como Depth Anything pueden estimar profundidad desde una sola imagen,
pero la profundidad normalmente no tiene la misma fiabilidad métrica que el
LiDAR. Sólo los consideraría si el LiDAR no cubre una zona necesaria o como
información complementaria.

### Modelos grandes de detección o Transformers

Detectores grandes o modelos como RT-DETR pueden alcanzar buena precisión, pero
su latencia y consumo no se justifican inicialmente para tres clases simples.
Primero se debería medir si YOLO26n presenta realmente un problema de precisión.

### Aprendizaje por refuerzo de extremo a extremo

Entrenar una política que convierta imágenes directamente en velocidades sería
difícil de validar y requeriría muchas simulaciones o recorridos. También sería
menos explicable y podría reaccionar de forma impredecible en el circuito real.
No es una buena primera opción para un proyecto con tiempo y robots limitados.

## Plan de implementación recomendado

### Fase 1: seguridad y checkpoints

1. Integrar `QRCodeDetector` en `turtlebot_autonomous.py`.
2. Registrar checkpoints únicos.
3. Implementar detección de obstáculos frontales por sectores del LiDAR.
4. Añadir watchdogs para cámara, LiDAR y conexión.
5. Limitar las velocidades lineales y angulares.

### Fase 2: percepción de señales

1. Capturar y etiquetar el dataset real.
2. Entrenar YOLO26n.
3. Medir precisión, recall, mAP y latencia.
4. Añadir confirmación temporal y cooldown.
5. Guardar videos y eventos para analizar errores.

### Fase 3: control y navegación

1. Reemplazar giros temporizados por control de yaw.
2. Integrar ROS 2 y los tópicos reales del TurtleBot.
3. Configurar Nav2 y Collision Monitor.
4. Configurar SLAM Toolbox.
5. Añadir recuperación ante bloqueo y detección de falta de progreso.

### Fase 4: mejoras experimentales

1. Comparar YOLO con el baseline HSV más HOG/SVM.
2. Evaluar segmentación del piso si aporta información útil.
3. Exportar el modelo a ONNX u otro formato optimizado.
4. Ajustar velocidades usando resultados medidos en el circuito.

## Experimentos para el informe

Para justificar las decisiones técnicas se pueden realizar los siguientes
experimentos:

1. Comparar YOLO26n y visión clásica en precisión y FPS.
2. Medir detección de señales a distintas distancias.
3. Medir giros temporizados frente a giros por odometría.
4. Comparar el uso de dos rayos del LiDAR con sectores completos.
5. Evaluar la tasa de colisiones con y sin Collision Monitor.
6. Medir el porcentaje de checkpoints detectados una sola vez correctamente.
7. Probar iluminación clara, oscura y con sombras.
8. Probar obstáculos en posiciones que no aparecieron durante el desarrollo.

Conviene registrar para cada intento:

- Tiempo total.
- Checkpoints alcanzados.
- Penalizaciones y colisiones.
- Distancia recorrida.
- Número de falsos positivos de señales.
- FPS de percepción.
- Latencia entre percepción y movimiento.
- Número de recuperaciones y rutas repetidas.

## Conclusión

La propuesta final recomendada es:

**YOLO26n + QRCodeDetector + LiDAR + SLAM Toolbox + Nav2 + Collision Monitor +
odometría**, coordinados mediante una máquina de estados o Behavior Tree.

Esta arquitectura es más robusta que depender únicamente de visión o de un
modelo de deep learning. También permite explicar claramente en la presentación
qué componente resuelve cada problema, medir su contribución por separado y
relacionar las decisiones técnicas con las reglas y penalizaciones de la
competencia.

## Referencias

- [Detección de objetos con Ultralytics](https://docs.ultralytics.com/tasks/detect)
- [QRCodeDetectorAruco de OpenCV](https://docs.opencv.org/4.x/d3/db0/classcv_1_1QRCodeDetectorAruco.html)
- [Behavior Trees de Nav2](https://docs.nav2.org/behavior_trees/index.html)
- [Collision Monitor de Nav2](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)
- [SLAM Toolbox para ROS 2](https://docs.ros.org/en/ros2_packages/humble/api/slam_toolbox/index.html)

## Análisis de las imágenes de `signals/`

La carpeta contiene cuatro fotografías de las señales físicas que se utilizarán
en el circuito:

| Archivo | Contenido observado | Clase o tratamiento recomendado |
|---|---|---|
| `signals/1.jpeg` | Flecha de giro hacia la derecha dentro de un círculo | `turn_right` |
| `signals/2.jpeg` | Flecha de giro hacia la izquierda dentro de un círculo | `turn_left` |
| `signals/3.jpeg` | Código QR impreso | Procesarlo con OpenCV, no con YOLO |
| `signals/4.jpeg` | Círculo con una barra diagonal | `stop`, sólo si esa es la acción definida por los organizadores |

### Características favorables

- Las señales tienen alto contraste entre negro y blanco.
- Los símbolos ocupan una parte grande de la superficie impresa.
- Las señales izquierda y derecha tienen formas claramente diferenciables.
- El QR conserva sus tres patrones cuadrados de posicionamiento.
- La simplicidad de las formas permite construir tanto una solución YOLO como
  un baseline de visión clásica.

### Riesgos observados

#### Cantidad insuficiente de datos

Actualmente hay una sola fotografía de cada símbolo. Estas imágenes sirven como
referencia visual o como base para generar ejemplos sintéticos, pero no forman
un dataset suficiente para entrenar y validar un detector.

Entrenar directamente con estas fotografías produciría sobreajuste: el modelo
podría memorizar la textura del papel, el cartón, la iluminación y el encuadre,
en lugar de aprender la forma de las señales.

#### Fotografías demasiado cercanas

Las señales ocupan prácticamente toda la imagen. Durante la competencia
aparecerán mucho más pequeñas y rodeadas por paredes, cajas, personas u otros
objetos. El dataset debe representar principalmente la vista real desde el
robot y no sólo primeros planos.

#### Poca variedad de perspectiva

Las cuatro imágenes son casi frontales. Se necesitan ejemplos vistos desde la
izquierda, derecha, arriba y abajo, además de diferentes distancias. También se
debe incluir deformación de perspectiva en el aumento de datos.

#### Textura y defectos del material

Se observan dobleces, desgaste, cinta, bordes de cartón y variaciones en la
impresión. Estos elementos pueden cambiar entre la fotografía de referencia y
el día de la competencia. El modelo no debe depender de ellos.

#### Ambigüedad de la cuarta señal

`signals/4.jpeg` no contiene la palabra `STOP`; visualmente representa una
prohibición mediante una barra diagonal. Antes de etiquetarla como `stop` se
debe confirmar que los organizadores definieron explícitamente esa acción. Si
su significado real es distinto, se debe cambiar `CLASS_NAMES` y el mapeo de
acciones para evitar una interpretación incorrecta.

#### Los QR no deben mezclarse con las clases YOLO

El QR de `signals/3.jpeg` no debería añadirse como una cuarta clase del detector
de señales. `QRCodeDetector` puede localizarlo y decodificar su contenido, lo
que permite distinguir checkpoints diferentes sin entrenar una clase por cada
código.

También hacen falta las imágenes o valores de los tres QR definitivos. Con una
sola muestra no se puede verificar que el sistema registre correctamente los
tres checkpoints únicos.

### Estrategia recomendada para crear el dataset

#### 1. Captura con la cámara del TurtleBot

Colocar las señales físicas en diferentes lugares del laboratorio y grabar
recorridos desde la cámara que se utilizará en la competencia. Las capturas
deben incluir:

- Distancias cortas, medias y largas.
- Señales centradas y en los extremos de la imagen.
- Ángulos frontales y laterales.
- Iluminación intensa, tenue y con sombras.
- Desenfoque producido por el movimiento del robot.
- Oclusiones parciales por cajas u obstáculos.
- Fondos y alturas de montaje diferentes.

#### 2. Extracción controlada de fotogramas

No conviene extraer todos los fotogramas consecutivos de un video porque serían
casi idénticos. Se puede seleccionar uno cada cierto intervalo o cuando exista
un cambio apreciable de distancia o perspectiva.

Como primera meta se recomienda obtener al menos 200 imágenes variadas por
clase de señal. La diversidad de escenas es más importante que acumular miles
de fotogramas repetidos.

#### 3. Imágenes negativas

Se deben incluir imágenes del circuito que no contengan ninguna señal:

- Círculos, ruedas y objetos curvos.
- Flechas o símbolos presentes en carteles ajenos.
- Patrones cuadrados parecidos a QR.
- Cajas, patas de mesas y bordes de paredes.
- Escenas oscuras y fotogramas con desenfoque.

Estas imágenes se añaden sin bounding boxes y ayudan a reducir falsos
positivos.

#### 4. División correcta del dataset

La separación debe realizarse por sesión o recorrido:

- 70 % para entrenamiento.
- 15 % para validación.
- 15 % para prueba final.

Los fotogramas de un mismo video no deberían repartirse entre los tres grupos,
porque la similitud entre imágenes produciría métricas artificialmente altas.

#### 5. Aumentos de datos

La configuración actual acierta al usar `fliplr=0.0`: voltear horizontalmente
una flecha derecha la convertiría en una señal izquierda con una etiqueta
incorrecta.

Son adecuados los cambios moderados de brillo, contraste, escala, traslación y
rotación. Se puede añadir perspectiva, desenfoque y ruido JPEG, pero sin
deformar tanto el símbolo que deje de representar la señal real.

#### 6. Datos sintéticos

Los símbolos de las fotografías pueden recortarse y proyectarse sobre fondos
del laboratorio con distintas escalas y perspectivas. Estos ejemplos ayudan a
iniciar el entrenamiento, pero deben combinarse con capturas reales desde el
TurtleBot y no reemplazarlas.

### Alternativa de visión clásica favorecida por estas señales

Debido a que todas las señales son monocromáticas y de alto contraste, se puede
crear un baseline eficiente:

1. Convertir la imagen a escala de grises.
2. Aplicar umbral adaptativo u Otsu.
3. Buscar contornos circulares o cuadriláteros.
4. Rectificar la región mediante homografía.
5. Comparar la forma normalizada contra las plantillas izquierda, derecha y
   parada.

Este método podría funcionar bien en condiciones controladas, pero YOLO seguirá
siendo más robusto frente a fondos complejos, oclusiones y cambios importantes
de iluminación. Conviene implementar el método clásico como comparación para el
informe y mantener YOLO como detector principal.

### Recomendaciones específicas para el QR

- Obtener los tres QR definitivos y registrar el texto esperado de cada uno.
- Probar `detectAndDecodeMulti` si más de un código puede aparecer en una toma.
- Mantener suficiente borde blanco alrededor de cada código.
- Medir la máxima distancia desde la que puede decodificarse con la cámara real.
- Confirmar el mismo valor en varios fotogramas antes de registrar un checkpoint.
- Guardar cada valor en `visited_checkpoints` para contarlo una sola vez.
- Reducir temporalmente la velocidad si el QR se detecta pero todavía no se
  puede decodificar.

## Resumen final: cosas que faltan y cosas que debemos mejorar

### Bloqueantes para tener una demostración completa

- Implementar o incorporar el nodo ROS 2 del TurtleBot que publica cámara,
  LiDAR y odometría y recibe comandos de velocidad.
- Obtener acceso a los tres QR definitivos y confirmar qué acción representa
  exactamente `signals/4.jpeg`.
- Capturar y etiquetar un dataset real; las cuatro imágenes de `signals/` no son
  suficientes para entrenar.
- Entrenar el detector, guardar `best.pt` y corregir la ruta común entre
  `train_signs.py` y `turtlebot_autonomous.py`.
- Integrar la lectura y el registro de checkpoints QR en el modo autónomo.

### Mejoras críticas de seguridad

- Detectar obstáculos usando sectores frontales completos del LiDAR.
- Detener el robot si dejan de llegar cámara, LiDAR, odometría o comandos.
- Limitar y suavizar las velocidades lineales y angulares.
- Añadir una parada de emergencia y un Collision Monitor.
- Probar inicialmente con velocidades bajas y supervisión física.

### Mejoras de percepción

- Confirmar señales durante varios fotogramas y añadir un cooldown después de
  ejecutar una acción.
- Evaluar por separado precisión, recall, mAP, falsos positivos y latencia.
- Añadir capturas lejanas, laterales, oscuras, desenfocadas y parcialmente
  ocultas.
- Incorporar imágenes negativas del laboratorio.
- Separar entrenamiento, validación y prueba por recorridos completos.
- Medir la distancia mínima y máxima de reconocimiento para señales y QR.

### Mejoras de navegación y control

- Reemplazar los giros temporizados por control cerrado con odometría o IMU.
- Usar varios sectores del LiDAR en lugar de dos índices fijos.
- Integrar Nav2 para planificación, replanificación y recuperación.
- Usar SLAM Toolbox para mapear el circuito y evitar recorridos repetidos.
- Detectar falta de progreso y ejecutar una maniobra segura de recuperación.
- Coordinar prioridades mediante una máquina de estados o Behavior Tree.

### Mejoras de ingeniería y evaluación

- Crear un archivo reproducible de dependencias.
- Mover IP, puertos, velocidades, rutas y umbrales a un archivo de configuración.
- Añadir registros con timestamps y guardar videos de las pruebas.
- Crear pruebas unitarias para parsing, estados, cooldowns y registro de QR.
- Probar pérdida de paquetes, desconexión, datos inválidos y ausencia del modelo.
- Comparar YOLO con el baseline clásico de umbralización y plantillas.
- Documentar resultados, limitaciones y decisiones usando métricas reales.

### Orden recomendado de trabajo

1. Confirmar las cuatro acciones y obtener los tres QR.
2. Completar la comunicación ROS 2 con el robot.
3. Implementar parada frontal, watchdogs y límites de velocidad.
4. Integrar QR y registro único de checkpoints.
5. Capturar, etiquetar y dividir correctamente el dataset.
6. Entrenar YOLO y añadir confirmación temporal.
7. Implementar giros por odometría.
8. Integrar Nav2, Collision Monitor y SLAM.
9. Realizar pruebas completas y producir las métricas del informe.
