# 🧠 TurtleBot 4 – Setup Completo (VM + Robot físico)

**Versión de Ubuntu:** [Ubuntu 24.04 LTS (Noble)](https://releases.ubuntu.com/24.04)  
**Máquina virtual:** [VirtualBox](https://www.virtualbox.org)  
**ROS 2:** [Jazzy Jalisco](https://docs.ros.org/en/jazzy/)  
**Robot:** TurtleBot 4 Lite  
**Manual oficial:** [TurtleBot 4 User Manual – Basic Setup](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)

---

## 🧩 Índice
1. Crear la máquina virtual con Ubuntu 24.04
2. Preparar Ubuntu para ROS 2 Jazzy
3. Instalar ROS 2 Jazzy
4. Instalar paquetes del TurtleBot 4
5. Configurar y conectar al TurtleBot 4
6. Verificación de comunicación (talker/listener)
7. Instalación de sensores y bringup
8. Pruebas de movimiento y cámara/LiDAR
9. Notas, tips y solución de errores

---

## 1️⃣ Crear máquina virtual con Ubuntu 24.04

1. Instala **VirtualBox**.
2. Crea una nueva VM:
   - **Tipo:** Linux → Ubuntu (64-bit)
   - **RAM:** mínimo 4 GB (recomendado 8 GB)
   - **Disco:** 20 GB +
   - **ISO:** adjunta Ubuntu 24.04.
3. Instala Ubuntu normalmente.
4. Instala las **Guest Additions** (para mejor resolución y clipboard).
5. Actualiza el sistema:
   ```bash
   sudo apt update
   sudo apt upgrade -y
   sudo reboot
   ```

---

## 2️⃣ Preparar Ubuntu para ROS 2 Jazzy

```bash
# Asegurar entorno UTF-8
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Habilitar repositorios
sudo apt install software-properties-common -y
sudo add-apt-repository universe

# Añadir repositorio de ROS
sudo apt install curl gnupg2 lsb-release -y
sudo mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

---

## 3️⃣ Instalar ROS 2 Jazzy

```bash
sudo apt install ros-jazzy-desktop -y
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verifica que ROS 2 funciona:
```bash
ros2 run demo_nodes_cpp talker
```

---

## 4️⃣ Instalar paquetes específicos del TurtleBot 4

```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot4-desktop -y
```

Verifica que esté instalado:
```bash
ros2 pkg list | grep turtlebot4
```

---

## 5️⃣ Configurar y conectar al TurtleBot 4

1. **Encender el robot**.
2. Conéctate a su red Wi-Fi:
   - **SSID:** `turtlebot4`
   - **Contraseña:** `turtlebot4`
3. Accede por SSH:
   ```bash
   ssh ubuntu@10.42.0.1
   # contraseña: turtlebot4
   ```
4. Ejecuta el asistente de configuración:
   ```bash
   turtlebot4-setup
   ```
   Completa los parámetros de Wi-Fi:
   ```
   SSID: Lab_Computech_5G
   Password: Computech2025!
   ```
5. Luego abre en un navegador:
   ```
   http://<ip-del-turtlebot>:8080
   ```
   En la interfaz web, configura la red del **Create 3**.

6. Reinicia el robot:
   ```bash
   sudo reboot
   ```

---

## 6️⃣ Verificación de comunicación (listener ↔ talker)

### En el **robot**:
```bash
sudo apt update
sudo mkdir -p /usr/share/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-demo-nodes-cpp ros-$ROS_DISTRO-demo-nodes-py
```

### En el **robot (terminal 1)**:
```bash
ros2 run demo_nodes_cpp listener
```

### En la **VM (terminal 2)**:
```bash
ros2 run demo_nodes_cpp talker
```

✅ Si se ven los mensajes transmitidos, la comunicación ROS está funcionando correctamente.  
❌ Si no, revisar el `ROS_DOMAIN_ID` o llamar a **Cortijo** 😎

---

## 7️⃣ Instalación de sensores y bringup

En el **robot**, instala los paquetes esenciales:

```bash
sudo apt install -y   ros-jazzy-rplidar-ros   ros-jazzy-depthai-ros   ros-jazzy-irobot-create-nodes   ros-jazzy-turtlebot4-msgs   ros-jazzy-turtlebot4-description   ros-jazzy-turtlebot4-bringup
```

Apaga y vuelve a encender el TurtleBot.

Luego de reconectarte por SSH:

```bash
ros2 launch turtlebot4_bringup lite.launch.py
```

Deberías ver al ejecutar:
```bash
ros2 topic list
```
Una lista extensa que incluya:
```
/battery_state
/cmd_vel
/odom
/scan
/oakd/rgb/preview/image_raw
/tf
...
```

Si no aparece, llamar a Cortijo.

---

## 8️⃣ Pruebas de movimiento y sensores

### 🧭 Movimiento manual
En el **TurtleBot**:
```bash
ros2 topic echo /cmd_vel
```

En la **VM**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

### 📷 Ver la cámara
```bash
ros2 run rqt_image_view rqt_image_view
```

### 🌐 Activar sensores individualmente
- **LiDAR:**
  ```bash
  ros2 launch turtlebot4_bringup rplidar.launch.py
  ```
- **Cámara OAK-D:**
  ```bash
  ros2 launch turtlebot4_bringup oakd.launch.py
  ```

✅ Si escuchas el sonido alegre del robot (“pu puru pupu 🎵”), el bringup se cargó correctamente.

---

## 9️⃣ Notas y solución de errores

- **Cambiar o revisar el dominio ROS:**
  ```bash
  echo $ROS_DOMAIN_ID
  export ROS_DOMAIN_ID=4   # valor entre 0 y 255
  ```
  Usa el mismo valor en la VM y el TurtleBot.

- **Actualizar variables de entorno:**
  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

- **Verificar comunicación:**
  ```bash
  ros2 topic list
  ```

- **Si algo falla:**
  - Reinicia el Create 3 (`turtlebot4-setup`, aplicar red, reboot).
  - Revisa conexión Wi-Fi y ping entre VM ↔ TurtleBot.
  - Si nada resulta: **llamar a Cortijo** 😄
