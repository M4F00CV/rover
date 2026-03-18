# 🚀 Simulador Lunar - Misión Autónoma (TMR)

Este repositorio contiene el entorno de simulación en **ROS 2 y Gazebo** para un rover lunar autónomo de 6 ruedas. El sistema está diseñado para el Torneo Mexicano de Robótica (TMR) e incluye capacidades avanzadas de navegación inercial, mapeo topográfico 2.5D, y visión artificial.

## 🧠 Características Principales
* **Fusión de Sensores (Odometría + IMU):** Corrección de derrape (Wheel Slip) y estabilización de cabeceo utilizando un MPU-6050 simulado.
* **Mapeo Topográfico 2.5D:** Generación de mapas de relieve y curvas de nivel en tiempo real usando un LiDAR TF-Luna frontal ("Bastón de Ciego").
* **Visión Artificial y OCR:** Detección de rocas de colores con YOLOv8 y lectura de letreros (Inicio/Fin) utilizando Tesseract OCR.
* **Máquina de Estados Finita (FSM):** Control autónomo de misiones (Recolección -> Búsqueda de Meta -> Retorno a Base).

## 💻 Requisitos del Sistema
* **SO:** Ubuntu 24.04 LTS (Nativo o vía WSL2 en Windows)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Simulador:** Gazebo Harmonic
* **Python:** 3.12+

## ⚙️ Instalación de Dependencias

Antes de compilar el espacio de trabajo, asegúrate de instalar todas las dependencias del sistema y de Python.

**1. Dependencias del Sistema (ROS 2 y Tesseract OCR)**
```bash
sudo apt update
sudo apt install tesseract-ocr tesseract-ocr-spa -y
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-teleop-twist-keyboard -y

Dependencias de python

pip3 install "numpy<2" --break-system-packages
pip3 install opencv-python ultralytics pytesseract scipy --break-system-packages

(Nota: Se usa numpy<2 por compatibilidad estricta con cv_bridge en ROS 2 Jazzy).

🚀 Guía de Ejecución (Orquestación de Terminales)
Para correr la simulación completa, abre diferentes terminales y ejecuta los comandos en este orden:

Terminal 1: Lanzar Gazebo
cd ~/rover_ws/src/lunar_simulation/worlds
export GZ_SIM_RENDER_ENGINE_SERVER=ogre
export GZ_SIM_RENDER_ENGINE_GUI=ogre
gz sim lunar_arena.sdf

(⚠️ Recuerda presionar el botón de Play en Gazebo).

Terminal 2: Puente ROS-Gazebo (Puente Direccional)
ros2 run ros_gz_bridge parameter_bridge \
/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
/tf_luna/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
/camera_tilt/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double \
--ros-args -p use_sim_time:=true

Terminal 3: Control de Movimiento
Para manejo manual:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

(Opcional) Para piloto autónomo de pruebas:
python3 ~/rover_ws/src/lunar_simulation/scripts/piloto_cuadrado.py

Terminal 4: Cerebro de Visión y FSM
python3 ~/rover_ws/src/lunar_simulation/scripts/cerebro_rover.py
(Presiona la tecla R en la ventana de video para simular la recolección de piedras con el brazo robótico).

Terminal 5: Radar Topográfico
python3 ~/rover_ws/src/lunar_simulation/scripts/mapeador_topografico.py


