#!/bin/bash
echo "Instalando dependencias de ROS 2 Jazzy para el Rover Lunar..."

sudo apt update

# Puente entre ROS 2 y Gazebo Harmonic
sudo apt install -y ros-jazzy-ros-gz

# Controladores y simuladores (Diff Drive, Joint States, etc.)
sudo apt install -y ros-jazzy-xacro \
                    ros-jazzy-ros2-control \
                    ros-jazzy-diff-drive-controller \
                    ros-jazzy-joint-state-broadcaster

# Mensajes de sensores, navegación y visión
sudo apt install -y ros-jazzy-sensor-msgs \
                    ros-jazzy-nav-msgs \
                    ros-jazzy-geometry-msgs \
                    ros-jazzy-visualization-msgs \
                    ros-jazzy-cv-bridge

# Herramienta de control remoto (Teclado)
sudo apt install -y ros-jazzy-teleop-twist-keyboard

echo "¡Dependencias de ROS 2 instaladas correctamente!"
