#!/bin/bash
echo "========================================="
echo " Instalando Dependencias - Rover TMR     "
echo "========================================="

echo "[1/3] Actualizando repositorios..."
sudo apt update

echo "[2/3] Instalando librerías del sistema y ROS 2..."
sudo apt install -y tesseract-ocr tesseract-ocr-spa
sudo apt install -y ros-jazzy-ros-gz-bridge ros-jazzy-teleop-twist-keyboard

echo "[3/3] Instalando librerías de Python..."
pip3 install "numpy<2" --break-system-packages
pip3 install opencv-python ultralytics pytesseract scipy --break-system-packages

echo "========================================="
echo " ✅ Instalación Completada con Éxito.    "
echo "========================================="
