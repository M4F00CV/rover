import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import pytesseract
from ultralytics import YOLO
import time
import os
import json
from difflib import SequenceMatcher

class CerebroRoverMisionNode(Node):
    def __init__(self):
        super().__init__('cerebro_rover_mision_node')

        self.get_logger().info('Inicializando Cerebro ROS 2 (YOLO + FSM + OCR)...')
        
        ruta_modelo = os.path.join(os.path.dirname(__file__), 'best.pt')
        self.model = YOLO(ruta_modelo)
        self.bridge = CvBridge()

        # Suscriptores y Publicadores
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)
        self.tilt_pub = self.create_publisher(Float64, '/camera_tilt/cmd_pos', 10)
        self.marker_pub = self.create_publisher(String, '/map_markers', 10) # Envía rocas al mapa

        # Máquina de Estados
        self.estado_actual = "RECOLECTANDO_PIEDRAS"
        self.total_piedras_en_arena = 3
        self.piedras_en_garra = 0
        self.ultimo_ocr_time = time.time()
        
        # Enviar cámara a posición inicial (Mirando al suelo)
        self.mover_camara(0.5) # 0.5 rad hacia abajo
        self.get_logger().info('STATUS: Mirando al suelo. Buscando rocas lunares.')

    def mover_camara(self, angulo_radianes):
        msg = Float64()
        msg.data = float(angulo_radianes)
        self.tilt_pub.publish(msg)

    def reportar_hallazgo_al_mapa(self, tipo, distancia_estimada, clase_nombre):
        # Enviaremos un JSON simple al mapeador
        data = {"type": tipo, "name": clase_nombre, "dist": distancia_estimada}
        msg = String()
        msg.data = json.dumps(data)
        self.marker_pub.publish(msg)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        results = self.model(frame, conf=0.4, verbose=False)
        annotated_frame = results[0].plot() 

        h, w, _ = annotated_frame.shape
        cv2.rectangle(annotated_frame, (0, 0), (w, 100), (0,0,0), -1)

        color_fase = (0, 255, 0)
        if "BUSCANDO" in self.estado_actual: color_fase = (0, 255, 255)
        if "REGRESANDO" in self.estado_actual: color_fase = (0, 165, 255)

        cv2.putText(annotated_frame, f"FASE: {self.estado_actual}", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_fase, 2)
        cv2.putText(annotated_frame, f"Piedras: {self.piedras_en_garra}/{self.total_piedras_en_arena}", (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Análisis de las detecciones de YOLO
        for box in results[0].boxes:
            clase_nombre = self.model.names[int(box.cls[0])]
            
            # 1. Si estamos recolectando y vemos una roca, avisar al mapa
            if self.estado_actual == "RECOLECTANDO_PIEDRAS" and "roca" in clase_nombre.lower():
                # Estimación burda: si la caja es grande, está cerca (ej. 1.0 m). Si es chica, está lejos.
                alto_caja = float(box.xyxy[0][3] - box.xyxy[0][1])
                distancia_estimada = 1.0 if alto_caja > 100 else 3.0 
                self.reportar_hallazgo_al_mapa("roca", distancia_estimada, clase_nombre)

            # 2. Lógica de Letreros
            if self.estado_actual in ["BUSCANDO_META", "REGRESANDO_INICIO"]:
                # --- AQUÍ ADAPTAMOS PARA EL SIMULADOR ---
                # Como el OCR fallará en los bloques grises de Gazebo, confiamos en la clase de YOLO
                if "letrero" in clase_nombre.lower() or "meta" in clase_nombre.lower() or "fin" in clase_nombre.lower():
                    
                    self.reportar_hallazgo_al_mapa("letrero", 2.0, clase_nombre)
                    cv2.putText(annotated_frame, f"YOLO CLASIFICO: {clase_nombre}", (w-400, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
                    
                    if self.estado_actual == "BUSCANDO_META":
                        self.get_logger().info('META DETECTADA por YOLO. Retornando a base.')
                        self.estado_actual = "REGRESANDO_INICIO"
                        
                    elif self.estado_actual == "REGRESANDO_INICIO":
                        self.get_logger().info('BASE DETECTADA por YOLO. Misión completada.')
                        self.estado_actual = "MISION_FINALIZADA"

        if self.estado_actual == "MISION_FINALIZADA":
            cv2.putText(annotated_frame, "MISION CUMPLIDA", (w//4, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 4)

        cv2.imshow("Rover Control Center", annotated_frame)

        # SIMULADOR TECLADO
        key = cv2.waitKey(1)
        if key == ord('r') or key == ord('R'):
            if self.estado_actual == "RECOLECTANDO_PIEDRAS" and self.piedras_en_garra < self.total_piedras_en_arena:
                self.piedras_en_garra += 1
                self.get_logger().info(f'Piedra recogida: {self.piedras_en_garra}/{self.total_piedras_en_arena}')
                if self.piedras_en_garra == self.total_piedras_en_arena:
                    self.get_logger().info('Piedras completas. Levantando cámara y Buscando META.')
                    self.mover_camara(-0.2) # Levanta la cámara para mirar al horizonte
                    self.estado_actual = "BUSCANDO_META"

def main(args=None):
    rclpy.init(args=args)
    node = CerebroRoverMisionNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
