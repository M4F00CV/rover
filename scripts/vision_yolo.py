import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math
import os
import numpy as np

class RoverVision(Node):
    def __init__(self):
        super().__init__('rover_vision_yolo')
        
        self.bridge = CvBridge()
        ruta_modelo = os.path.join(os.path.dirname(__file__), 'best.pt')
        self.model = YOLO(ruta_modelo)
        
        self.colores_clases = {
            'roca_azul': (255, 0, 0),    
            'roca_verde': (0, 255, 0),  
            'roca_roja': (0, 0, 255)    
        }
        
        qos_profile = rclpy.qos.qos_profile_sensor_data
        
        # Suscriptores (Ojos y Láser)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)
        self.create_subscription(LaserScan, '/tf_luna/scan', self.scan_callback, qos_profile)
        
        # Publicador (Motor del LiDAR)
        self.pan_pub = self.create_publisher(Float64, '/lidar_pan/cmd_pos', 10)
        
        # Parámetros de la cámara Arducam simulada
        self.fov = 1.047 # FOV horizontal de ~60 grados en radianes
        self.img_width = 640
        
        self.tracked_rock = None
        self.last_distance = float('inf')
        
        self.get_logger().info("--- IA + LiDAR INICIADOS. Buscando rocas... ---")

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            dist = min(msg.ranges) # Extraemos el rayo central/más corto
            self.last_distance = dist

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        results = self.model(img, conf=0.35, verbose=False)
        best_box = None
        max_area = 0

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                nombre_clase = self.model.names[cls_id] 
                color_box = self.colores_clases.get(nombre_clase, (255, 255, 255))

                cv2.rectangle(img, (x1, y1), (x2, y2), color_box, 2)
                conf = float(box.conf[0])
                label = f'{nombre_clase} {int(conf*100)}%'
                
                t_size = cv2.getTextSize(label, 0, fontScale=0.6, thickness=2)[0]
                c2 = x1 + t_size[0], y1 - t_size[1] - 3
                cv2.rectangle(img, (x1, y1), c2, color_box, -1, cv2.LINE_AA) 
                cv2.putText(img, label, (x1, y1 - 2), 0, 0.6, (255, 255, 255), thickness=1, lineType=cv2.LINE_AA)
                
                area = (x2 - x1) * (y2 - y1)
                if area > max_area:
                    max_area = area
                    best_box = (x1, x2, nombre_clase)
        
        if best_box:
            x1, x2, nombre_clase = best_box
            center_x = (x1 + x2) / 2.0 
            angle = (self.img_width / 2.0 - center_x) * (self.fov / self.img_width)
            
            msg_pos = Float64()
            msg_pos.data = float(angle)
            self.pan_pub.publish(msg_pos)
            
            cv2.line(img, (int(center_x), 0), (int(center_x), 480), (0, 255, 255), 2)
            
            # --- NUEVO: Mostrar siempre la distancia en pantalla ---
            if np.isinf(self.last_distance) or np.isnan(self.last_distance):
                texto_dist = "Rango: Infinito (Cielo)"
            else:
                texto_dist = f"Rango: {self.last_distance:.2f} m"
                
            cv2.putText(img, texto_dist, (10, 30), 0, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
            # -------------------------------------------------------
            
        else:
            msg_pos = Float64()
            msg_pos.data = 0.0
            self.pan_pub.publish(msg_pos)

        cv2.imshow('ROVER VISION + RADAR', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RoverVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
