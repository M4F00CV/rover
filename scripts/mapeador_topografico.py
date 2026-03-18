import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
import json

class MapeadorTopograficoAvanzado(Node):
    def __init__(self):
        super().__init__('mapeador_topografico_avanzado')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        
        self.create_subscription(String, '/map_markers', self.marker_callback, 10)
        self.marcadores_descubiertos = [] # Lista para guardar lo que YOLO vea

        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.create_subscription(LaserScan, '/tf_luna/scan', self.scan_callback, qos_profile)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile)

        self.resolution = 0.05 
        self.map_size = 300 
        self.elevation_map = np.full((self.map_size, self.map_size), np.nan, dtype=np.float32)

        # Variables de Fusión de Sensores
        self.fused_x = 0.0
        self.fused_y = 0.0
        self.last_odom_x = None
        self.last_odom_y = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.odom_yaw = 0.0
        
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        
        self.path = []
        self.lidar_pitch_base = 35.0 * (np.pi / 180.0) 
        self.lidar_x_offset = 0.25 
        self.lidar_z_offset = 0.25
        
        self.timer = self.create_timer(0.5, self.update_display) 
        self.get_logger().info("🌍 Mapeador Fusión (Odom+IMU) Iniciado.")

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(q)
        euler = r.as_euler('xyz')
        self.imu_roll = euler[0]
        self.imu_pitch = euler[1]
        self.imu_yaw = euler[2] # Guardamos la brújula perfecta

    def marker_callback(self, msg):
        import json
        datos = json.loads(msg.data)
        
        # Calcular posición aproximada del objeto en el mapa
        dist = datos["dist"]
        # Asumimos que el objeto está frente al rover en ese momento
        obj_x = self.current_x + dist * math.cos(self.odom_yaw)
        obj_y = self.current_y + dist * math.sin(self.odom_yaw)
        
        nuevo_marcador = {"x": obj_x, "y": obj_y, "type": datos["type"], "name": datos["name"]}
        
        # Evitar duplicados (si ya hay uno igual a menos de 50 cm)
        duplicado = False
        for m in self.marcadores_descubiertos:
            if m["name"] == nuevo_marcador["name"] and math.hypot(m["x"]-obj_x, m["y"]-obj_y) < 0.5:
                duplicado = True
                break
                
        if not duplicado:
            self.marcadores_descubiertos.append(nuevo_marcador)

    def odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        if self.last_odom_x is None:
            self.last_odom_x = ox
            self.last_odom_y = oy
            return

        # 1. ¿Cuánto avanzaron las llantas realmente?
        dx = ox - self.last_odom_x
        dy = oy - self.last_odom_y
        dist_avanzada = math.hypot(dx, dy)

        # 2. Proyectar ese avance usando la dirección real del IMU
        self.fused_x += dist_avanzada * math.cos(self.imu_yaw)
        self.fused_y += dist_avanzada * math.sin(self.imu_yaw)

        self.last_odom_x = ox
        self.last_odom_y = oy

        # 3. Sobreescribir las coordenadas corruptas con nuestras coordenadas fusionadas
        self.current_x = self.fused_x
        self.current_y = self.fused_y
        self.odom_yaw = self.imu_yaw 
        
        if len(self.path) == 0 or np.hypot(self.current_x - self.path[-1][0], self.current_y - self.path[-1][1]) > 0.05:
            self.path.append((self.current_x, self.current_y))

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            dist = min(msg.ranges)
            if not (np.isinf(dist) or np.isnan(dist)):
                pitch_real = self.lidar_pitch_base + self.imu_pitch
                
                dist_horizontal = dist * np.cos(pitch_real)
                caida_vertical = dist * np.sin(pitch_real)
                
                impact_x_rel = self.lidar_x_offset + dist_horizontal
                impact_y_rel = 0.0 
                
                impact_x_global = self.current_x + (impact_x_rel * np.cos(self.odom_yaw) - impact_y_rel * np.sin(self.odom_yaw))
                impact_y_global = self.current_y + (impact_x_rel * np.sin(self.odom_yaw) + impact_y_rel * np.cos(self.odom_yaw))
                impact_z_global = self.current_z + self.lidar_z_offset - caida_vertical
                
                px = int((impact_x_global + 7.5) / self.resolution)
                py = self.map_size - int((impact_y_global + 7.5) / self.resolution)
                
                if 0 <= px < self.map_size and 0 <= py < self.map_size:
                    self.elevation_map[py, px] = impact_z_global

    def update_display(self):
        valid_pixels = ~np.isnan(self.elevation_map)
        if not np.any(valid_pixels):
            return 

        min_elev, max_elev = -0.3, 0.3
        
        elev_8bit = np.zeros_like(self.elevation_map, dtype=np.uint8)
        elev_normalized = (self.elevation_map[valid_pixels] - min_elev) / (max_elev - min_elev)
        elev_8bit[valid_pixels] = np.clip(elev_normalized * 255, 0, 255)
        
        mask = np.zeros_like(self.elevation_map, dtype=np.uint8)
        mask[valid_pixels] = 255
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        elev_dilated = cv2.dilate(elev_8bit, kernel)
        mask_dilated = cv2.dilate(mask, kernel)

        colormap = cv2.applyColorMap(elev_dilated, cv2.COLORMAP_JET)
        colormap[mask_dilated == 0] = [30, 30, 30] 

        escalones = (elev_dilated // 21) * 21
        bordes = cv2.Canny(escalones.astype(np.uint8), 10, 50)
        colormap[bordes > 0] = [255, 255, 255]

        display_img = cv2.resize(colormap, (600, 600), interpolation=cv2.INTER_NEAREST)

        if len(self.path) > 1:
            pts_scaled = []
            for (x, y) in self.path:
                px = int((x + 7.5) / self.resolution) * (600 / self.map_size)
                py = (self.map_size - int((y + 7.5) / self.resolution)) * (600 / self.map_size)
                pts_scaled.append([int(px), int(py)])
            cv2.polylines(display_img, [np.int32(pts_scaled)], False, (150, 150, 255), 2, cv2.LINE_AA)

        px_r = int(int((self.current_x + 7.5) / self.resolution) * (600 / self.map_size))
        py_r = int((self.map_size - int((self.current_y + 7.5) / self.resolution)) * (600 / self.map_size))
        cv2.circle(display_img, (px_r, py_r), 6, (0, 0, 255), -1)

        cv2.rectangle(display_img, (520, 50), (590, 250), (0,0,0), -1)
        cv2.rectangle(display_img, (520, 50), (550, 250), (255,255,255), 1)
        for i in range(200):
            val = 255 - int((i/200)*255)
            color = cv2.applyColorMap(np.uint8([[val]]), cv2.COLORMAP_JET)[0][0]
            cv2.line(display_img, (520, 50+i), (550, 50+i), color.tolist(), 1)

        # --- DIBUJAR MARCADORES YOLO ---
        for m in self.marcadores_descubiertos:
            px_m = int((m["x"] + 7.5) / self.resolution) * (600 / self.map_size)
            py_m = (self.map_size - int((m["y"] + 7.5) / self.resolution)) * (600 / self.map_size)
            px_m, py_m = int(px_m), int(py_m)
            
            if m["type"] == "roca":
                cv2.circle(display_img, (px_m, py_m), 8, (255, 0, 255), -1) # Punto Magenta para rocas
                cv2.putText(display_img, "Roca", (px_m+10, py_m), 0, 0.4, (255, 0, 255), 1)
            elif m["type"] == "letrero":
                cv2.rectangle(display_img, (px_m-10, py_m-10), (px_m+10, py_m+10), (0, 255, 255), -1) # Cuadro Amarillo para letreros
                cv2.putText(display_img, "META/BASE", (px_m+15, py_m), 0, 0.4, (0, 255, 255), 1)
        
        cv2.putText(display_img, "+30cm", (555, 60), 0, 0.4, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(display_img, " 0cm", (555, 155), 0, 0.4, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(display_img, "-30cm", (555, 250), 0, 0.4, (255,255,255), 1, cv2.LINE_AA)

        cv2.imshow("Radar Topografico Estabilizado", display_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MapeadorTopograficoAvanzado()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
