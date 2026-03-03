import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import cv2

class TopographicMapper(Node):
    def __init__(self):
        super().__init__('topographic_mapper')
        
        self.resolution = 0.05
        self.map_size = 200
        # Iniciamos con un fondo gris oscuro para que resalten las alturas
        self.grid = np.full((self.map_size, self.map_size), 100, dtype=np.uint8)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0  # <--- ¡Atraparemos la altitud aquí!
        self.last_lidar_dist = 0.1 
        
        qos_profile = rclpy.qos.qos_profile_sensor_data
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.create_subscription(LaserScan, '/tf_luna/scan', self.scan_callback, qos_profile)
        
        self.timer = self.create_timer(0.1, self.update_display)
        self.get_logger().info("Nodo Cartógrafo Topográfico Real Listo. ¡A conducir!")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z  # <--- Altura global del rover

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            dist = msg.ranges[0]
            if not (np.isinf(dist) or np.isnan(dist)):
                self.last_lidar_dist = dist
                
    def update_display(self):
        # Topografía = Altura del rover - distancia al suelo
        elevacion_real = self.current_z - self.last_lidar_dist
        
        # Mapeamos la elevación a color: 
        # Zonas altas = Blanco (255), Zonas bajas/surcos = Negro (0)
        # Ajustamos el rango dependiendo de las alturas de tu modelo 3D
        color_gris = int(np.clip((elevacion_real + 0.2) / 0.4 * 255, 0, 255))
        
        px = int((self.current_x + 5.0) / self.resolution)
        py = self.map_size - int((self.current_y + 5.0) / self.resolution)
        
        if 0 <= px < self.map_size and 0 <= py < self.map_size:
            cv2.circle(self.grid, (px, py), 2, color_gris, -1)
            
        cv2.imshow("Mapeo Topografico Lunar", cv2.resize(self.grid, (600, 600), interpolation=cv2.INTER_NEAREST))
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TopographicMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
