import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class RadarMonitor(Node):
    def __init__(self):
        super().__init__('radar_monitor')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(LaserScan, '/tf_luna/scan', self.scan_callback, qos_profile)
        self.get_logger().info("📡 Monitor de Radar Activo. Esperando objetivo...")

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            # Tomamos el rayo más corto de los 3 que disparamos
            dist = min(msg.ranges) 
            
            if np.isinf(dist) or np.isnan(dist):
                self.get_logger().info("[LiDAR] Rango: Infinito (Buscando...)")
            else:
                self.get_logger().info(f"[LiDAR] ¡Objetivo a {dist:.3f} metros!")

def main(args=None):
    rclpy.init(args=args)
    node = RadarMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
