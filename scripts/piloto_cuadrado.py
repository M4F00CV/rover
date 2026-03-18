import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from scipy.spatial.transform import Rotation as R

class PilotoCuadrado(Node):
    def __init__(self):
        super().__init__('piloto_cuadrado')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.state = 'INIT'
        self.target_distance = 2.0  
        self.target_angle = math.pi / 2.0  
        self.sides_completed = 0
        
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        
        # Coordenadas Fusionadas
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0 
        self.last_odom_x = None
        self.last_odom_y = None
        
        self.odom_ready = False
        self.imu_ready = False
        
        self.timer = self.create_timer(0.05, self.control_loop) 
        self.get_logger().info("🤖 Piloto Inercial Avanzado Iniciado. Esperando sensores...")

    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(q)
        self.current_yaw = r.as_euler('xyz')[2]
        self.imu_ready = True

    def odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        
        if not self.odom_ready:
            self.last_odom_x = ox
            self.last_odom_y = oy
            self.odom_ready = True
            return

        # Fusión manual de avance y rotación
        dist = math.hypot(ox - self.last_odom_x, oy - self.last_odom_y)
        self.current_x += dist * math.cos(self.current_yaw)
        self.current_y += dist * math.sin(self.current_yaw)

        self.last_odom_x = ox
        self.last_odom_y = oy

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        msg = Twist()
        
        if self.state == 'INIT':
            if self.odom_ready and self.imu_ready:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_yaw = self.current_yaw
                self.state = 'MOVING_FORWARD'
                self.get_logger().info("🚀 Sensores en línea. Iniciando Lado 1")
            return
        
        if self.state == 'MOVING_FORWARD':
            distance_traveled = math.hypot(self.current_x - self.start_x, self.current_y - self.start_y)
            if distance_traveled < self.target_distance:
                msg.linear.x = 0.5  
            else:
                msg.linear.x = 0.0
                self.start_yaw = self.current_yaw 
                self.state = 'TURNING'
                self.get_logger().info(f"🔄 Lado {self.sides_completed + 1} listo. Girando 90° reales...")
                
        elif self.state == 'TURNING':
            angle_turned = abs(self.normalize_angle(self.current_yaw - self.start_yaw))
            error = self.target_angle - angle_turned
            
            if error > 0.05: 
                turn_speed = max(0.3, error * 1.5) 
                msg.angular.z = turn_speed
            else:
                msg.angular.z = 0.0
                self.sides_completed += 1
                if self.sides_completed < 4:
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                    self.state = 'MOVING_FORWARD'
                    self.get_logger().info(f"🚀 Iniciando Lado {self.sides_completed + 1}")
                else:
                    self.state = 'FINISHED'
                    self.get_logger().info("✅ Cuadrado Perfecto completado.")
                    
        elif self.state == 'FINISHED':
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PilotoCuadrado()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
