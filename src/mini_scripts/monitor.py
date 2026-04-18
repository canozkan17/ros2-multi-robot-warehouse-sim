import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import sys

class LiveYawMonitor(Node):
    def __init__(self):
        super().__init__('live_yaw_monitor')
        # Sadece odometriyi dinliyoruz
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        print("🚀 Canlı Odometri/Açı Monitörü Başlatıldı... (Çıkmak için Ctrl+C)")
        print("-" * 60)

    def odom_callback(self, msg):
        # Senin kodundaki Quaternion -> Euler (Yaw) dönüşümü
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        # İnsan gözüyle daha rahat takip etmek için dereceye çeviriyoruz
        current_yaw_deg = math.degrees(current_yaw_rad)
        
        # Terminali tek satırda güncelleme işlemi
        # \r imleci başa alır, ljust(60) ise önceki uzun karakterlerin hayaletini siler
        output = f"\r[LIVE] Current Yaw Degree: {current_yaw_deg:+.3f}°"
        sys.stdout.write(output.ljust(60))
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = LiveYawMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C yapıldığında alt satıra geçerek temiz kapanış
        print("\n\n🛑 Monitör durduruldu. Terminale dönülüyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
