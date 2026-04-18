import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import threading
import subprocess
import json
import time


class ProTurner(Node):
    def __init__(self):
        super().__init__('pro_turner')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.raw_yaw = 0.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw_offset = 0.0
        self.offset_set = False
        self.is_turning = False
        self.odom_received = False
        self.correction_round = 0
        self.max_corrections = 6
        self.fine_threshold = 0.01
        self.final_threshold = 0.00035
        self.slow_min_speed = 0.008
        self.pending_calibration = False
        self.calibration_target_yaw = None
        self.gazebo_probe_enabled = True
        self.gazebo_probe_failed_once = False
        self.last_gazebo_probe_ts = 0.0
        self.gazebo_probe_interval = 0.2
        
        # Kontrol dongusu frekansini artirdik (daha hassas takip)
        self.timer = self.create_timer(0.02, self.control_loop) 

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = self.normalize_angle(self.raw_yaw - self.yaw_offset)
        self.odom_received = True

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_current_yaw(self):
        return self.normalize_angle(self.raw_yaw - self.yaw_offset)

    def control_yaw(self):
        if self.pending_calibration and hasattr(self, 'gazebo_yaw'):
            return self.normalize_angle(self.gazebo_yaw)
        return self.get_current_yaw()

    def start_turn(self, angle_deg):
        if not self.odom_received:
            return
        self.current_yaw = self.get_current_yaw()
        self.target_yaw = self.normalize_angle(self.current_yaw + math.radians(angle_deg))
        self.is_turning = True

    def control_loop(self):
        if not self.is_turning: return

        if self.pending_calibration and self.gazebo_probe_enabled:
            now = time.monotonic()
            if now - self.last_gazebo_probe_ts >= self.gazebo_probe_interval:
                self.get_gazebo_world_yaw()
                self.last_gazebo_probe_ts = now

        self.current_yaw = self.control_yaw()
        error = self.normalize_angle(self.target_yaw - self.current_yaw)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if abs(error) < self.final_threshold:
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.is_turning = False
            self.correction_round = 0
            if self.pending_calibration and self.calibration_target_yaw is not None:
                # Hedef eksene (0/90/180/270) dönüldüğünde odometri ofsetini buna sabitliyoruz.
                # Gazebo yaw sadece hedef seçimi için kullanılıyor; sonrasında current_yaw hedefe eşit olmalı.
                self.yaw_offset = self.raw_yaw - self.calibration_target_yaw
                self.current_yaw = self.normalize_angle(self.raw_yaw - self.yaw_offset)
                self.pending_calibration = False
                print(f"Calibration aligned. Yaw offset: {math.degrees(self.yaw_offset):.3f}°, rel: {math.degrees(self.current_yaw):.2f}°")
            gazebo_str = f", gazebo: {math.degrees(self.gazebo_yaw):.2f}" if hasattr(self, 'gazebo_yaw') else ''
            print(f"Done. Current Degree (rel): {math.degrees(self.current_yaw):.2f}, raw: {math.degrees(self.raw_yaw):.2f}{gazebo_str}")
            return
        elif abs(error) < self.fine_threshold:
            speed = 0.4 * error
            if speed > 0:
                msg.twist.angular.z = max(speed, self.slow_min_speed)
            else:
                msg.twist.angular.z = min(speed, -self.slow_min_speed)
            self.cmd_pub.publish(msg)
            return

        # Yavaslama egrisi (Hedefe yaklastikca hizi kısıp overshoot'u engelliyoruz)
        # Kp degeri (1.5) ile oynayarak sertligi ayarlayabilirsin
        speed = 1.5 * error 
        
        # Hiz limitleri: Cok yavas donerse takilir, cok hizli donerse savrulur
        max_s = 0.3
        min_s = 0.05
        
        if speed > 0:
            msg.twist.angular.z = min(max(speed, min_s), max_s)
        else:
            msg.twist.angular.z = max(min(speed, -min_s), -max_s)
            
        self.cmd_pub.publish(msg)

    def parse_json_stream(self, text):
        decoder = json.JSONDecoder()
        idx = 0
        text = text.strip()
        while idx < len(text):
            try:
                obj, end = decoder.raw_decode(text[idx:])
                return obj
            except json.JSONDecodeError:
                idx = text.find('{', idx + 1)
                if idx == -1:
                    break
        return None

    def get_gazebo_world_yaw(self):
        if not self.gazebo_probe_enabled:
            return None
        try:
            result = subprocess.run(
                ['gz', 'topic', '-e', '-t', '/world/world_demo/pose/info', '--json-output', '-n', '1'],
                capture_output=True,
                text=True,
                timeout=5.0,
                check=True
            )
            data = self.parse_json_stream(result.stdout)
            if not data:
                return None
            for pose in data.get('pose', []):
                if pose.get('name') == 'robot1':
                    q = pose.get('orientation', {})
                    x = float(q.get('x', 0.0))
                    y = float(q.get('y', 0.0))
                    z = float(q.get('z', 0.0))
                    w = float(q.get('w', 1.0))
                    siny_cosp = 2.0 * (w * z + x * y)
                    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    self.gazebo_yaw = yaw
                    return yaw
        except Exception as e:
            if not self.gazebo_probe_failed_once:
                print(f'Gazebo world yaw read failed: {e}')
                print('Gazebo yaw probing disabled; continuing with odometry yaw.')
            self.gazebo_probe_failed_once = True
            self.gazebo_probe_enabled = False
        return None

    def calibrate_yaw(self, align=True):
        if not self.odom_received:
            print('Odom not received yet, calibration failed.')
            return
        gazebo_yaw = self.get_gazebo_world_yaw()
        if gazebo_yaw is None:
            print('Gazebo world yaw not available; using raw odom snap calibration.')
            raw_deg = math.degrees(self.raw_yaw)
            snapped_deg = round(raw_deg / 90.0) * 90.0
            target_yaw = math.radians(snapped_deg)
        else:
            gazebo_deg = math.degrees(gazebo_yaw)
            snapped_deg = round(gazebo_deg / 90.0) * 90.0
            target_yaw = math.radians(snapped_deg)
            print(f'Gazebo yaw {gazebo_deg:.2f}°, snapping to {snapped_deg:.0f}°')

        if align:
            self.calibration_target_yaw = self.normalize_angle(target_yaw)
            self.pending_calibration = True
            self.target_yaw = self.calibration_target_yaw
            self.is_turning = True
            print(f'Calibrating and aligning to {snapped_deg:.0f}° (relative).')
            return

        self.yaw_offset = self.raw_yaw - target_yaw
        self.current_yaw = self.normalize_angle(self.raw_yaw - self.yaw_offset)
        self.offset_set = True
        print(f'Calibrated yaw offset: {math.degrees(self.yaw_offset):.3f}° (snapped to {snapped_deg:.0f}° grid axis).')

    def get_display_yaw(self):
        return math.degrees(self.current_yaw), math.degrees(self.raw_yaw), math.degrees(self.yaw_offset)


def keyboard_loop(node):
    print("\n'l' sola, 'r' saga, 'w' ileri (0.14m/s), 'c' kalibre et, 'h' aci durumu, 's' DUR, 'q' cikis")
    while rclpy.ok():
        c = input("Command: ").strip().lower()
        if c == 'l': node.start_turn(90)
        elif c == 'r': node.start_turn(-90)
        elif c == 'w':
            m = TwistStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.linear.x = 0.14
            node.cmd_pub.publish(m)
        elif c == 's':
            node.is_turning = False
            m = TwistStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            node.cmd_pub.publish(m)
        elif c == 'c':
            node.calibrate_yaw(align=True)
        elif c == 'h':
            cur, raw, off = node.get_display_yaw()
            print(f'Aci durumu -> rel: {cur:.2f} deg, raw: {raw:.2f} deg, offset: {off:.2f} deg')
        elif c == 'q':
            safe_shutdown()
            break


def safe_shutdown():
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

def main():
    rclpy.init()
    node = ProTurner()
    threading.Thread(target=keyboard_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    safe_shutdown()

if __name__ == '__main__': main()