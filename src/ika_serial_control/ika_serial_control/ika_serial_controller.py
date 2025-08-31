#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
import serial
import threading
import time
import math


class IKASerialController(Node):
    def __init__(self):
        super().__init__('ika_serial_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('max_linear_speed', 170)  # IKA için PWM limiti
        self.declare_parameter('max_angular_speed', 60.0)  # Maksimum dönüş açısı
        
        # Scaling parametreleri
        self.declare_parameter('linear_scale_factor', 85.0)  # 2 m/s = 170 PWM (170/2=85)
        self.declare_parameter('angular_scale_factor', 60.0)  # 1 rad/s = 60 derece
        self.declare_parameter('min_pwm_threshold', 10)      # Minimum PWM değeri
        self.declare_parameter('max_cmd_linear', 2.0)        # Maksimum kabul edilen linear hız
        self.declare_parameter('max_cmd_angular', 1.5)       # Maksimum kabul edilen angular hız
        
        # IKA özel parametreleri
        self.declare_parameter('enable_steering', True)      # Direksiyon kontrolü aktif mi
        self.declare_parameter('steering_sensitivity', 1.0)  # Direksiyon hassasiyeti
        self.declare_parameter('wheel_base', 0.8)           # Tekerlekler arası mesafe (m)
        self.declare_parameter('track_width', 0.78)         # Tekerlek izi genişliği (m)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().integer_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # Scaling parametreleri
        self.linear_scale_factor = self.get_parameter('linear_scale_factor').get_parameter_value().double_value
        self.angular_scale_factor = self.get_parameter('angular_scale_factor').get_parameter_value().double_value
        self.min_pwm_threshold = self.get_parameter('min_pwm_threshold').get_parameter_value().integer_value
        self.max_cmd_linear = self.get_parameter('max_cmd_linear').get_parameter_value().double_value
        self.max_cmd_angular = self.get_parameter('max_cmd_angular').get_parameter_value().double_value
        
        # IKA özel parametreleri
        self.enable_steering = self.get_parameter('enable_steering').get_parameter_value().bool_value
        self.steering_sensitivity = self.get_parameter('steering_sensitivity').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        
        # Serial connection
        self.serial_connection = None
        self.connect_to_arduino()
        
        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ROS2 publishers
        self.status_pub = self.create_publisher(String, 'ika_status', 10)
        self.steer_angle_pub = self.create_publisher(Float32, 'current_steer_angle', 10)
        self.current_speed_pub = self.create_publisher(Int32, 'current_speed', 10)
        self.wheel_speeds_pub = self.create_publisher(String, 'wheel_speeds', 10)
        
        # Control variables
        self.current_steer_angle = 0.0
        self.current_speed = 0
        self.last_command_time = time.time()
        self.command_timeout = 0.5  # 500ms timeout
        
        # IKA tekerlek durumları
        self.wheel_speeds = {
            'sol_on_teker': 0,
            'sol_orta_teker': 0,
            'sol_arka_teker': 0,
            'sag_on_teker': 0,
            'sag_orta_teker': 0,
            'sag_arka_teker': 0
        }
        
        # Start serial reader thread
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()
        
        # Timer for command timeout
        self.command_timer = self.create_timer(0.1, self.check_command_timeout)
        
        self.get_logger().info(f'IKA Serial Controller started on {self.serial_port}')

    def connect_to_arduino(self):
        """Arduino ile serial bağlantı kur"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Arduino'nun başlatılması için bekle
            self.get_logger().info(f'Connected to IKA Arduino on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to IKA Arduino: {str(e)}')
            self.serial_connection = None

    def cmd_vel_callback(self, msg):
        """Twist mesajını IKA Arduino komutuna çevir"""
        if self.serial_connection is None:
            self.get_logger().warn('No serial connection available')
            return
        
        # Input sınırlama
        linear_input = max(-self.max_cmd_linear, min(self.max_cmd_linear, msg.linear.x))
        angular_input = max(-self.max_cmd_angular, min(self.max_cmd_angular, msg.angular.z))
        
        # Linear velocity'yi PWM değerine çevir
        linear_abs = abs(linear_input)
        if linear_abs < 0.01:  # Çok küçük değerleri sıfırla
            speed_pwm = 0
        else:
            # Scaling factor kullanarak PWM hesapla
            speed_pwm = int(linear_abs * self.linear_scale_factor)
            
            # Minimum threshold uygula
            if speed_pwm > 0 and speed_pwm < self.min_pwm_threshold:
                speed_pwm = self.min_pwm_threshold
            
            # Maksimum sınır
            speed_pwm = min(speed_pwm, self.max_linear_speed)
            
            # Geri gidiş için işaret
            if linear_input < 0:
                speed_pwm = -speed_pwm
        
        # Angular velocity'yi direksiyon açısına çevir
        if abs(angular_input) < 0.01:  # Çok küçük değerleri sıfırla
            steer_angle = 0.0
        else:
            # Scaling factor kullanarak açı hesapla
            steer_angle = angular_input * self.angular_scale_factor * self.steering_sensitivity
            
            # Maksimum açı sınırı
            steer_angle = max(-self.max_angular_speed, 
                             min(self.max_angular_speed, steer_angle))
        
        self.send_ika_command(steer_angle, speed_pwm)
        self.last_command_time = time.time()
        
        # Debug log
        self.get_logger().debug(f'IKA Input: linear={linear_input:.2f}, angular={angular_input:.2f} → '
                               f'Output: pwm={speed_pwm}, angle={steer_angle:.1f}°')

    def send_ika_command(self, steer_angle, speed):
        """IKA Arduino'ya komut gönder"""
        if self.serial_connection is None:
            return
        
        try:
            # IKA komut formatı: "STEER:angle,SPEED:speed\n"
            command = f"STEER:{steer_angle:.1f},SPEED:{speed}\n"
            self.serial_connection.write(command.encode())
            self.get_logger().debug(f'Sent IKA command: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send IKA command: {str(e)}')

    def check_command_timeout(self):
        """Komut timeout kontrolü - güvenlik için"""
        if time.time() - self.last_command_time > self.command_timeout:
            # Timeout durumunda dur
            if self.serial_connection:
                self.send_ika_command(0.0, 0)
                self.get_logger().warn('Command timeout - stopping IKA')

    def serial_reader(self):
        """Arduino'dan gelen mesajları oku"""
        while rclpy.ok() and self.serial_connection:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode().strip()
                    if line:
                        self.parse_ika_message(line)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {str(e)}')
                time.sleep(0.1)

    def parse_ika_message(self, message):
        """IKA Arduino mesajlarını parse et"""
        if message.startswith("STATUS:"):
            # STATUS: steer=15.0, pwm=150, sol_on=120, sol_orta=125, sol_arka=118, sag_on=122, sag_orta=127, sag_arka=120
            self.status_pub.publish(String(data=message))
            
            # Parse individual values
            parts = message.replace("STATUS: ", "").split(", ")
            for part in parts:
                if "steer=" in part:
                    steer_val = float(part.split("=")[1])
                    self.steer_angle_pub.publish(Float32(data=steer_val))
                    self.current_steer_angle = steer_val
                elif "pwm=" in part:
                    pwm_val = int(part.split("=")[1])
                    self.current_speed_pub.publish(Int32(data=pwm_val))
                    self.current_speed = pwm_val
                elif "sol_on=" in part:
                    self.wheel_speeds['sol_on_teker'] = int(part.split("=")[1])
                elif "sol_orta=" in part:
                    self.wheel_speeds['sol_orta_teker'] = int(part.split("=")[1])
                elif "sol_arka=" in part:
                    self.wheel_speeds['sol_arka_teker'] = int(part.split("=")[1])
                elif "sag_on=" in part:
                    self.wheel_speeds['sag_on_teker'] = int(part.split("=")[1])
                elif "sag_orta=" in part:
                    self.wheel_speeds['sag_orta_teker'] = int(part.split("=")[1])
                elif "sag_arka=" in part:
                    self.wheel_speeds['sag_arka_teker'] = int(part.split("=")[1])
            
            # Tekerlek hızlarını yayınla
            wheel_speeds_str = f"sol_on:{self.wheel_speeds['sol_on_teker']}, " \
                              f"sol_orta:{self.wheel_speeds['sol_orta_teker']}, " \
                              f"sol_arka:{self.wheel_speeds['sol_arka_teker']}, " \
                              f"sag_on:{self.wheel_speeds['sag_on_teker']}, " \
                              f"sag_orta:{self.wheel_speeds['sag_orta_teker']}, " \
                              f"sag_arka:{self.wheel_speeds['sag_arka_teker']}"
            self.wheel_speeds_pub.publish(String(data=wheel_speeds_str))
                    
        elif message.startswith("CMD_ACK:"):
            self.get_logger().debug(f'IKA Arduino ACK: {message}')
        elif message.startswith("STEER_SET:"):
            steer_val = float(message.split(":")[1])
            self.get_logger().info(f'IKA Steering set to: {steer_val}°')
        elif message.startswith("SPEED_SET:"):
            speed_val = int(message.split(":")[1])
            self.get_logger().info(f'IKA Speed set to: {speed_val}')
        elif message.startswith("ERROR:"):
            self.get_logger().warn(f'IKA Arduino Error: {message}')
        else:
            self.get_logger().info(f'IKA Arduino: {message}')

    def destroy_node(self):
        """Node kapanırken temizlik yap"""
        if self.serial_connection:
            # Son komut olarak dur
            self.send_ika_command(0.0, 0)
            time.sleep(0.1)
            self.serial_connection.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IKASerialController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
