#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty
import select
import math
import time

KEY_MAPPINGS = {
    'w': (1.0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward  
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    'q': (1.0, 1.0),    # Forward + left
    'e': (1.0, -1.0),   # Forward + right
    'z': (-1.0, 1.0),   # Backward + left
    'c': (-1.0, -1.0),  # Backward + right
    'x': (0.0, 0.0),    # Stop
}

class RoverTeleopNode(Node):
    def __init__(self):
        super().__init__('rover_teleop_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_wheels_pub = self.create_publisher(Float64MultiArray, '/left_wheels_controller/commands', 10)
        self.right_wheels_pub = self.create_publisher(Float64MultiArray, '/right_wheels_controller/commands', 10)
        self.front_steering_pub = self.create_publisher(Float64MultiArray, '/front_steering_controller/commands', 10)
        self.rear_steering_pub = self.create_publisher(Float64MultiArray, '/rear_steering_controller/commands', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_linear_speed', 2.0),
                ('max_angular_speed', 1.5),
                ('speed_increment', 0.1),
                ('control_method', 'cmd_vel'),  # 'cmd_vel' or 'direct'
                ('wheel_base_length', 1.2),
                ('wheel_base_width', 0.8)
            ]
        )
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.control_method = self.get_parameter('control_method').value
        self.wheel_base_length = self.get_parameter('wheel_base_length').value
        self.wheel_base_width = self.get_parameter('wheel_base_width').value

        self.current_linear_speed = 0.5
        self.current_angular_speed = 0.5

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Rover teleop node started')
        self.get_logger().info(f'Control method: {self.control_method}')
        self.print_usage()

    def print_usage(self):
        print("""
Rover Teleop Control:
---------------------------
Movement:
  w/s: Forward/Backward
  a/d: Turn Left/Right  
  q/e: Forward+Left/Right
  z/c: Backward+Left/Right
  x: Stop

Speed Control:
  t/g: Increase/Decrease linear speed
  r/f: Increase/Decrease angular speed

Other:
  SPACE: Emergency stop
  CTRL+C: Quit

Current speeds:
  Linear: {:.1f} m/s (max: {:.1f})
  Angular: {:.1f} rad/s (max: {:.1f})
        """.format(
            self.current_linear_speed, self.max_linear_speed,
            self.current_angular_speed, self.max_angular_speed
        ))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def is_data(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def calculate_ackermann_steering(self, linear_vel, angular_vel):
        if abs(angular_vel) < 1e-6:
            return 0.0, 0.0, 0.0, 0.0
        if abs(linear_vel) < 1e-6:
            turn_angle = math.pi / 4  # 45 degrees
            return turn_angle, -turn_angle, -turn_angle, turn_angle
        turn_radius = linear_vel / angular_vel
        if turn_radius > 0:  # Left turn
            front_left_angle = math.atan(self.wheel_base_length / (turn_radius + self.wheel_base_width/2))
            front_right_angle = math.atan(self.wheel_base_length / (turn_radius - self.wheel_base_width/2))
        else:  # Right turn
            front_left_angle = math.atan(self.wheel_base_length / (abs(turn_radius) - self.wheel_base_width/2))
            front_right_angle = math.atan(self.wheel_base_length / (abs(turn_radius) + self.wheel_base_width/2))
            front_left_angle = -front_left_angle
            front_right_angle = -front_right_angle
        rear_left_angle = -front_left_angle
        rear_right_angle = -front_right_angle
        return front_left_angle, front_right_angle, rear_left_angle, rear_right_angle

    def run(self):
        try:
            rate = 10  # Hz
            while rclpy.ok():
                if self.is_data():  # Tuş basıldıysa
                    key = self.get_key()
                    if key in KEY_MAPPINGS:
                        lin, ang = KEY_MAPPINGS[key]
                        self.current_linear_speed = lin * self.max_linear_speed
                        self.current_angular_speed = ang * self.max_angular_speed
                    elif key == 't':
                        self.max_linear_speed = min(self.max_linear_speed + self.speed_increment, 5.0)
                    elif key == 'g':
                        self.max_linear_speed = max(self.max_linear_speed - self.speed_increment, 0.0)
                    elif key == 'r':
                        self.max_angular_speed = min(self.max_angular_speed + self.speed_increment, 3.0)
                    elif key == 'f':
                        self.max_angular_speed = max(self.max_angular_speed - self.speed_increment, 0.0)
                    elif key == ' ':
                        self.current_linear_speed = 0.0
                        self.current_angular_speed = 0.0
                    elif key == '\x03':  # CTRL+C
                        break
                    self.print_usage()
                # Her durumda komutları publish et
                if self.control_method == 'cmd_vel':
                    twist = Twist()
                    twist.linear.x = self.current_linear_speed
                    twist.angular.z = self.current_angular_speed
                    self.cmd_vel_pub.publish(twist)
                else:
                    front_left, front_right, rear_left, rear_right = self.calculate_ackermann_steering(
                        self.current_linear_speed, self.current_angular_speed
                    )
                    front_steer = Float64MultiArray()
                    front_steer.data = [front_left, front_right]
                    rear_steer = Float64MultiArray()
                    rear_steer.data = [rear_left, rear_right]
                    wheel_speed = Float64MultiArray()
                    wheel_speed.data = [self.current_linear_speed] * 3  # 3 left, 3 right
                    self.front_steering_pub.publish(front_steer)
                    self.rear_steering_pub.publish(rear_steer)
                    self.left_wheels_pub.publish(wheel_speed)
                    self.right_wheels_pub.publish(wheel_speed)
                time.sleep(1.0 / rate)
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 