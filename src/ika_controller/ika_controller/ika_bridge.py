#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import math

class IkaBridge(Node):
    def __init__(self):
        super().__init__('ika_bridge')

        # Declare parameters
        self.declare_parameter('wheelbase', 1.2)  # meters - distance between front and rear axles
        self.declare_parameter('track_width', 0.8)  # meters - distance between left and right wheels
        self.declare_parameter('wheel_radius', 0.15)  # meters
        self.declare_parameter('max_steering_angle', 45.0)  # degrees
        self.declare_parameter('velocity_threshold', 0.5)  # m/s, 4WS mod geçişi için

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_steering_angle = math.radians(self.get_parameter('max_steering_angle').get_parameter_value().double_value)
        self.velocity_threshold = self.get_parameter('velocity_threshold').get_parameter_value().double_value

        # Publishers for steering (4 steerable wheels)
        self.front_steering_publisher = self.create_publisher(
            Float64MultiArray, 
            '/front_steering_controller/commands', 
            10
        )
        
        self.rear_steering_publisher = self.create_publisher(
            Float64MultiArray, 
            '/rear_steering_controller/commands', 
            10
        )
        
        # Publishers for velocity (6 drive wheels)
        self.left_wheels_publisher = self.create_publisher(
            Float64MultiArray, 
            '/left_wheels_controller/commands', 
            10
        )
        
        self.right_wheels_publisher = self.create_publisher(
            Float64MultiArray, 
            '/right_wheels_controller/commands', 
            10
        )

        # Subscriber
        self.ika_control_subscriber = self.create_subscription(
            TwistStamped,
            '/ika_control/reference',
            self.ika_control_callback,
            10
        )

        self.get_logger().info('Ika Bridge node started')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m, Track width: {self.track_width}m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m, Max steering angle: {math.degrees(self.max_steering_angle)}°')

    def ika_control_callback(self, msg):
        linear_velocity = msg.twist.linear.x  # m/s
        angular_velocity = msg.twist.angular.z  # rad/s

        # Ackermann steer açısı (ön teker)
        if abs(linear_velocity) > 1e-3 and abs(angular_velocity) > 1e-3:
            front_steering_angle = math.atan(angular_velocity * self.wheelbase / linear_velocity)
        else:
            front_steering_angle = 0.0

        # Deadband uygula
        if abs(front_steering_angle) < 0.01:
            front_steering_angle = 0.0
        rear_steering_angle = -front_steering_angle
        if abs(rear_steering_angle) < 0.01:
            rear_steering_angle = 0.0
        # Limit
        front_steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, front_steering_angle))
        rear_steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, rear_steering_angle))

        # Komutları oluştur
        front_steering_cmd = Float64MultiArray()
        front_steering_cmd.data = [front_steering_angle, front_steering_angle]

        rear_steering_cmd = Float64MultiArray()
        rear_steering_cmd.data = [rear_steering_angle, rear_steering_angle]

        # Teker hızları
        wheel_velocity = linear_velocity / self.wheel_radius
        left_wheel_velocity = wheel_velocity
        right_wheel_velocity = wheel_velocity

        left_wheels_cmd = Float64MultiArray()
        left_wheels_cmd.data = [left_wheel_velocity, left_wheel_velocity, left_wheel_velocity]

        right_wheels_cmd = Float64MultiArray()
        right_wheels_cmd.data = [right_wheel_velocity, right_wheel_velocity, right_wheel_velocity]

        # Publish
        self.front_steering_publisher.publish(front_steering_cmd)
        self.rear_steering_publisher.publish(rear_steering_cmd)
        self.left_wheels_publisher.publish(left_wheels_cmd)
        self.right_wheels_publisher.publish(right_wheels_cmd)

        # Debug log
        self.get_logger().info(
            f"lin: {linear_velocity:.3f}, ang: {angular_velocity:.3f}, front: {front_steering_angle:.3f}, rear: {rear_steering_angle:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IkaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 