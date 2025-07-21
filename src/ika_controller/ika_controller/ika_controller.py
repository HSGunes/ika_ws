#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import time
import math

class IkaControllerNode(Node):
    def __init__(self):
        super().__init__('ika_controller_node')

        # Declare parameters
        self.declare_parameter('linear_velocity_max', 2.0)
        self.declare_parameter('angular_velocity_max', 1.5)
        self.declare_parameter('linear_acceleration_max', 1.0)
        self.declare_parameter('angular_acceleration_max', 1.0)
        self.declare_parameter('twist_topic', '/cmd_vel')
        self.declare_parameter('ika_control_topic', '/ika_control/reference')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('wheelbase', 1.2)  # Distance between front and rear axles
        self.declare_parameter('track_width', 0.8)  # Distance between left and right wheels
        self.declare_parameter('wheel_radius', 0.15)  # Wheel radius in meters

        # Get parameters
        self.linear_velocity_max = self.get_parameter('linear_velocity_max').get_parameter_value().double_value
        self.angular_velocity_max = self.get_parameter('angular_velocity_max').get_parameter_value().double_value
        self.linear_acceleration_max = self.get_parameter('linear_acceleration_max').get_parameter_value().double_value
        self.angular_acceleration_max = self.get_parameter('angular_acceleration_max').get_parameter_value().double_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        ika_control_topic = self.get_parameter('ika_control_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # State variables
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_cmd_vel_time = self.get_clock().now().nanoseconds / 1e9

        # Publisher and Subscriber
        self.publisher_ = self.create_publisher(TwistStamped, ika_control_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            twist_topic,
            self.cmd_vel_callback,
            10)
            
        # Create a timer for publishing with ramping
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_twist)
        
        self.get_logger().info("Ika Controller Node has been started.")
        self.get_logger().info(f"Subscribing to: {twist_topic}")
        self.get_logger().info(f"Publishing to: {ika_control_topic}")
        self.get_logger().info(f"Linear vel max: {self.linear_velocity_max} m/s, accel max: {self.linear_acceleration_max} m/s^2")
        self.get_logger().info(f"Angular vel max: {self.angular_velocity_max} rad/s, accel max: {self.angular_acceleration_max} rad/s^2")
        self.get_logger().info(f"Wheelbase: {self.wheelbase}m, Track width: {self.track_width}m, Wheel radius: {self.wheel_radius}m")

    def cmd_vel_callback(self, msg):
        self.target_linear_velocity = msg.linear.x
        self.target_angular_velocity = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now().nanoseconds / 1e9

    def publish_twist(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # If no command is received for 0.5 seconds, set target to zero
        if (current_time - self.last_cmd_vel_time) > 0.5:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

        # Get the timer period (dt)
        dt = self.timer.timer_period_ns / 1e9

        # Clamp target velocities to max limits
        self.target_linear_velocity = max(-self.linear_velocity_max, min(self.linear_velocity_max, self.target_linear_velocity))
        self.target_angular_velocity = max(-self.angular_velocity_max, min(self.angular_velocity_max, self.target_angular_velocity))
        
        # Apply acceleration limits
        delta_linear = self.target_linear_velocity - self.current_linear_velocity
        max_linear_change = self.linear_acceleration_max * dt
        linear_change = max(-max_linear_change, min(max_linear_change, delta_linear))
        self.current_linear_velocity += linear_change

        delta_angular = self.target_angular_velocity - self.current_angular_velocity
        max_angular_change = self.angular_acceleration_max * dt
        angular_change = max(-max_angular_change, min(max_angular_change, delta_angular))
        self.current_angular_velocity += angular_change
        
        # Create and publish the message
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link' 
        twist_stamped_msg.twist.linear.x = self.current_linear_velocity
        twist_stamped_msg.twist.angular.z = self.current_angular_velocity
        self.publisher_.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IkaControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 