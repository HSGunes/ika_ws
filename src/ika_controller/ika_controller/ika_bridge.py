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

        # Get parameters
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_steering_angle = math.radians(self.get_parameter('max_steering_angle').get_parameter_value().double_value)

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
        # Extract linear and angular velocities
        linear_velocity = msg.twist.linear.x  # m/s
        angular_velocity = msg.twist.angular.z  # rad/s

        # Calculate steering angles for 4-wheel steering
        if abs(linear_velocity) > 0.001:  # Avoid division by zero
            # For 4-wheel steering, we can use different steering strategies
            # Here we implement a simple crab steering (all wheels steer the same angle)
            steering_angle = math.atan(angular_velocity * self.wheelbase / linear_velocity)
        else:
            steering_angle = 0.0

        # Limit steering angle
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

        # Calculate wheel velocities for 6 wheels
        # For differential steering with 6 wheels, we need to consider the track width
        if abs(angular_velocity) > 0.001:
            # Left wheels: slower when turning left, faster when turning right
            left_wheel_velocity = (linear_velocity - angular_velocity * self.track_width / 2.0) / self.wheel_radius
            # Right wheels: faster when turning left, slower when turning right
            right_wheel_velocity = (linear_velocity + angular_velocity * self.track_width / 2.0) / self.wheel_radius
        else:
            # Straight line motion
            wheel_velocity = linear_velocity / self.wheel_radius
            left_wheel_velocity = wheel_velocity
            right_wheel_velocity = wheel_velocity

        # Create steering commands
        front_steering_cmd = Float64MultiArray()
        front_steering_cmd.data = [steering_angle, steering_angle]  # left_front, right_front

        rear_steering_cmd = Float64MultiArray()
        rear_steering_cmd.data = [steering_angle, steering_angle]  # left_rear, right_rear

        # Create velocity commands for 6 wheels (3 left, 3 right)
        left_wheels_cmd = Float64MultiArray()
        left_wheels_cmd.data = [left_wheel_velocity, left_wheel_velocity, left_wheel_velocity]  # front, middle, rear

        right_wheels_cmd = Float64MultiArray()
        right_wheels_cmd.data = [right_wheel_velocity, right_wheel_velocity, right_wheel_velocity]  # front, middle, rear

        # Publish commands
        self.front_steering_publisher.publish(front_steering_cmd)
        self.rear_steering_publisher.publish(rear_steering_cmd)
        self.left_wheels_publisher.publish(left_wheels_cmd)
        self.right_wheels_publisher.publish(right_wheels_cmd)

        # Log for debugging
        if abs(linear_velocity) > 0.001 or abs(angular_velocity) > 0.001:
            self.get_logger().debug(
                f'Linear: {linear_velocity:.3f} m/s, Angular: {angular_velocity:.3f} rad/s, '
                f'Steering: {math.degrees(steering_angle):.1f}°, '
                f'Left vel: {left_wheel_velocity:.3f} rad/s, Right vel: {right_wheel_velocity:.3f} rad/s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = IkaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 