#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import numpy as np
from tf_transformations import quaternion_from_euler
import math

class RoverOdometryNode(Node):
    def __init__(self):
        super().__init__('rover_odometry_node')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        # Robot parameters (meters)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.165),  # 165mm wheel radius from STL
                ('wheel_base_length', 1.0),  # Distance between front and rear axles
                ('wheel_base_width', 0.8),   # Distance between left and right wheels
                ('publish_tf', True),
                ('base_frame_id', 'base_link'),
                ('odom_frame_id', 'odom')
            ]
        )
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_length = self.get_parameter('wheel_base_length').value
        self.wheel_base_width = self.get_parameter('wheel_base_width').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Previous wheel positions for velocity calculation
        self.prev_wheel_positions = {}
        self.prev_steering_angles = {}
        
        # Joint names
        self.drive_joints = [
            'sol_on_teker',      # front left drive
            'sol_orta_teker',    # middle left drive  
            'sol_arka_teker',    # rear left drive
            'sag_on_teker',      # front right drive
            'sag_orta_teker',    # middle right drive
            'sag_arka_teker'     # rear right drive
        ]
        
        self.steering_joints = [
            'sol_on_360',        # front left steering
            'sag_on_360',        # front right steering
            'sol_arka_360',      # rear left steering
            'sag_arka_360'       # rear right steering
        ]
        
        self.get_logger().info('Rover odometry node initialized')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Wheelbase: {self.wheel_base_length}m x {self.wheel_base_width}m')

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.001:
            return
        # Extract wheel velocities and steering angles
        wheel_velocities = {}
        steering_angles = {}
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.drive_joints:
                wheel_velocities[joint_name] = msg.velocity[i]
            elif joint_name in self.steering_joints:
                steering_angles[joint_name] = msg.position[i]
        # Calculate odometry using 6-wheel rover kinematics
        linear_vel, angular_vel = self.calculate_rover_velocity(
            wheel_velocities, steering_angles
        )
        # Deadband uygula (gürültü engelle)
        if abs(linear_vel) < 0.005:
            linear_vel = 0.0
        if abs(angular_vel) < 0.005:
            angular_vel = 0.0
        # (İsteğe bağlı) Low-pass filter ile yumuşatma
        alpha = 0.2
        if hasattr(self, 'last_linear_vel'):
            linear_vel = alpha * linear_vel + (1 - alpha) * self.last_linear_vel
            angular_vel = alpha * angular_vel + (1 - alpha) * self.last_angular_vel
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel
        # Update pose
        self.update_odometry(linear_vel, angular_vel, dt, current_time)
        self.last_time = current_time

    def calculate_rover_velocity(self, wheel_velocities, steering_angles):
        required_drives = ['sol_on_teker', 'sol_arka_teker', 'sag_on_teker', 'sag_arka_teker']
        required_steers = ['sol_on_360', 'sag_on_360', 'sol_arka_360', 'sag_arka_360']
        if not all(joint in wheel_velocities for joint in required_drives):
            return 0.0, 0.0
        if not all(joint in steering_angles for joint in required_steers):
            return 0.0, 0.0
        wheel_positions = {
            'sol_on_360': (self.wheel_base_length/2, self.wheel_base_width/2),      # front left
            'sag_on_360': (self.wheel_base_length/2, -self.wheel_base_width/2),     # front right  
            'sol_arka_360': (-self.wheel_base_length/2, self.wheel_base_width/2),   # rear left
            'sag_arka_360': (-self.wheel_base_length/2, -self.wheel_base_width/2)   # rear right
        }
        wheel_linear_velocities = {
            'sol_on_teker': wheel_velocities['sol_on_teker'] * self.wheel_radius,
            'sag_on_teker': wheel_velocities['sag_on_teker'] * self.wheel_radius,
            'sol_arka_teker': wheel_velocities['sol_arka_teker'] * self.wheel_radius,
            'sag_arka_teker': wheel_velocities['sag_arka_teker'] * self.wheel_radius
        }
        A = []
        b = []
        steering_mapping = {
            'sol_on_360': 'sol_on_teker',
            'sag_on_360': 'sag_on_teker', 
            'sol_arka_360': 'sol_arka_teker',
            'sag_arka_360': 'sag_arka_teker'
        }
        for steer_joint, drive_joint in steering_mapping.items():
            if steer_joint in steering_angles and drive_joint in wheel_linear_velocities:
                x_w, y_w = wheel_positions[steer_joint]
                steering_angle = steering_angles[steer_joint]
                v_wheel = wheel_linear_velocities[drive_joint]
                v_wheel_x = v_wheel * math.cos(steering_angle)
                v_wheel_y = v_wheel * math.sin(steering_angle)
                A.append([1, 0, -y_w])
                b.append(v_wheel_x)
                A.append([0, 1, x_w])
                b.append(v_wheel_y)
        if len(A) < 3:
            return 0.0, 0.0
        try:
            A = np.array(A)
            b = np.array(b)
            result = np.linalg.lstsq(A, b, rcond=None)[0]
            vx, vy, omega = result[0], result[1], result[2]
            linear_velocity = math.sqrt(vx*vx + vy*vy)
            if vx < 0:
                linear_velocity = -linear_velocity
            angular_velocity = omega
            return linear_velocity, angular_velocity
        except np.linalg.LinAlgError:
            self.get_logger().warn("Failed to solve odometry equations")
            return 0.0, 0.0

    def update_odometry(self, linear_vel, angular_vel, dt, current_time):
        self.theta += angular_vel * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        theta_mid = self.theta - 0.5 * angular_vel * dt
        self.x += linear_vel * math.cos(theta_mid) * dt
        self.y += linear_vel * math.sin(theta_mid) * dt
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        odom_msg.pose.covariance[0] = 0.1
        odom_msg.pose.covariance[7] = 0.1
        odom_msg.pose.covariance[35] = 0.1
        odom_msg.twist.covariance[0] = 0.1
        odom_msg.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom_msg)
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = q[0]
            tf_msg.transform.rotation.y = q[1]
            tf_msg.transform.rotation.z = q[2]
            tf_msg.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = RoverOdometryNode()
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 