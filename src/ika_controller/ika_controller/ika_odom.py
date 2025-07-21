#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import numpy as np
import math

class IkaOdomNode(Node):
    def __init__(self):
        super().__init__('ika_odom_node')

        # Parameters
        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('track_width', 0.8)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # Joint names
        self.drive_joints = [
            'sol_on_teker', 'sol_orta_teker', 'sol_arka_teker',
            'sag_on_teker', 'sag_orta_teker', 'sag_arka_teker'
        ]
        self.steer_joints = [
            'sol_on_360', 'sag_on_360', 'sol_arka_360', 'sag_arka_360'
        ]

        # State
        self.last_joint_state = None
        self.last_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vth = 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 50)

        self.get_logger().info('IKA Odom node started!')

    def joint_state_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = now
            self.last_joint_state = msg
            return
        dt = now - self.last_time
        if dt <= 0.0:
            return

        # Get joint positions and velocities
        joint_pos = dict(zip(msg.name, msg.position))
        joint_vel = dict(zip(msg.name, msg.velocity))

        # Steer angles (radians)
        steer_angles = [joint_pos.get(j, 0.0) for j in self.steer_joints]
        # Drive wheel velocities (rad/s)
        drive_vels = [joint_vel.get(j, 0.0) for j in self.drive_joints]

        # Her teker için hız vektörünü hesapla
        wheel_coords = [
            [ self.wheelbase/2,  self.track_width/2],  # sol_on_teker (LF)
            [ 0.0,              self.track_width/2],  # sol_orta_teker (LM)
            [-self.wheelbase/2, self.track_width/2],  # sol_arka_teker (LR)
            [ self.wheelbase/2, -self.track_width/2], # sag_on_teker (RF)
            [ 0.0,             -self.track_width/2],  # sag_orta_teker (RM)
            [-self.wheelbase/2,-self.track_width/2],  # sag_arka_teker (RR)
        ]
        # Steer açıları: [LF, RF, LR, RR]
        steer_map = [0, 3, 2, 1]  # [LF, RF, LR, RR] indexleri
        wheel_steer = [steer_angles[0], steer_angles[1], steer_angles[2], steer_angles[3], 0.0, 0.0]
        # Orta tekerler steer yapmıyor, steer açıları 0

        # Her tekerin hız vektörünü hesapla
        vx_list = []
        vy_list = []
        for i in range(6):
            v = drive_vels[i] * self.wheel_radius  # m/s
            steer = wheel_steer[i] if i < 4 else 0.0
            vx_list.append(v * math.cos(steer))
            vy_list.append(v * math.sin(steer))

        # Ortalama hızları al
        vx = np.mean(vx_list)
        vy = np.mean(vy_list)

        # Yaw rate tahmini (ön ve arka steer ile)
        # (vx sağ/sol teker farkı ile de yapılabilir)
        vth = 0.0
        if abs(self.wheelbase) > 1e-3:
            # Ön ve arka steer ortalaması ile dönme yarıçapı
            avg_steer = (steer_angles[0] - steer_angles[1] + steer_angles[2] - steer_angles[3]) / 4.0
            if abs(avg_steer) > 1e-3:
                turning_radius = self.wheelbase / math.tan(avg_steer)
                vth = vx / turning_radius
            else:
                vth = 0.0

        # Pozisyonu güncelle
        delta_x = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        delta_y = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        delta_yaw = vth * dt
        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Odometry mesajı oluştur
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth
        self.odom_pub.publish(odom_msg)

        # TF yayınla
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_link_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        self.last_time = now
        self.last_joint_state = msg


def main(args=None):
    rclpy.init(args=args)
    node = IkaOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 