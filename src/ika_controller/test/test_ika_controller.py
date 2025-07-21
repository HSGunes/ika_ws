#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time


class TestIkaController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_ika_controller')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.front_steering_sub = self.node.create_subscription(
            Float64MultiArray, '/front_steering_controller/commands', self.front_steering_callback, 10)
        self.rear_steering_sub = self.node.create_subscription(
            Float64MultiArray, '/rear_steering_controller/commands', self.rear_steering_callback, 10)
        self.left_wheels_sub = self.node.create_subscription(
            Float64MultiArray, '/left_wheels_controller/commands', self.left_wheels_callback, 10)
        self.right_wheels_sub = self.node.create_subscription(
            Float64MultiArray, '/right_wheels_controller/commands', self.right_wheels_callback, 10)
        
        self.front_steering_received = False
        self.rear_steering_received = False
        self.left_wheels_received = False
        self.right_wheels_received = False

    def tearDown(self):
        self.node.destroy_node()

    def front_steering_callback(self, msg):
        self.front_steering_received = True
        self.front_steering_data = msg.data

    def rear_steering_callback(self, msg):
        self.rear_steering_received = True
        self.rear_steering_data = msg.data

    def left_wheels_callback(self, msg):
        self.left_wheels_received = True
        self.left_wheels_data = msg.data

    def right_wheels_callback(self, msg):
        self.right_wheels_received = True
        self.right_wheels_data = msg.data

    def test_straight_forward_motion(self):
        # Test straight forward motion
        cmd_vel = Twist()
        cmd_vel.linear.x = 1.0
        cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Wait for messages
        timeout = 2.0
        start_time = time.time()
        while (not self.front_steering_received or not self.rear_steering_received or 
               not self.left_wheels_received or not self.right_wheels_received) and \
              (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Assert that all messages were received
        self.assertTrue(self.front_steering_received, "Front steering message not received")
        self.assertTrue(self.rear_steering_received, "Rear steering message not received")
        self.assertTrue(self.left_wheels_received, "Left wheels message not received")
        self.assertTrue(self.right_wheels_received, "Right wheels message not received")
        
        # Check that steering angles are zero for straight motion
        self.assertAlmostEqual(self.front_steering_data[0], 0.0, places=2)
        self.assertAlmostEqual(self.front_steering_data[1], 0.0, places=2)
        self.assertAlmostEqual(self.rear_steering_data[0], 0.0, places=2)
        self.assertAlmostEqual(self.rear_steering_data[1], 0.0, places=2)
        
        # Check that wheel velocities are positive for forward motion
        for vel in self.left_wheels_data:
            self.assertGreater(vel, 0.0)
        for vel in self.right_wheels_data:
            self.assertGreater(vel, 0.0)


if __name__ == '__main__':
    unittest.main() 