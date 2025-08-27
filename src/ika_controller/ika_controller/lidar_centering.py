#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarCenteringNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_centering_node')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('kp_lateral', 1.0)
        self.declare_parameter('kp_heading', 1.2)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('side_sector_deg', 40.0)  # sector half-width around left/right
        self.declare_parameter('front_sector_deg', 30.0)  # slow down/avoid frontal obstacles
        self.declare_parameter('front_stop_distance', 0.6)
        self.declare_parameter('front_slow_distance', 1.0)
        self.declare_parameter('max_angular_speed', 1.2)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.kp_lateral = float(self.get_parameter('kp_lateral').value)
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.side_sector_deg = float(self.get_parameter('side_sector_deg').value)
        self.front_sector_deg = float(self.get_parameter('front_sector_deg').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_slow_distance = float(self.get_parameter('front_slow_distance').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f"Lidar centering started. Sub: {scan_topic}, Pub: {cmd_vel_topic}")

    def scan_callback(self, scan: LaserScan) -> None:
        if not scan.ranges:
            return
        ranges = list(scan.ranges)
        # sanitize ranges
        ranges = [self._clamp_range(r, scan.range_min, scan.range_max) for r in ranges]

        # Compute sectors indices
        side_half = math.radians(self.side_sector_deg)
        front_half = math.radians(self.front_sector_deg)

        # Angles per index
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        count = len(ranges)

        def indices_for_angle_window(center_angle: float, half_width: float) -> List[int]:
            start_ang = center_angle - half_width
            end_ang = center_angle + half_width
            start_idx = max(0, int((start_ang - angle_min) / angle_inc))
            end_idx = min(count - 1, int((end_ang - angle_min) / angle_inc))
            if end_idx < start_idx:
                start_idx, end_idx = end_idx, start_idx
            return list(range(start_idx, end_idx + 1))

        # Left ~ +90deg, Right ~ -90deg, Front ~ 0deg in laser frame (assuming base_link forward is 0)
        left_idx = indices_for_angle_window(math.pi/2, side_half)
        right_idx = indices_for_angle_window(-math.pi/2, side_half)
        front_idx = indices_for_angle_window(0.0, front_half)

        left_dist = self._robust_mean([ranges[i] for i in left_idx])
        right_dist = self._robust_mean([ranges[i] for i in right_idx])
        front_dist = self._robust_min([ranges[i] for i in front_idx])

        # Control logic
        cmd = Twist()

        # Forward speed modulation by front obstacle
        if front_dist is not None and front_dist < self.front_stop_distance:
            cmd.linear.x = 0.0
        elif front_dist is not None and front_dist < self.front_slow_distance:
            scale = (front_dist - self.front_stop_distance) / max(1e-3, (self.front_slow_distance - self.front_stop_distance))
            scale = max(0.0, min(1.0, scale))
            cmd.linear.x = self.forward_speed * scale
        else:
            cmd.linear.x = self.forward_speed

        # Angular control: center between left/right walls
        ang_cmd = 0.0
        if left_dist is not None and right_dist is not None:
            # positive ang_cmd -> turn left
            lateral_error = right_dist - left_dist  # if right is far, drift right => turn left
            ang_cmd += self.kp_lateral * lateral_error
        elif left_dist is not None:
            # only left seen -> keep some clearance by turning slightly right if too close
            ang_cmd -= self.kp_lateral * (self._desired_side_clearance() - left_dist)
        elif right_dist is not None:
            # only right seen -> keep clearance by turning slightly left if too close
            ang_cmd += self.kp_lateral * (self._desired_side_clearance() - right_dist)

        # Heading correction using front sector asymmetry
        if front_idx:
            # compute weighted heading bias from front points
            heading_bias = self._front_heading_bias(scan, ranges, front_idx)
            ang_cmd += self.kp_heading * heading_bias

        # Saturate angular speed
        ang_cmd = max(-self.max_angular_speed, min(self.max_angular_speed, ang_cmd))
        cmd.angular.z = ang_cmd

        self.cmd_pub.publish(cmd)

    def _front_heading_bias(self, scan: LaserScan, ranges: List[float], idxs: List[int]) -> float:
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        num = 0.0
        den = 0.0
        for i in idxs:
            ang = angle_min + i * angle_inc
            r = ranges[i]
            if math.isfinite(r):
                weight = 1.0 / max(r, 1e-3)
                num += weight * math.sin(ang)  # left(+)/right(-)
                den += weight
        if den < 1e-6:
            return 0.0
        return num / den

    def _desired_side_clearance(self) -> float:
        return 0.6

    def _clamp_range(self, r: float, rmin: float, rmax: float) -> float:
        if r is None or math.isnan(r) or math.isinf(r):
            return float('inf')
        r = max(self.min_range if self.min_range > 0 else rmin, min(self.max_range if self.max_range > 0 else rmax, r))
        return r

    def _robust_mean(self, vals: List[float]) -> Optional[float]:
        finite = [v for v in vals if math.isfinite(v)]
        if not finite:
            return None
        finite.sort()
        k = max(1, int(0.1 * len(finite)))
        trimmed = finite[k:-k] if len(finite) > 2 * k else finite
        return sum(trimmed) / len(trimmed)

    def _robust_min(self, vals: List[float]) -> Optional[float]:
        finite = [v for v in vals if math.isfinite(v)]
        if not finite:
            return None
        return min(finite)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarCenteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
