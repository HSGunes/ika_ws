#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Start ika controller node
    ika_controller_node = Node(
        package='ika_controller',
        executable='ika_controller',
        name='ika_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_velocity_max': 2.0,
            'angular_velocity_max': 1.5,
            'linear_acceleration_max': 1.0,
            'angular_acceleration_max': 1.0,
            'wheelbase': 1.2,
            'track_width': 0.8,
            'wheel_radius': 0.15,
        }],
    )

    # Start ika bridge node
    ika_bridge_node = Node(
        package='ika_controller',
        executable='ika_bridge',
        name='ika_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheelbase': 1.2,
            'track_width': 0.8,
            'wheel_radius': 0.15,
            'max_steering_angle': 45.0,
        }],
    )

    ground_truth_odom_node = Node(
        package='ika_controller',
        executable='ika_ground_truth_odom',
        name='ika_ground_truth_odom_node',
        output='screen',
        parameters=[
            {'model_name': 'ika',
             'odom_frame': 'odom',
             'base_link_frame': 'base_link',
             'publish_tf': True,
             'odom_topic': '/odom_ground_truth'}
        ]
    )

    nodes = [
        ika_controller_node,
        ika_bridge_node,
    ]

    ld = LaunchDescription()
    ld.add_action(ground_truth_odom_node)
    return ld 