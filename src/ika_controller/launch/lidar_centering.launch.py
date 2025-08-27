#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('use_sim_time', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('scan_topic', default_value='/scan'))
    declared_arguments.append(DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'))
    declared_arguments.append(DeclareLaunchArgument('forward_speed', default_value='0.3'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    forward_speed = LaunchConfiguration('forward_speed')

    lidar_centering_node = Node(
        package='ika_controller',
        executable='lidar_centering',
        name='lidar_centering',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'forward_speed': forward_speed,
        }]
    )

    return LaunchDescription(declared_arguments + [lidar_centering_node])
