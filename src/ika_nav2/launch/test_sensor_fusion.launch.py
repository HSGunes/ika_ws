#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paket dizinlerini al
    ika_nav2_dir = get_package_share_directory('ika_nav2')
    
    # Konfigürasyon dosyalarının yolları
    ekf_config = os.path.join(ika_nav2_dir, 'config', 'ekf.yaml')
    
    # Launch argümanları
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch argümanları
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # EKF (robot_localization) Düğümü - Sadece EKF'yi test etmek için
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odom'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
    ])
