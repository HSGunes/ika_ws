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
    rf2o_dir = get_package_share_directory('rf2o_laser_odometry')
    zed_dir = get_package_share_directory('zed_wrapper')
    
    # Konfigürasyon dosyalarının yolları
    ekf_config = os.path.join(ika_nav2_dir, 'config', 'ekf.yaml')
    zed_config = os.path.join(ika_nav2_dir, 'config', 'zed2i_no_area.yaml')
    # Launch argümanları
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    publish_tf_rf2o = LaunchConfiguration('publish_tf_rf2o', default='false')
    publish_tf_zed = LaunchConfiguration('publish_tf_zed', default='false')
    
    return LaunchDescription([
        # Launch argümanları
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'publish_tf_rf2o',
            default_value='false',
            description='RF2O TF yayını (EKF kullanırken false olmalı)'
        ),
        DeclareLaunchArgument(
            'publish_tf_zed',
            default_value='false',
            description='ZED TF yayını (EKF kullanırken false olmalı)'
        ),
        
        # RF2O Lazer Odometri Düğümü
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rf2o_dir, 'launch', 'rf2o_laser_odometry.launch.py')
            ),
            launch_arguments={
                'publish_tf': publish_tf_rf2o,
                'odom_topic_name': '/odom_rf2o',
                'use_sim_time': use_sim_time
            }.items()
        ),
        
        # ZED Kamera Düğümü
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_dir, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'camera_model': 'zed2i',
                'publish_tf': publish_tf_zed,
                'use_sim_time': use_sim_time,
                'ros_params_override_path': zed_config
            }.items()
        ),
        
        # EKF (robot_localization) Düğümü
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
