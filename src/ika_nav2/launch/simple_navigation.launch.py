#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='ika_nav2').find('ika_nav2')
    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': 'false',
        'yaml_filename': PathJoinSubstitution([pkg_share, 'maps', 'map.yaml'])
    }

    configured_params = RewrittenYaml(
        source_file=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_dir_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Use composed bringup if True, use separate launch files if False')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of container that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Include TF Setup Launch
    tf_setup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('ika_mapping').find('ika_mapping'), 'launch', 'tf_setup.launch.py')]),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items())

    # Include RF2O Laser Odometry Launch
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('rf2o_laser_odometry').find('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')]))

    # Map Server Node
    map_server_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_server', 
             PathJoinSubstitution([pkg_share, 'maps', 'map.yaml']),
             '--ros-args', '-p', 'use_sim_time:=false'],
        output='screen'
    )

    # Nav2 Nodes (manual launch instead of bringup)
    amcl_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_amcl', 'amcl', 
             '--ros-args', '-p', 'use_sim_time:=false',
             '--params-file', os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        output='screen'
    )
    
    controller_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_controller', 'controller_server', 
             '--ros-args', '-p', 'use_sim_time:=false',
             '--params-file', os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        output='screen'
    )
    
    planner_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_planner', 'planner_server', 
             '--ros-args', '-p', 'use_sim_time:=false',
             '--params-file', os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        output='screen'
    )
    
    bt_navigator_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_bt_navigator', 'bt_navigator', 
             '--ros-args', '-p', 'use_sim_time:=false',
             '--params-file', os.path.join(pkg_share, 'config', 'nav2_params.yaml')],
        output='screen'
    )
    
    lifecycle_manager_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', 
             '--ros-args', '-p', 'node_names:=[amcl,controller_server,planner_server,bt_navigator]', 
             '-p', 'autostart:=true'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_dir_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch TF setup, RF2O odometry, map server, and navigation
    ld.add_action(tf_setup_launch)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(controller_node)
    ld.add_action(planner_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(lifecycle_manager_node)

    return ld
