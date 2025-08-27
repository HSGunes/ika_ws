#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ika_description'), 'urdf', 'ika.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Get controllers file
    controllers_file_path = PathJoinSubstitution(
        [
            FindPackageShare('ika_controller'),
            'config',
            'controllers.yaml',
        ]
    )

    # Start controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file_path],
        output='both',
    )

    # Start robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start static transform publisher for map to odom (if needed)
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     output='screen',
    # )

    # Load and start controllers with delays
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    front_steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['front_steering_controller', '-c', '/controller_manager'],
        output='screen',
    )

    rear_steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rear_steering_controller', '-c', '/controller_manager'],
        output='screen',
    )

    left_wheels_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_wheels_controller', '-c', '/controller_manager'],
        output='screen',
    )

    right_wheels_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_wheels_controller', '-c', '/controller_manager'],
        output='screen',
    )

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


    # Start joint_state_publisher for static joints (visual tf completeness)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'publish_default_positions': True}]
    # )

 
    # Add delays for controller loading
    delayed_front_steering = TimerAction(
        period=2.0,
        actions=[front_steering_spawner]
    )

    delayed_rear_steering = TimerAction(
        period=4.0,
        actions=[rear_steering_spawner]
    )

    delayed_left_wheels = TimerAction(
        period=6.0,
        actions=[left_wheels_spawner]
    )

    delayed_right_wheels = TimerAction(
        period=8.0,
        actions=[right_wheels_spawner]
    )

    nodes = [
        controller_manager_node,
        robot_state_pub_node,
        # static_tf_node,
        joint_state_spawner,
        delayed_front_steering,
        delayed_rear_steering,
        delayed_left_wheels,
        delayed_right_wheels,
        ika_controller_node,
        ika_bridge_node,
        
    ]

    return LaunchDescription(declared_arguments + nodes) 