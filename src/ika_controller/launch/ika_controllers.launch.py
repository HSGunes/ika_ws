#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='ika_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='ika.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='diff_drive_controller',
            description='Robot controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers \
        configuration have to be updated.',
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controller = LaunchConfiguration('robot_controller')
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ),
            ' ',
            'prefix:=',
            prefix,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Get controllers file
    controllers_file_path = PathJoinSubstitution(
        [
            FindPackageShare('ika_controller'),
            'config',
            controllers_file,
        ]
    )

    # Load controllers
    load_controllers = []
    for controller in [
        'joint_state_broadcaster',
        'front_steering_controller',
        'rear_steering_controller',
        'left_wheels_controller',
        'right_wheels_controller',
    ]:
        load_controllers.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller, '-c', '/controller_manager'],
                output='screen',
            )
        )

    # Start controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file_path],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # Start robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
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

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=load_controllers,
        )
    )

    nodes = [
        controller_manager_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        ika_controller_node,
        ika_bridge_node,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes) 