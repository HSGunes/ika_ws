#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for IKA Arduino connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='170',
        description='Maximum linear speed (PWM value 0-170 for IKA)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='60.0',
        description='Maximum angular speed (degrees)'
    )
    
    # Scaling parametreleri
    linear_scale_factor_arg = DeclareLaunchArgument(
        'linear_scale_factor',
        default_value='85.0',
        description='Linear velocity scaling factor (PWM per m/s) - 2.0 m/s = 170 PWM'
    )
    
    angular_scale_factor_arg = DeclareLaunchArgument(
        'angular_scale_factor',
        default_value='60.0',
        description='Angular velocity scaling factor (degrees per rad/s)'
    )
    
    min_pwm_threshold_arg = DeclareLaunchArgument(
        'min_pwm_threshold',
        default_value='10',
        description='Minimum PWM threshold'
    )
    
    max_cmd_linear_arg = DeclareLaunchArgument(
        'max_cmd_linear',
        default_value='2.0',
        description='Maximum accepted linear command (m/s)'
    )
    
    max_cmd_angular_arg = DeclareLaunchArgument(
        'max_cmd_angular',
        default_value='1.5',
        description='Maximum accepted angular command (rad/s)'
    )
    
    # IKA özel parametreleri
    enable_steering_arg = DeclareLaunchArgument(
        'enable_steering',
        default_value='true',
        description='Enable steering control for IKA'
    )
    
    steering_sensitivity_arg = DeclareLaunchArgument(
        'steering_sensitivity',
        default_value='1.0',
        description='Steering sensitivity multiplier'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.8',
        description='IKA wheel base (m)'
    )
    
    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='0.78',
        description='IKA track width (m)'
    )
    
    # IKA Serial controller node
    ika_serial_controller_node = Node(
        package='ika_serial_control',
        executable='ika_serial_controller',
        name='ika_serial_controller',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'linear_scale_factor': LaunchConfiguration('linear_scale_factor'),
            'angular_scale_factor': LaunchConfiguration('angular_scale_factor'),
            'min_pwm_threshold': LaunchConfiguration('min_pwm_threshold'),
            'max_cmd_linear': LaunchConfiguration('max_cmd_linear'),
            'max_cmd_angular': LaunchConfiguration('max_cmd_angular'),
            'enable_steering': LaunchConfiguration('enable_steering'),
            'steering_sensitivity': LaunchConfiguration('steering_sensitivity'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'track_width': LaunchConfiguration('track_width'),
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('ika_status', '/ika_status'),
            ('current_steer_angle', '/current_steer_angle'),
            ('current_speed', '/current_speed'),
            ('wheel_speeds', '/wheel_speeds'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        linear_scale_factor_arg,
        angular_scale_factor_arg,
        min_pwm_threshold_arg,
        max_cmd_linear_arg,
        max_cmd_angular_arg,
        enable_steering_arg,
        steering_sensitivity_arg,
        wheel_base_arg,
        track_width_arg,
        ika_serial_controller_node,
    ])
