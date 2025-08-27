#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ika_description"), 
                "urdf", 
                "ika.xacro"
            ]),
            " ",
            "use_sim_time:=",
            use_sim_time,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher - TF dönüşümlerini yayınlar
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher - Joint state'leri yayınlar
    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Static TF Publisher - map to odom dönüşümü
    tf_static_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Gerçek lidar için ek static transform (eğer gerekirse)
    # Bu, gerçek lidar'ın konumunu base_link'e göre tanımlar
    lidar_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_publisher",
        output="screen",
        arguments=["0", "0", "0.5", "0", "0", "0", "base_link", "laser"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes = [
        robot_state_pub_node,
        joint_state_pub_node,
        tf_static_publisher,
        lidar_tf_publisher,
    ]

    return LaunchDescription(declared_arguments + nodes)
