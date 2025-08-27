#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



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

    # Get the launch directory
    pkg_share = FindPackageShare(package="ika_mapping").find("ika_mapping")
    ika_description_pkg_share = FindPackageShare(package="ika_description").find("ika_description")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([ika_description_pkg_share, "urdf", "ika.xacro"]),
            " ",
            "use_sim_time:=",
            use_sim_time,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher
    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )



    # SLAM Toolbox
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "slam_toolbox_params.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
    )

    # TF Static Publisher for map to odom
    tf_static_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes = [
        robot_state_pub_node,
        joint_state_pub_node,
        tf_static_publisher,
        slam_toolbox_node,
     
    ]
 

    return LaunchDescription(declared_arguments + nodes) 