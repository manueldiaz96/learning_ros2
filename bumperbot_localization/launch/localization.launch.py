#!/usr/bin/python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_description = LaunchDescription()

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf2.yaml")],
    )

    imu_republisher = Node(
        package="bumperbot_localization",
        executable="imu_republisher"
    )

    launch_description.add_action(static_transform_publisher)
    launch_description.add_action(robot_localization)
    launch_description.add_action(imu_republisher)
    
    return launch_description