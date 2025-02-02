#!/usr/bin/python3
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    c3po_node = Node(
        package="learn_cpp_ros",
        executable="robot_news_publisher",
        name="robot_news_c3po",
        parameters=[
            {"robot_name": "c3po"}
        ]
    )

    giskard_node = Node(
        package="learn_cpp_ros",
        executable="robot_news_publisher",
        name="robot_news_giskard",
        parameters=[
            {"robot_name": "giskard"}
        ]
    )

    bb8_node = Node(
        package="learn_cpp_ros",
        executable="robot_news_publisher",
        name="robot_news_bb8",
        parameters=[
            {"robot_name": "bb8"}
        ]
    )

    daneel_node = Node(
        package="learn_cpp_ros",
        executable="robot_news_publisher",
        name="robot_news_daneel",
        parameters=[
            {"robot_name": "daneel"}
        ]
    )

    lander_node = Node(
        package="learn_cpp_ros",
        executable="robot_news_publisher",
        name="robot_news_lander",
        parameters=[
            {"robot_name": "lander"}
        ]
    )

    smartphone_node = Node(
        package="learn_cpp_ros",
        executable="smartphone",
        name="smartphone",
    )


    ld.add_action(c3po_node)
    ld.add_action(giskard_node)
    ld.add_action(bb8_node)
    ld.add_action(lander_node)
    ld.add_action(daneel_node)
    ld.add_action(smartphone_node)
    return ld