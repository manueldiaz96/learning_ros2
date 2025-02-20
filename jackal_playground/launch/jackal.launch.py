from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    launch_description = LaunchDescription()

    urdf_model = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("jackal_playground"), "urdf", "jackal_simple.urdf.xacro"),
        description= "Path to URDF Xacro model"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description" : robot_description}]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    launch_description.add_action(urdf_model)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(rviz_node)

    return launch_description