#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    launch_description = LaunchDescription()

    jackal_description = get_package_share_directory("jackal_playground")

    ros_distro = os.environ["ROS_DISTRO"]

    is_ignition = "True" if ros_distro == "humble" else "False"

    urdf_model = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("jackal_playground"), "urdf", "jackal.urdf.xacro"),
        description= "Path to URDF Xacro model"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(jackal_description).parent.resolve())
            ]
        )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", " empty.sdf"]
                    )
                ]
             )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "jackal"],
    )

    

    launch_description.add_action(urdf_model)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(gazebo_resource_path)
    launch_description.add_action(gazebo)
    launch_description.add_action(gz_spawn_entity)

    return launch_description