from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_description = LaunchDescription()

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    launch_description.add_action(joint_state_broadcaster_spawner)
    launch_description.add_action(simple_controller_broadcaster_spawner)

    return launch_description