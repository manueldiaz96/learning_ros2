from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    launch_description = LaunchDescription()

    controller_params_file = os.path.join(
        get_package_share_directory('jackal_playground'),
        'config',
        'jackal_controllers.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.098",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.37559",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # Load controller parameters
    robot_controller_params = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params_file],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jackal_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )

    delay_wheel_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[wheel_controller_spawner],
        )
    )

    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(wheel_radius_arg)
    launch_description.add_action(wheel_separation_arg)
    launch_description.add_action(robot_controller_params)  # Add the controller parameters
    launch_description.add_action(joint_state_broadcaster_spawner)
    launch_description.add_action(delay_wheel_controller)

    return launch_description