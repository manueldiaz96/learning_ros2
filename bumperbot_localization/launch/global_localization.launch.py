from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
                                             "use_sim_time", 
                                             default_value="true"
                                            )
    
    map_name_arg = DeclareLaunchArgument(
                                         "map_name", 
                                         default_value="small_house"
                                        )
    
    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(get_package_share_directory("bumperbot_localization"), "config", "amcl.yaml")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    amcl_config = LaunchConfiguration("amcl_config")
    
    lifecycle_nodes = ["map_server", "amcl"]

    map_path = PathJoinSubstitution([
        get_package_share_directory("bumperbot_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])

    nav2_map_server = Node(package="nav2_map_server",
                           executable="map_server",
                           output="screen",
                           parameters=[
                               {"yaml_filename": map_path},
                               {"use_sim_time": use_sim_time}
                           ])
    


    nav2_lifecycle_manager = Node(package="nav2_lifecycle_manager",
                                  executable="lifecycle_manager",
                                  name="lifecycle_manager_localization",
                                  output="screen",
                                  parameters=[
                                      {"node_names": lifecycle_nodes},
                                      {"use_sim_time": use_sim_time},
                                      {"autostart": True}
                                  ]
                                )
    

    nav2_amcl = Node(package="nav2_amcl",
                     executable="amcl",
                     output="screen",
                     parameters=[
                                  amcl_config,
                                  {"use_sim_time":use_sim_time}
                                ]
                    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_lifecycle_manager,
        nav2_amcl
    ])