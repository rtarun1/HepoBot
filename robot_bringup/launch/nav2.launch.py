import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Map and Nav2 config files
    map_yaml = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("robot_bringup"),
            "maps",
            "obstacles_world.yaml",
        ),
    )

    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
                "map",
                default_value=map_yaml,
                description="Full path to map file to load",
            )
    ) 
            
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    
    nav2_parameters_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("robot_bringup"),
            "config",
            "nav2_params.yaml",
        ),
    )

    # Nav2 bringup launch file
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": nav2_parameters_file,
        }.items(),
    )
    rviz_config = os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'navigation.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
    )

    nodes = [
        nav2,
        rviz
    ]

    return LaunchDescription(launch_arguments + nodes)
