from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a3_launch.py"]
            ),
        ),
        launch_arguments={"frame_id": "laser_frame"}.items(),
    )

    angle_range_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_navigation"),
                    "config",
                    "angle_range_filter.yaml",
                ]
            )
        ],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )
    

    return LaunchDescription(
        [
            sllidar_launch,
            angle_range_filter_node,
        ]
    )
