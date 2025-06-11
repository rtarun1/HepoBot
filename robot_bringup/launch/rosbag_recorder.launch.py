# ROS2 Launch file for rosbag recording of topics listed in a yaml file
from datetime import datetime
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from yaml import safe_load

def generate_launch_description():
    # Get path to package
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    # Get path to config file
    config_dir = os.path.join(robot_bringup_dir, 'config')
    rosbag_record_yaml = os.path.join(config_dir, 'rosbag_record.yaml')
    topics_for_recording = safe_load(open(rosbag_record_yaml, 'r'))['topics']
    
    recordings_path = os.path.join('/records')
    
    # Get path to rosbag file
    # Name of rosbag file contains the timestamp of when the rosbag was started
    timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    folderName = 'rosbag_' + timestamp
    rosbag_dir = os.path.join(recordings_path, folderName)
    
    # Declare launch arguments
    rosbag_storage_dir_arg = DeclareLaunchArgument(
        'rosbag_storage_dir', 
        default_value=rosbag_dir, 
        description='Path to rosbag file'
    )
    
    rosbag_storage_dir = LaunchConfiguration('rosbag_storage_dir')
    rosbag_cmd = ['ros2', 'bag', 'record']
    
    if topics_for_recording:
        for topic in topics_for_recording:
            rosbag_cmd.append(topic)
        rosbag_cmd.extend([
            '-o', rosbag_storage_dir,
            '-s', 'sqlite3', 
        ])
    
    start_rosbag_yaml = ExecuteProcess(
        cmd=rosbag_cmd,
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(rosbag_storage_dir_arg)
    ld.add_action(start_rosbag_yaml)
    
    return ld