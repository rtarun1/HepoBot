import os
import datetime
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    Shutdown,
    TimerAction,
)
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    launch_arguments = []
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Launch in simulation mode'
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Use RViz'
        )
    )
    launch_arguments.append(
        DeclareLaunchArgument(
            'record',
            default_value='False',
            description='Record in rosbag'
        )
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration('use_rviz')
    record = LaunchConfiguration("record")

    # set log output path
    get_current_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_full_path = os.path.join('/home/container_user/ros2_ws/src/records/', get_current_timestamp)
    rosbag_full_path = os.path.join(log_full_path, 'rosbag')
    package_path = get_package_share_directory('robot_bringup')

    # ROBOT DESCRIPTION
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robot_bringup"),
                            "urdf",
                            "robot_core.urdf.xacro",
                        ]
                    ),
                    " use_sim_time:=",
                    use_sim_time,
                ]
            ),
            value_type=str,
        )
    }

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, 
                    {"use_sim_time": use_sim_time}],
    )

    # CONTROLLER MANAGER NODE
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_bringup"),
                    "config",
                    "robot_controllers.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        remappings=[
            # ("/cmd_vel", "/robot_base_controller/cmd_vel_unstamped"),
            ("/robot_base_controller/odom", "/odom"),
        ],
        condition=UnlessCondition(use_sim_time),
    )
    
    default_world = os.path.join(
        get_package_share_directory('robot_bringup'),
        'worlds',
        'obstacles.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
                    condition=IfCondition(use_sim_time)
             )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'robot',
                                   '-z', '0.2'],
                        parameters=[{'use_sim_time': use_sim_time}],
                        condition=IfCondition(use_sim_time),
                        output='screen')

   # JOINT STATE BROADCASTER
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    # ROBOT CONTROLLER
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    bridge_params = os.path.join(get_package_share_directory('robot_bringup'), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time),
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # delay start of joint_state_broadcaster_spawner after ros2 control 
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher,
                    on_start=[joint_state_broadcaster_spawner],
                )
            )
    
    #delay start of robot_controller_spawner after joint_state_broadcaster_spawner
    delay_robot_controller_spawner = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_controller_spawner],
                )
            )
    joy_params = os.path.join(get_package_share_directory('robot_bringup'),'config','joystick.yaml')
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            # remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
    
    twist_mux_params = os.path.join(get_package_share_directory('robot_bringup'),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/robot_base_controller/cmd_vel_unstamped')],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', get_package_share_directory('robot_bringup') + '/rviz/robot.rviz'],
        on_exit=Shutdown(),
        condition=IfCondition(use_rviz),
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a3_launch.py"]
            ),
        ),
        launch_arguments={"frame_id": "laser_frame"}.items(),
        condition=UnlessCondition(use_sim_time),
    )

    angle_range_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_bringup"),
                    "config",
                    "angle_range_filter.yaml",
                ]
            )
        ],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
        # condition=UnlessCondition(use_sim_time),
    )

    rosbag_recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_path, '/launch/rosbag_recorder.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rosbag_storage_dir': rosbag_full_path,
        }.items(),
        condition=IfCondition(record),
    )

    rosbag_with_delay = TimerAction(
        period=10.0, 
        actions=[rosbag_recorder_launch],
        condition=IfCondition(record),
    )
    
    
    nodes = [
            robot_state_publisher,
            ros2_control,
            delay_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner,
            sllidar_launch,
            angle_range_filter_node,
            world_arg,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
            joy_node,
            teleop_node,
            twist_mux,
            rviz_node,
            rosbag_with_delay,
    ]
    
    return LaunchDescription(launch_arguments + nodes)