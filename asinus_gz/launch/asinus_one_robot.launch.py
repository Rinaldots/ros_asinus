"""Launch Gazebo with a world that has Asinus."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    use_gazebo_ros_control = LaunchConfiguration('use_gazebo_ros_control')
    world_file = LaunchConfiguration('world_file')

    # Paths
    pkg_asinus_gz = get_package_share_directory('asinus_gz')
    pkg_asinus_description = get_package_share_directory('asinus_description')
    
    # Default world file
    default_world_file = os.path.join(pkg_asinus_gz, 'worlds', 'empty.world')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='asinus',
        description='Robot namespace'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    use_gazebo_ros_control_arg = DeclareLaunchArgument(
        'use_gazebo_ros_control',
        default_value='true',
        description='Use Gazebo ROS Control plugin'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_file,
        description='World file to load'
    )

    # Description of the asinus - use direct xacro processing for Gazebo
    from launch.substitutions import Command, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare
    from launch_ros.parameter_descriptions import ParameterValue
    
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('asinus_gz'),
            'urdf',
            'asinus_gz.xacro'
        ]),
        ' namespace:=', namespace,
        ' use_gazebo_ros_control:=', use_gazebo_ros_control
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Gazebo launch (Ignition Gazebo)
    gazebo_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v4', LaunchConfiguration('world_file')],
        output='screen',
        additional_env={
            'IGN_GAZEBO_RESOURCE_PATH': ':'.join([
                os.path.join(pkg_asinus_description, '..', '..'),  # Workspace share
                '/opt/ros/humble/share'  # Path padrão do ROS
            ])
        }
    )

    # Robot spawn
    robot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/asinus/robot_description',
            '-name', 'asinus_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        namespace=namespace,
        output='screen'
    )

    # Diff Drive Controller Spawner
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_base_controller'],
        namespace=namespace,
        output='screen'
    )
            

    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_asinus_gz, 'rviz', 'asinus_gz.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        use_rviz_arg,
        use_gazebo_ros_control_arg,
        world_file_arg,
        robot_state_publisher,
        gazebo_launch,
        robot_spawn,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        rviz_node,
    ])
