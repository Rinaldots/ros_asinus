# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world_file = LaunchConfiguration('world_file', default='empty.world')
    pkg_asinus_description = FindPackageShare('asinus_description').find('asinus_description')
    pkg_asinus_gz = FindPackageShare('asinus_gz').find('asinus_gz')
    # Add the mesh/model path for asinus_description to the GAZEBO/GZ resource paths
    asinus_mesh_path = os.path.join(pkg_asinus_description, 'meshes')
    asinus_model_path = pkg_asinus_description  # The package root may contain model.config etc.

    robot_name_for_config = "asinus"  # Assuming 'asinus' is the base name for config files

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('asinus_gz'),
            'urdf',
            'asinus_gz.xacro'
        ]),
        ' yaml_config_dir:=', os.path.join(pkg_asinus_description, 'config', robot_name_for_config), # Use a consistent name for config path
    ])

    params = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'prefix': "", # Set to empty string as link names in URDF will be prefixed
    }
    
    # Set up environment variables for Gazebo resource paths
    gz_resource_path = ':'.join([
        os.path.join(pkg_asinus_description, '..'),
        os.path.join(pkg_asinus_gz, '..'),
        asinus_model_path,
        asinus_mesh_path,
        '/opt/ros/humble/share'
    ])
    
    # Set environment variables
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = gz_resource_path
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    # Robot state publisher
    node_robot_state_publisher = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[params],
    )
    
    robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("ros2_control_asinus"),
                "config",
                "diffbot_controllers.yaml",
            ]
        )


    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diffbot_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diffbot_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Bridge for clock and camera topics
    bridge_params = PathJoinSubstitution([
        FindPackageShare("asinus_gz"),
        'params',
        'asinus_params.yaml']
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.world',
            description='World file to load in Gazebo'),
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diffbot_base_controller_spawner],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
    ])