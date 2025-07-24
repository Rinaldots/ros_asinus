import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Helper to ensure config path uses the base robot name, not the namespaced one
    robot_name_for_config = "asinus" # Assuming 'asinus' is the base name for config files
    
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Namespace for the robot. Empty for no namespace.')
    namespace = LaunchConfiguration('namespace')
    # Obtains asinus_description's share directory path.
    pkg_asinus_description = get_package_share_directory('asinus_description')
    # Obtain urdf from xacro files using a default value for robot_name_arg.
    from launch.substitutions import Command, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('asinus_description'),
            'urdf',
            'asinus.urdf.xacro'
        ]),
        ' yaml_config_dir:=', os.path.join(pkg_asinus_description, 'config', robot_name_for_config), # Use a consistent name for config path
        ' frame_prefix_arg:=', namespace
    ])
    params = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'prefix': "", # Set to empty string as link names in URDF will be prefixed
    }

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[params],
                remappings={('/robot_description', 'robot_description'),}
    )

    # NOTE: Joint state publisher removed as it conflicts with motor hub controllers
    # The motor hub joints are continuous and should be controlled by the hardware interface
    # or diff_drive_controller, not reset to zero position by joint_state_publisher

    return LaunchDescription([
        namespace_arg,
        rsp,
        # jsp  # Commented out to prevent TF conflicts with motor hubs
    ])
