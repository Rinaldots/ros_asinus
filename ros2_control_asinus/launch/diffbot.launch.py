from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('asinus_description'),
            'urdf',
            'asinus.urdf.xacro'
        ]),
        ' yaml_config_dir:=', PathJoinSubstitution([
            FindPackageShare('asinus_description'),
            'config',
            'asinus'  # seu nome do robô aqui
        ]),
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_asinus"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("asinus_description"), "rviz", "diffbot_view.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "controller_manager"],  # removido /
    
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz automatically"),
        robot_state_pub_node,
        robot_controller_spawner,
        control_node,
        rviz_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        
    ])
