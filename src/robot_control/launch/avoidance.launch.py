from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('robot_control'), 'config', 'avoidance.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_control',
            executable='avoidance_controller',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool,
                    ),
                },
            ],
        ),
    ])
