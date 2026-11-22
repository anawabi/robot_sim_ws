
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Find the installed robot_description package.
    package_share = get_package_share_directory(
        'robot_description'
    )

    # Locate the Xacro model.
    xacro_file = PathJoinSubstitution([
        package_share,
        'urdf',
        'four_wheel_robot.urdf.xacro'
    ])

    # Convert Xacro into URDF XML.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Create robot_state_publisher node.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    odom_tf_broadcaster_node = Node(
        package='robot_description',
        executable='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        odom_tf_broadcaster_node,
    ])
