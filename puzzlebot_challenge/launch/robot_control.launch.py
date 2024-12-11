from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('sim', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='puzzlebot',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg_puzzlebot_description = get_package_share_directory('puzzlebot_challenge')
    namespace = LaunchConfiguration('namespace')

    control = Node(
        package='puzzlebot_challenge',
        executable='robot_control',
        name='robot_control'
    )
    
    bug2 = Node(
        package='puzzlebot_challenge',
        executable='bug2',
        name='Bug2'
    )

    handle = Node(
        package='puzzlebot_challenge',
        executable='handle_object',
        name='handle_object'
    )

    # condition=IfCondition(LaunchConfiguration('rviz')),


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(control)
    ld.add_action(bug2)
    ld.add_action(handle)
    return ld