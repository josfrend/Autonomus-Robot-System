import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='world',
                          description='Ignition World'),
]


def generate_launch_description():

    # Directories
    pkg_name = get_package_share_directory(
        'puzzlebot_gazebo')
    # pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
    #     'turtlebot4_ignition_gui_plugins')
    # pkg_turtlebot4_description = get_package_share_directory(
    #     'turtlebot4_description')
    # pkg_irobot_create_description = get_package_share_directory(
    #     'irobot_create_description')
    # pkg_irobot_create_ignition_bringup = get_package_share_directory(
    #     'irobot_create_ignition_bringup')
    # pkg_irobot_create_ignition_plugins = get_package_share_directory(
    #     'irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')
    

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_name, 'gazebo') + ':' + '/home/alfredo/ros2_ws/src/puzzlebot/puzzlebot_gazebo/gazebo'])
    
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(pkg_name, 'gazebo/plugins'), ':' + '$IGN_GAZEBO_SYSTEM_PLUGIN_PATH'])

    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                          '.sdf',
                          ' -r',
                          ' -v 4'])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)
    return ld