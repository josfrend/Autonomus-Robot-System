import os
import yaml

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
    DeclareLaunchArgument('world', default_value='test_ground',
                          description='Ignition World'),
]


def generate_launch_description():

    # Directories
    pkg_gazebo = get_package_share_directory(
        'puzzlebot_gazebo')
    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_gz_sim')
    
    yaml_file_path = os.path.join(pkg_gazebo, 'config', 'gazebo.yaml')

    with open(yaml_file_path, 'r') as yaml_file:
        params = yaml.safe_load(yaml_file)
    
    gazebo_path = params['parameters']['gazebo_model_path']

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_gazebo, 'gazebo') + ':' + 
            gazebo_path + ':' +
            '$GZ_SIM_RESOURCE_PATH'])
    
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(pkg_gazebo, 'gazebo/plugins'), ':' + '$GZ_SIM_SYSTEM_PLUGIN_PATH'])

    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'gz_sim.launch.py'])

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