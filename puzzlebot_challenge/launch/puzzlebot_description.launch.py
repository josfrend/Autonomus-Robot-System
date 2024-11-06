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
    urdf_file = get_package_share_directory('puzzlebot_challenge') + '/urdf/puzzlebot.urdf'#PathJoinSubstitution([pkg_puzzlebot_description,
    #                                    'urdf',
                                    #    'simple.urdf'])
    rviz_config_path = get_package_share_directory('puzzlebot_challenge') + '/rviz/manipulator.rviz'
    namespace = LaunchConfiguration('namespace')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': open(urdf_file).read()},
        ],
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static')
        # ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    tf2_ros = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_joint',
            arguments=['odomPose_x', 'odomPose_y', 'odomPose_z', 'odomPose_roll', 'odomPose_pitch', 'odomPose_yaw', 'map', 'odom']
        )

    joint_state_publisher = Node(
        package='puzzlebot_challenge',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        # parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static')
        # ]
    )

    pose_sim = Node(
            package='puzzlebot_challenge',
            executable='pose_sim',
            name='puzzlebot_kinematics'
    )

    localisation = Node(
            package = 'puzzlebot_challenge',
            executable = 'odometry',
            name = 'localisation'
    )

    # condition=IfCondition(LaunchConfiguration('rviz')),


    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(tf2_ros)
    ld.add_action(joint_state_publisher)
    ld.add_action(pose_sim)
    ld.add_action(localisation)
    return ld