from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='puzzlebot',
                          description='Ignition model name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='simple_world',
                          description='World name'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    dock_name = LaunchConfiguration('dock_name')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')

    puzzlebot_gazebo_pkg = get_package_share_directory(
        'puzzlebot_gazebo')
    
    puzzlebot_description_pkg = get_package_share_directory(
        'puzzlebot_challenge'
    )

    leds = [
        'power',
        'motors',
        'comms',
        'wifi',
        'battery',
        'user1',
        'user2'
    ]

    # pkg_irobot_create_ignition_bringup = get_package_share_directory(
    #     'irobot_create_ignition_bringup')

    # create3_ros_gz_bridge_launch = PathJoinSubstitution(
    #     [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])

    # create3_bridge = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([create3_ros_gz_bridge_launch]),
    #     launch_arguments=[
    #         ('robot_name', robot_name),
    #         ('dock_name', dock_name),
    #         ('namespace', namespace),
    #         ('world', world)
    #     ]
    # )

    puzzlebot_description_launch = PathJoinSubstitution(
        [puzzlebot_description_pkg, 'launch', 'puzzlebot_description.launch.py']
    )

    puzzlebot_gz_launch = PathJoinSubstitution(
        [puzzlebot_gazebo_pkg, 'launch', 'ignition.launch.py']
    )

    puzzlebot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([puzzlebot_description_launch])
    )

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([puzzlebot_gz_launch]),
        launch_arguments=[
            # ('namespace', namespace),
            ('world', world)
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_robot_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
                   '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float'
                  ],
        output='screen'
    )
    


    # lidar bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/chassis/sensor/rplidar/scan' +
             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/chassis/sensor/rplidar/scan'],
                'scan')
        ]
    )

    # Camera sensor bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/chassis/sensor/camera/image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/chassis/sensor/camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[ignition.msgs.CameraInfo'],
            ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/chassis/sensor/camera/image'],
             '/video_source/raw'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/chassis/sensor/camera/camera_info'],
             '/camera_info')
            ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(puzzlebot_description)
    ld.add_action(gazebo_simulator)
    ld.add_action(bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(camera_bridge)
    return ld