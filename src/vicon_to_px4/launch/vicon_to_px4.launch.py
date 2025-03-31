from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # VRPN Mocap Client - using YAML launch file
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('vrpn_mocap'),
                    'launch/client.launch.yaml'
                ])
            ]),
            launch_arguments={
                'server': LaunchConfiguration('server', default='192.168.0.2'),
                'port': LaunchConfiguration('port', default='3883')
            }.items()
        ),

        # Micro XRCE-DDS Agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '-b', '921600', '--dev', '/dev/ttyUSB0'],
            output='screen'
        ),

        # Vicon to PX4 converter node
        Node(
            package='vicon_to_px4',
            executable='vicon_to_px4',
            name='vicon_to_px4',
            parameters=[{
                'pose_topic': PathJoinSubstitution([
                    '/vrpn_mocap/',
                    LaunchConfiguration('tracker_name', default='Obj1'),
                    '/pose'
                ])
            }],
            output='screen'
        )
    ])
