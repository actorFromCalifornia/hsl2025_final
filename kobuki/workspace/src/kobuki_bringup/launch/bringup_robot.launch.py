import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('kobuki_bringup')

    default_rtabmap_params = os.path.join(bringup_share, 'config', 'rtabmap_params.yaml')
    default_nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    sensors_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('kobuki_bringup'),
            'launch',
            'sensors.launch.py',
        ])
    )

    navigation_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('kobuki_bringup'),
            'launch',
            'navigation.launch.py',
        ])
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if available.'
        ),
        DeclareLaunchArgument(
            'rtabmap_params_file',
            default_value=default_rtabmap_params,
            description='Full path to the RTAB-Map parameters file.'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Full path to the Nav2 parameters file.'
        ),
        IncludeLaunchDescription(
            sensors_launch,
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            navigation_launch,
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rtabmap_params_file': rtabmap_params_file,
                'nav2_params_file': nav2_params_file,
            }.items(),
        ),
    ])
