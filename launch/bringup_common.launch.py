import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    bringup_share = get_package_share_directory('solution_bringup')

    default_rtabmap_params = os.path.join(bringup_share, 'config', 'rtabmap_params.yaml')
    default_nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    navigation_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('solution_bringup'),
            'launch',
            'navigation.launch.py',
        ])
    )
    
    coverage_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('solution_bringup'),
            'launch',
            'coverage.launch.py',
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
            navigation_launch,
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rtabmap_params_file': rtabmap_params_file,
                'nav2_params_file': nav2_params_file,
            }.items(),
        ),
        Node(
            package='solution_bringup',
            executable='fixed_goal_node',
            name='fixed_goal_setter',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'goal_x': 0.0,
                    'goal_y': 3.5,
                    'goal_yaw': 0.0,
                    'max_attempts': 100,
                    'retry_radius': 0.1,
                }
            ],
        ),
        Node(
            package='mark_detector',
            executable='mark_detector_node',
            name='mark_detector_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
