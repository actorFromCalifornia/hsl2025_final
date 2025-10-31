import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_params(package_name: str, relative_path: str, key: str):
    params_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(params_path, 'r', encoding='utf-8') as params_file:
        data = yaml.safe_load(params_file)
    return data[key]['ros__parameters']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    kobuki_params = _load_params('kobuki_node', 'config/kobuki_node_params.yaml', 'kobuki_ros_node')
    cmd_vel_params = _load_params('cmd_vel_mux', 'config/cmd_vel_mux_params.yaml', 'cmd_vel_mux')

    kobuki_node = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        name='kobuki_ros_node',
        output='screen',
        parameters=[kobuki_params, {'use_sim_time': use_sim_time}],
    )

    cmd_vel_mux_node = Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux',
        output='screen',
        parameters=[cmd_vel_params, {'use_sim_time': use_sim_time}],
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch',
                'mid360.launch.py',
            ])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock for all nodes.'
        ),
        livox_launch,
        kobuki_node,
        cmd_vel_mux_node,
    ])
