from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensors_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('kobuki_bringup'),
            'launch',
            'sensors.launch.py',
        ])
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'bringup_common.launch.py',
                ])
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            sensors_launch,
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),
    ])
