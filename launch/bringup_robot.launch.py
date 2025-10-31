from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensors_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('kobuki_bringup'),
            'launch',
            'sensors.launch.py',
        ])
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'bringup_common.launch.py',
                ])
            )
        ),
        IncludeLaunchDescription(
            sensors_launch,
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
