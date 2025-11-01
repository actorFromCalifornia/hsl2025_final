from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'bringup_common.launch.py',
                ])
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])
