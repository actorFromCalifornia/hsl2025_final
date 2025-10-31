from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


BAG_MODE = 'bag'
ROBOT_MODE = 'robot'


LAUNCH_ROOT = Path(__file__).resolve().parent


def _include_selected_launch(context):
    mode = LaunchConfiguration('mode').perform(context)
    launch_arguments = {
        name: value
        for name, value in context.launch_configurations.items()
        if name != 'mode'
    }

    launch_filename = 'bringup_bag.launch.py' if mode == BAG_MODE else 'bringup_robot.launch.py'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(LAUNCH_ROOT / launch_filename)),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value=ROBOT_MODE,
            description='Select bringup mode: robot (default) or bag player.',
        ),
        OpaqueFunction(function=_include_selected_launch),
    ])
