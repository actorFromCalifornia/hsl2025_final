from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node


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
        # Node(
        #     package='solution_bringup',
        #     executable='fixed_goal_node',
        #     name='fixed_goal_setter',
        #     output='screen',
        #     parameters=[
        #         {
        #             'use_sim_time': True,
        #             'goal_x': 0.0,
        #             'goal_y': 2.5,
        #             'goal_yaw': 0.0,
        #             'max_attempts': 100,
        #             'retry_radius': 0.1,
        #         }
        #     ],
        # ),
    ])
