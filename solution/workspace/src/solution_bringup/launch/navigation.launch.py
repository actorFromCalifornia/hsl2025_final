import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('solution_bringup')

    default_rtabmap_params = os.path.join(bringup_share, 'config', 'rtabmap_params.yaml')
    default_nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    default_pointcloud_to_laserscan_params = os.path.join(
        bringup_share, 'config', 'pointcloud_to_laserscan.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    pointcloud_to_laserscan_params_file = LaunchConfiguration(
        'pointcloud_to_laserscan_params_file'
    )

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
        DeclareLaunchArgument(
            'pointcloud_to_laserscan_params_file',
            default_value=default_pointcloud_to_laserscan_params,
            description='Full path to the pointcloud_to_laserscan parameters file.'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[pointcloud_to_laserscan_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ],
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', '/scan'),
                ('map', '/rtabmap_map'),
                ('imu', '/livox/imu'),
                ('odom_info', '/rtabmap/odom_info'),
                ('odom', '/icp_odom'),
            ],
            arguments=['--delete_db_on_start'],
        ),
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[rtabmap_params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/livox/imu'),
                ('odom', '/icp_odom'),
            ]
        ),
        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        #     remappings=[
        #         ('/cmd_vel', '/commands/velocity'),
        #         ('/odom', '/odom'),
        #         ('/map', '/rtabmap_map'),
        #     ],
        # ),
        # Node(
        #     package='nav2_smoother',
        #     executable='smoother_server',
        #     name='smoother_server',
        #     output='screen',
        #     parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # ),
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        #     remappings=[('/map', '/rtabmap_map')],
        # ),
        # Node(
        #     package='nav2_behaviors',
        #     executable='behavior_server',
        #     name='behavior_server',
        #     output='screen',
        #     parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # ),
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        # ),
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'autostart': True,
        #         'node_names': [
        #             'controller_server',
        #             'planner_server',
        #             'smoother_server',
        #             'behavior_server',
        #             'bt_navigator',
        #         ],
        #     }],
        # ),
    ])
