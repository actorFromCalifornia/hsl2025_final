from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch файл для запуска Grid-Based Coverage ноды.
    
    Запускает ноду coverage_node, которая будет:
    1. Подписываться на /rtabmap_map (карта)
    2. Подписываться на /odom (позиция робота)
    3. Использовать Nav2 для навигации к целям
    """
    
    coverage_node = Node(
        package='coverage_node',
        executable='coverage_node',
        name='coverage_node',
        output='screen',
        parameters=[{
            'enable_marker_handling': True,
            'enable_cell_abort': False,
            'grid_origin_offset_x': 0.25,
            'grid_origin_offset_y': 1.5,
        }],
    )
    
    return LaunchDescription([
        coverage_node,
    ])

