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
        parameters=[],
    )
    
    return LaunchDescription([
        coverage_node,
    ])

