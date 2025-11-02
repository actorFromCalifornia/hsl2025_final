from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Получаем пути к пакетам
    webots_bridge_pkg = FindPackageShare('webots_bridge')
    
    # Пути к launch-файлам
    robot_launch_path = PathJoinSubstitution([
        webots_bridge_pkg, 'launch', 'robot_launch.py'
    ])
    
    return LaunchDescription([
        # Опционально: объявить аргументы, которые могут понадобиться
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        # Запуск robot_launch.py из webots_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),
    ])
