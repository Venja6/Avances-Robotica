"""
optimize.launch.py
-------------------
Lanza el nodo optimizer_node de trajectory_recorder.

Uso:
    ros2 launch trajectory_recorder optimize.launch.py
    ros2 launch trajectory_recorder optimize.launch.py \
        filename:=vuelta1.json min_segment_distance:=0.1 simplify_epsilon:=0.03
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_dir_arg = DeclareLaunchArgument(
        'input_dir',
        default_value=os.path.expanduser('~/turtlebot3_ws/trajectories'),
        description='Carpeta donde está la trayectoria cruda de entrada.',
    )
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value='trajectory.json',
        description='Trayectoria cruda grabada por recorder.py.',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/turtlebot3_ws/trajectories'),
        description='Carpeta donde guardar la ruta optimizada.',
    )
    output_filename_arg = DeclareLaunchArgument(
        'output_filename',
        default_value='trajectory_optimized.json',
        description='Nombre del archivo de salida con los waypoints.',
    )
    min_segment_distance_arg = DeclareLaunchArgument(
        'min_segment_distance',
        default_value='0.05',
        description='Longitud mínima (m) para conservar un segmento.',
    )
    simplify_epsilon_arg = DeclareLaunchArgument(
        'simplify_epsilon',
        default_value='0.0',
        description='Epsilon (m) para Douglas-Peucker. 0 desactiva el paso.',
    )
    auto_run_arg = DeclareLaunchArgument(
        'auto_run',
        default_value='true',
        description='Optimiza automáticamente al arrancar el nodo.',
    )

    optimizer_node = Node(
        package='trajectory_recorder',
        executable='optimizer_node',
        name='trajectory_optimizer_node',
        output='screen',
        parameters=[{
            'input_dir': LaunchConfiguration('input_dir'),
            'filename': LaunchConfiguration('filename'),
            'output_dir': LaunchConfiguration('output_dir'),
            'output_filename': LaunchConfiguration('output_filename'),
            'min_segment_distance': LaunchConfiguration('min_segment_distance'),
            'simplify_epsilon': LaunchConfiguration('simplify_epsilon'),
            'auto_run': LaunchConfiguration('auto_run'),
        }],
    )

    return LaunchDescription([
        input_dir_arg,
        filename_arg,
        output_dir_arg,
        output_filename_arg,
        min_segment_distance_arg,
        simplify_epsilon_arg,
        auto_run_arg,
        optimizer_node,
    ])
