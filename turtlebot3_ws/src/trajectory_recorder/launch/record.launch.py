"""
record.launch.py
-----------------
Lanza el nodo recorder_node de trajectory_recorder.

Uso:
    ros2 launch trajectory_recorder record.launch.py
    ros2 launch trajectory_recorder record.launch.py filename:=vuelta1.json record_rate:=20.0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value='trajectory.json',
        description='Nombre del archivo JSON donde se guardará la trayectoria.',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/turtlebot3_ws/trajectories'),
        description='Carpeta donde se guardará el archivo de trayectoria.',
    )
    record_rate_arg = DeclareLaunchArgument(
        'record_rate',
        default_value='10.0',
        description='Frecuencia de muestreo en Hz.',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Tópico de velocidad comandada a grabar.',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Tópico de odometría a grabar.',
    )

    recorder_node = Node(
        package='trajectory_recorder',
        executable='recorder_node',
        name='trajectory_recorder_node',
        output='screen',
        parameters=[{
            'filename': LaunchConfiguration('filename'),
            'output_dir': LaunchConfiguration('output_dir'),
            'record_rate': LaunchConfiguration('record_rate'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
    )

    return LaunchDescription([
        filename_arg,
        output_dir_arg,
        record_rate_arg,
        cmd_vel_topic_arg,
        odom_topic_arg,
        recorder_node,
    ])
